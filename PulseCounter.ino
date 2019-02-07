#include <NTPClient.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>
#include "ESP8266WiFi.h"

// PST offset is 8 hours, or -28800
// Recommended to use 0 if you're using epoch time
// programatically
#define NTP_OFFSET -28800

#define AUTO_INCREMENT_DM 1
#define AUTO_INCREMENT_SECONDS 5

#define DM_COUNT 0
#define DM_TIME 1
#define DM_WIFI 2
#define DM_MQTT 3
#define DM_MAX 3

#define COUNTER_ID "1"

const byte pulsePin = D7;
const byte modePin = D6;
volatile byte rawPulseCounter = 0;
volatile byte rawModeCounter = 0;
int numPulses = 0;

int currentDisplayMode = DM_COUNT;

const char* mqttServer = "192.168.0.127";
const int mqttPort = 1883;
const char* mqttUser = "automation";
const char* mqttPass = "anncoulter";

const char* statusTopic = "pulse/"COUNTER_ID"/available";
const char* subscribeTopic = "pulse/"COUNTER_ID"/#";
const char* countTopic = "pulse/"COUNTER_ID"/count";
const char* setCountTopic = "pulse/"COUNTER_ID"/setcount";
const char* setModeTopic = "pulse/"COUNTER_ID"/setmode";

void mqttCallback(char* topic, byte* payload, unsigned int length);

LiquidCrystal_I2C lcd(0x27, 16, 2);
WiFiClient espClient;
PubSubClient client(espClient);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_OFFSET);

void setup() {
  lcd.begin(16,2);
  lcd.init();
  lcd.clear();
  lcd.backlight();

  setupInterrupts();
  setupWifi();
  setupNtp();
  setupMqtt();
  // Clear all the setup junk away
  lcd.clear();
}

void setupInterrupts() {
  pinMode(pulsePin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pulsePin), handlePulse, FALLING);

  pinMode(modePin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(modePin), handleMode, FALLING);
}

void handlePulse() {
  rawPulseCounter++;
}

void handleMode() {
  rawModeCounter++;
}

void setupNtp() {
  lcdmsg("Connecting NTP..");
  timeClient.begin();
}

void setupWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  // Set up Wifi
  WiFi.begin("The Holy Trinity", "yassqueen");
  while(WiFi.status()  != WL_CONNECTED) {
    lcdmsg("Connecting to Wifi");
    delay(500);
  }
  lcdmsg("Connected");
}

void setupMqtt() {
  // Set up MQTT
  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqttCallback);
  
  while (!client.connected()) {
    lcdmsg("Connecting to MQTT");

    // The "offline" message here is the last will and testament
    if (client.connect("Pulse1", mqttUser, mqttPass, statusTopic, 1, 1, "offline")) {
      lcdmsg("Connected");
      if (client.subscribe(subscribeTopic)) {
        lcdmsg("Subscribed");
        client.loop();
        client.loop();
      }
    } else {    
      lcdmsg("MQTT Failed");
      delay(500);
    }
  }
  client.publish(statusTopic, "online");
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char* p = (char*)malloc(length + 1);
  // Copy the payload to the new buffer
  memcpy(p,payload,length);
  p[length] = '\0';
  
  //lcdmsg(p);
  if (strcmp(topic, setCountTopic) == 0) {
    String a = "";
    a += p;
    numPulses = a.toInt();
  }

  if (strcmp(topic, setModeTopic) == 0) {
    String a = "";
    a += p;
    setDisplayMode(a.toInt());
  }
}

void lcdClearLine(int line) {
  lcd.setCursor(0, line);
  lcd.print("                ");
}

void lcdmsg(String msg) {
  //lcdClearLine(0);
  lcd.setCursor(0,0);
  lcd.print(msg);
}

void lcd2(String msg) {
  //lcdClearLine(1);
  lcd.setCursor(0,1);
  lcd.print(msg);
}

void nextDisplayMode() {
  currentDisplayMode++;
  // When we're transitioning modes, clear the display.
  lcdClearLine(0);
  lcdClearLine(1);
  if (currentDisplayMode > DM_MAX) {
    currentDisplayMode = 0;
  }
}

void setDisplayMode(int mode) {
  currentDisplayMode = mode;
  lcdClearLine(0);
  lcdClearLine(1);
  if (currentDisplayMode > DM_MAX) {
    currentDisplayMode = 0;
  }
}

void loop() {

  // Handle queued interrupts for pulses
  if (rawPulseCounter > 0) {
    rawPulseCounter--;
    numPulses++;
  }

  // Handle requests to change display mode
  if (rawModeCounter > 0) {
    rawModeCounter --;
    // change mode
    nextDisplayMode();
  }
  
  // Check wifi health and re-join if we need to
  if (WiFi.status() != WL_CONNECTED) {
    setupWifi();
  }

  // Check mqtt health
  if (!client.connected()) {
    setupMqtt();
  }

  // handle MQTT things
  client.loop();
  timeClient.update();

  if (currentDisplayMode == DM_TIME) {
    lcdmsg("Current Time");
    lcd2(timeClient.getFormattedTime());
  }

  if (currentDisplayMode == DM_COUNT) {
    char countStr[16];
    sprintf(countStr, "Count: %4d", numPulses);
    lcdmsg(countStr);

    char gallonsStr[16];
    sprintf(gallonsStr, "Gallons: %f", numPulses * .75);
    lcd2(gallonsStr);
  }

  if (currentDisplayMode == DM_WIFI) {
    if (WiFi.status() == WL_CONNECTED) {
      lcdmsg("WIFI: Connected");

      char ipStr[16];
      sprintf(ipStr, "IP:%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
      lcd2(ipStr);
    } else {
      lcdmsg("WIFI: DISCONNECTED");
    }
  }

  if (currentDisplayMode == DM_MQTT) {
    int state = client.state();
    char mqttStr[16];

    switch (state) {
      case MQTT_CONNECTED:
        lcdmsg("MQTT: CONNECTED");
        lcd2(subscribeTopic);
      break;
      case MQTT_CONNECTION_TIMEOUT:
        lcdmsg("MQTT: TIMEOUT");
      break;
      case MQTT_CONNECTION_LOST:
        lcdmsg("MQTT: CONN LOSS");
      break;
      case MQTT_CONNECT_FAILED:
        lcdmsg("MQTT: CONN FAIL");
      break;
      case MQTT_DISCONNECTED:
        lcdmsg("MQTT: DISCONNECTED");
      break;
      case MQTT_CONNECT_BAD_PROTOCOL:
        lcdmsg("MQTT: BAD PROTO");
      break;
      case MQTT_CONNECT_BAD_CLIENT_ID:
        lcdmsg("MQTT: BAD ID");
      break;
      case MQTT_CONNECT_UNAVAILABLE:
        lcdmsg("MQTT: UNAVAILABLE");
      break;
      case MQTT_CONNECT_BAD_CREDENTIALS:
        lcdmsg("MQTT: BAD CREDS");
      break;
      case MQTT_CONNECT_UNAUTHORIZED:
        lcdmsg("MQTT: UNAUTH");
      break;      
    }
  }

  // every 2 seconds, change display mode
  static unsigned long lastRefreshTime = 0;
  if(timeClient.getEpochTime() - lastRefreshTime >= AUTO_INCREMENT_SECONDS)
  {
    lastRefreshTime = timeClient.getEpochTime();
    if (AUTO_INCREMENT_DM) {
      nextDisplayMode();
    }
  }
}
