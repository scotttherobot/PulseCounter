#include <NTPClient.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>
#include "ESP8266WiFi.h"
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>

// PST offset is 8 hours, or -28800
// Recommended to use 0 if you're using epoch time
// programatically
#define NTP_OFFSET -28800

#define AUTO_INCREMENT_DM 1
#define AUTO_INCREMENT_SECONDS 5
#define AUTO_INCREMENT_PAUSE_SECONDS 30

// Interval to turn the backlight off
#define BACKLIGHT_TIMEOUT 60

// Interval to publish an "online" message
#define HEALTHCHECK_TIMEOUT 60

#define DM_COUNT 0
#define DM_TIME 1
#define DM_WIFI 2
#define DM_MQTT 3
#define DM_RELAY 4
#define DM_MAX 4

#define DEVICE_ID "1"
#define DEVICE_NAME "pulse"

// The various pins that we use
const byte pulsePin = D7;
const byte modePin = D6;
const byte relay1Pin = D0;

// Some counters/state variables
volatile byte rawPulseCounter = 0;
volatile byte rawModeCounter = 0;
int numPulses = 0;
bool relay1State = false;

unsigned long lastUserInteraction = 0;
unsigned long lastHealthcheck = 0;
unsigned long lastDisplayModeChange = 0;
int pauseAutoIncrement = 0;

int currentDisplayMode = DM_COUNT;

const char* mqttServer = "192.168.0.127";
const int mqttPort = 1883;
const char* mqttUser = "automation";
const char* mqttPass = "anncoulter";

const char* statusTopic = DEVICE_NAME"/"DEVICE_ID"/available";
const char* subscribeTopic = DEVICE_NAME"/"DEVICE_ID"/#";
const char* countTopic = DEVICE_NAME"/"DEVICE_ID"/count";
const char* setCountTopic = DEVICE_NAME"/"DEVICE_ID"/setcount";
const char* setModeTopic = DEVICE_NAME"/"DEVICE_ID"/setmode";
const char* jsonInfoTopic = DEVICE_NAME"/"DEVICE_ID"/attributes";
const char* relay1Topic = DEVICE_NAME"/"DEVICE_ID"/relay1";

void mqttCallback(char* topic, byte* payload, unsigned int length);

LiquidCrystal_I2C lcd(0x27, 16, 2);
WiFiClient espClient;
PubSubClient client(espClient);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_OFFSET);

void setup() {
  // Start serial for debugging
  Serial.begin(9600);
  Serial.setTimeout(50);

  lcd.begin(16, 2);
  lcd.init();
  lcd.clear();
  lcd.backlight();

  setupInterruptsAndPins();
  setupWifi();
  setupNtp();
  setupMqtt();
  // Clear all the setup junk away
  lcd.clear();
  lastUserInteraction = timeClient.getEpochTime();

  //ArduinoOTA.setHostname("pulseCounter");
  ArduinoOTA.begin();
}

void setupInterruptsAndPins() {
  pinMode(relay1Pin, OUTPUT);
  digitalWrite(relay1Pin, LOW);

  pinMode(pulsePin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pulsePin), handlePulse, FALLING);

  pinMode(modePin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(modePin), handleMode, FALLING);
}

void handlePulse() {
  static unsigned long last_int_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_int_time > 200) {
    rawPulseCounter++;
  }

  last_int_time = interrupt_time;
}

void handleMode() {
  static unsigned long last_int_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_int_time > 200) {
    rawModeCounter++;
  }

  last_int_time = interrupt_time;
}

void setupNtp() {
  lcdmsg("Connecting NTP..");
  timeClient.begin();
}

void setupWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    // Wifi is already good, let's return.
    return;
  } else {
    // Wifi is not good. Set it up.
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    // Set up Wifi
    WiFi.begin("The Holy Trinity", "yassqueen");
    while (WiFi.status()  != WL_CONNECTED) {
      lcdmsg("Connecting to Wifi");
      delay(500);
    }
    lcdmsg("Connected");
  }
}

void setupMqtt() {
  // Before even attempting to connect to MQTT
  // We need to make sure that Wifi is healthy.
  setupWifi();

  // Set up MQTT
  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqttCallback);

  int attempts = 0;
  while (!client.connected()) {
    lcdmsg("Connecting to MQTT");

    // If we've tried 5 times to reconnect to MQTT then let's just reboot.
    if (attempts == 5) {
      ESP.restart();
    }

    // The "offline" message here is the last will and testament
    if (client.connect(DEVICE_NAME""DEVICE_ID, mqttUser, mqttPass, statusTopic, 1, 1, "offline")) {
      lcdmsg("Connected");
      if (client.subscribe(subscribeTopic)) {
        lcdmsg("Subscribed");
        client.loop();
        client.loop();
      }
    } else {
      attempts++;
      lcdmsg("MQTT Failed");
      delay(500);
      // Something isn't right. Do a Wifi health check.
      setupWifi();
    }
  }
  client.publish(statusTopic, "online");
  publishAttributes();
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char* p = (char*)malloc(length + 1);
  // Copy the payload to the new buffer
  memcpy(p, payload, length);
  p[length] = '\0';

  //lcdmsg(p);
  if (strcmp(topic, setCountTopic) == 0) {
    String a = "";
    a += p;
    numPulses = a.toInt();

    // Consider this an interaction and set the backlight timeout
    lastUserInteraction = timeClient.getEpochTime();
  }

  if (strcmp(topic, setModeTopic) == 0) {
    String a = "";
    a += p;
    setDisplayMode(a.toInt());

    // Consider this an interaction and set the backlight timeout
    lastUserInteraction = timeClient.getEpochTime();
  }

  // If it's on the relay topic
  if (strcmp(topic, relay1Topic) == 0) {
    if (strcmp(p, "ON") == 0) {
      relay1State = true;
    } else {
      relay1State = false;
    }
    publishCount();
  }
}

void lcdClearLine(int line) {
  lcd.setCursor(0, line);
  lcd.print("                ");
}

void lcdmsg(String msg) {
  //lcdClearLine(0);
  lcd.setCursor(0, 0);
  lcd.print(msg);
}

void lcd2(String msg) {
  //lcdClearLine(1);
  lcd.setCursor(0, 1);
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
  lcd.backlight();
  lcdClearLine(1);
  if (currentDisplayMode > DM_MAX) {
    currentDisplayMode = 0;
  }
}

void publishCount() {
  String json = String("{");
  json += "\"count_1\":";
  json += numPulses;
  json += ",\"relay_1\":";
  if (relay1State == true) {
    json += "\"ON\"";
  } else {
    json += "\"OFF\"";
  }
  json += "}";

  char jsonChar[200];
  json.toCharArray(jsonChar, 200);
  client.publish(countTopic, jsonChar);
}

double getFormattedSignalStrength() {
  if (WiFi.status() != WL_CONNECTED) {
    return -1;
  }

  int dBm = WiFi.RSSI();
  if (dBm <= -100) {
    return 0;
  }
  if (dBm >= -50) {
    return 100;
  }
  return 2 * (dBm + 100);
}

/**
   Build and publish a JSON blob with all the attributes
   of the device
   Attributes:
     DEVICE_NAME
     DEVICE_ID
     IP address
     Wifi RSSI
*/
void publishAttributes() {
  char ipStr[16];
  sprintf(ipStr, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );

  String json = String("{");
  json += "\"device_name\":";
  json += "\""DEVICE_NAME"\",";
  json += "\"device_id\":";
  json += "\""DEVICE_ID"\",";
  json += "\"ip_address\":\"";
  json += ipStr;
  json += "\",\"signal_percent\":\"";
  json += getFormattedSignalStrength();
  json += "\"}";

  char jsonChar[200];
  json.toCharArray(jsonChar, 200);
  client.publish(jsonInfoTopic, jsonChar);
}

void serviceSerialPort() {
  if (Serial.available() > 0) {
    String command = Serial.readString();
    command.trim();

    if (command.equals("wifi")) {
      Serial.write("\nwifi status\n");
      String sig = "signal: ";
      sig += getFormattedSignalStrength();
      sig += "%\n";

      if (WiFi.status() == WL_CONNECTED) {
        sig += "connected\n";
      } else if (WiFi.status() == WL_CONNECTION_LOST) {
        sig += "connection lost\n";
      } else if (WiFi.status() == WL_DISCONNECTED) {
        sig += "disconnected\n";
      }


      byte s[sig.length()];
      sig.getBytes(s, sig.length());
      Serial.write(s, sig.length());


    } else if (command.equals("mqtt")) {
      Serial.write("\nmqtt status\n");

      int state = client.state();
      switch (state) {
        case MQTT_CONNECTED:
          Serial.write("MQTT: CONNECTED");
          break;
        case MQTT_CONNECTION_TIMEOUT:
          Serial.write("MQTT: TIMEOUT");
          break;
        case MQTT_CONNECTION_LOST:
          Serial.write("MQTT: CONN LOSS");
          break;
        case MQTT_CONNECT_FAILED:
          Serial.write("MQTT: CONN FAIL");
          break;
        case MQTT_DISCONNECTED:
          Serial.write("MQTT: DISCONNECTED");
          break;
        case MQTT_CONNECT_BAD_PROTOCOL:
          Serial.write("MQTT: BAD PROTO");
          break;
        case MQTT_CONNECT_BAD_CLIENT_ID:
          Serial.write("MQTT: BAD ID");
          break;
        case MQTT_CONNECT_UNAVAILABLE:
          Serial.write("MQTT: UNAVAILABLE");
          break;
        case MQTT_CONNECT_BAD_CREDENTIALS:
          Serial.write("MQTT: BAD CREDS");
          break;
        case MQTT_CONNECT_UNAUTHORIZED:
          Serial.write("MQTT: UNAUTH");
          break;
      }

      Serial.write("\n");
    } else if (command.equals("count+")) {
      numPulses++;
      publishCount();
    } else if (command.equals("count-")) {
      numPulses--;
      publishCount();
    } else if (command.equals("dm")) {
      nextDisplayMode();

      // Reset the backlight timeout.
      lastUserInteraction = timeClient.getEpochTime();
      lastDisplayModeChange = timeClient.getEpochTime();
      // Pause auto increment
      pauseAutoIncrement = 1;
    } else if (command.equals("relay1 on")) {
      relay1State = true;
      publishCount();
    } else if (command.equals("relay1 off")) {
      relay1State = false;
      publishCount();
    }
  }
}

void loop() {

  ArduinoOTA.handle();

  serviceSerialPort();

  // Handle queued interrupts for pulses
  if (rawPulseCounter > 0) {
    rawPulseCounter--;
    numPulses++;
    publishCount();
  }

  // Handle requests to change display mode
  if (rawModeCounter > 0) {
    rawModeCounter --;
    nextDisplayMode();

    // Reset the backlight timeout.
    lastUserInteraction = timeClient.getEpochTime();
    lastDisplayModeChange = timeClient.getEpochTime();
    // Pause auto increment
    pauseAutoIncrement = 1;
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
      String sl = String("Wifi: ");
      sl += getFormattedSignalStrength();
      sl += "% ";
      lcdmsg(sl);

      char ipStr[16];
      sprintf(ipStr, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
      lcd2(ipStr);
    } else if (WiFi.status() == WL_CONNECTION_LOST) {
      lcdmsg("WIFI: CONN LOST");
    } else if (WiFi.status() == WL_DISCONNECTED) {
      lcdmsg("WIFI: Disconnect");
    } else {
      lcdmsg("WIFI: ERROR");
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

  if (currentDisplayMode == DM_RELAY) {
    lcdmsg("Relay 1:");
    if (relay1State == true) {
      lcd2("ON ");
    } else {
      lcd2("OFF");
    }
  }

  // every x seconds, change display mode unless we're paused
  if ((timeClient.getEpochTime() - lastDisplayModeChange >= AUTO_INCREMENT_SECONDS)  && !pauseAutoIncrement) {
    lastDisplayModeChange = timeClient.getEpochTime();
    if (AUTO_INCREMENT_DM) {
      nextDisplayMode();
    }
  }
  // Release the pause if we surpass the override time
  if ((timeClient.getEpochTime() - lastDisplayModeChange >= AUTO_INCREMENT_PAUSE_SECONDS)  && pauseAutoIncrement) {
    pauseAutoIncrement = 0;
  }

  // Check if the backlight should be on or off
  if (timeClient.getEpochTime() - lastUserInteraction >= BACKLIGHT_TIMEOUT) {
    lcd.noBacklight();
  } else {
    lcd.backlight();
  }

  // Check if it's time for a healthcheck
  if (timeClient.getEpochTime() - lastHealthcheck >= HEALTHCHECK_TIMEOUT) {
    client.publish(statusTopic, "online");
    publishCount();
    lastHealthcheck = timeClient.getEpochTime();
  }

  // Check if the relay needs to be turned on or off
  if (relay1State == true) {
    digitalWrite(relay1Pin, HIGH);
  } else {
    digitalWrite(relay1Pin, LOW);
  }
}
