#include "secrets.h"
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include <WiFiNINA.h>
#include <SPI.h>
#include <PubSubClient.h>


// The MQTT topics that this device should publish/subscribe
#define PUBLISH_TOPIC   "rtlee/feeds/test"
#define TEST_TOPIC   "rtlee/feeds/test"
#define SUBSCRIBE_TOPIC "rtlee/feeds/sub"
#define ANALOG_PIN 33
#define VREF 3300 // ADC's reference voltage on your Arduino,typical value:5000mV
#define CURRENT_INIT 4.00 // Current @ 0mm (uint: mA)
#define CURRENT_MAX 20.00 // Current @ 0mm (uint: mA)
#define DENSITY_WATER 1  // Pure water density normalized to 1
#define RANGE 5000 // Depth measuring range 5000mm (for water)
#define RESOLUTION 4095.0 // analog resolution
#define RESISTANCE 220.0 // ohms of the resistor
#define MM_PER_INCH 25.4 // millimeters per inch
#define CURRENT_OFFSET 0.6 // calibration constant in amps
#define NUM_READINGS 50

uint32_t pub_frequency = 60000; // ms frequency of publishing sensor readings
int status = WL_IDLE_STATUS;     // the WiFi radio's status
WiFiClient wifiClient;
long lastReconnectAttempt = 0;
PubSubClient client("io.adafruit.com", 1883, wifiClient);

boolean reconnect() {
  Serial.println("Attempting to connect to MQTT client");
  if (client.connect("arduinoClient", ADAFRUIT_USERNAME, ADAFRUIT_PASSWORD)) {
    Serial.println("Connected to MQTT client");
    // Once connected, publish an announcement...
    // client.publish(TEST_TOPIC,"hello world");
    // ... and resubscribe
    // client.subscribe(SUBSCRIBE_TOPIC);
  }
  return client.connected();
}


void publishMessage()
{
  StaticJsonDocument<200> doc;
  char buf[8];
  int16_t dataVoltage, analog;
  float dataCurrent, depth; //unit:mA
  doc["time"] = millis();

  // read depth
  int readings[NUM_READINGS];      // the readings from the analog input
  int total = 0;
  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < NUM_READINGS; thisReading++) {
    readings[thisReading] = analogRead(ANALOG_PIN);
    total = total + readings[thisReading];
  }
  analog = total / NUM_READINGS;
  
  dataVoltage = analog / RESOLUTION * VREF;
  dataCurrent = dataVoltage / RESISTANCE + CURRENT_OFFSET;
  depth = (dataCurrent - CURRENT_INIT) / (CURRENT_MAX - CURRENT_INIT) * (RANGE / DENSITY_WATER); //Calculate depth from current readings
  if (depth < 0)
  {
    depth = 0.0;
  }
  doc["sensor_voltage"] = dataVoltage;
  // doc["sensor_depth_mm"] = depth;
  doc["value"] = depth / MM_PER_INCH;
  doc["sensor_analog"] = analog;
  // doc["sensor_readings"] = readings;
  doc["sensor_current"] = dataCurrent;

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client
  client.publish(PUBLISH_TOPIC, jsonBuffer);
  
  Serial.println(jsonBuffer);  // print to serial
}

void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(WIFI_SSID);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // wait 5 seconds for connection:
    delay(5000);
  }


  Serial.println("You're connected to the network");
  lastReconnectAttempt = 0;
}

void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
}

void loop() {
    if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // Client connected
    client.loop();
    publishMessage();
    delay(5000);
  }
}
