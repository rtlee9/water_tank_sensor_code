#include "secrets.h"
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include <WiFiNINA.h>
#include <SPI.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>


// The MQTT topics that this device should publish/subscribe
#define PUBLISH_TOPIC   "rtlee/feeds/water-sensor-feed"
#define TEST_TOPIC   "rtlee/feeds/test"
#define SUBSCRIBE_TOPIC "rtlee/feeds/sub"
#define ANALOG_PIN A7
#define VREF 3300 // ADC's reference voltage on your Arduino,typical value:5000mV
#define CURRENT_INIT 4.00 // Current @ 0mm (uint: mA)
#define CURRENT_MAX 20.00 // Current @ 0mm (uint: mA)
#define DENSITY_WATER 1  // Pure water density normalized to 1
#define RANGE 5000 // Depth measuring range 5000mm (for water)
#define RESISTANCE 220.0 // ohms of the resistor
#define MM_PER_INCH 25.4 // millimeters per inch
#define CURRENT_OFFSET 0 // calibration constant in amps
#define NUM_READINGS 8
#define PUB_FREQUENCY 5000  // ms frequency of publishing sensor readings
#define RESOLUTION_BITS 10 // analog resolution

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

  dataVoltage = analog / pow(2, RESOLUTION_BITS) * VREF;
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
  serializeJson(doc, jsonBuffer);
  // publish to test topic if on serial (native USB) else publish to prod
  if (Serial) {
    client.publish(TEST_TOPIC, jsonBuffer);
  }
  else {
    client.publish(PUBLISH_TOPIC, jsonBuffer);
  }

  Serial.println(jsonBuffer);  // print to serial
}

void connectWifi() {
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
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(WIFI_SSID);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // wait 5 seconds for connection:
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
    delay(5000);
  }
  Serial.println("You're connected to the network");
  statusLight(5, 100);
}


void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(9600);
  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  statusLight(10, 100);

  connectWifi();
  lastReconnectAttempt = 0;  // set MQTT last attempt
  analogReadResolution(RESOLUTION_BITS);

  // OTA
  ArduinoOTA.begin(WiFi.localIP(), "nano_water_monitor", OTA_PASSWORD, InternalStorage);

}

void statusLight(int num_readings, int sleep) {
  for (int n = 0; n < num_readings; n++) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(sleep);                       // wait
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(sleep);                       // wait
  }
}

void loop() {
  // ensure connected to wifi
  if (status != WL_CONNECTED) {
    connectWifi();
  }

  // ensure connected to MQTT
  if (!client.connected()) {
    digitalWrite(LED_BUILTIN, LOW);
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
    ArduinoOTA.poll();
    digitalWrite(LED_BUILTIN, HIGH);
    client.loop();
    publishMessage();
    delay(PUB_FREQUENCY); // TODO: don't sleep just check elapsed time if timeout
  }
}
