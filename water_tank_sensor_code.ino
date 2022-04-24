#include "secrets.h"
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include <U8x8lib.h>

// The MQTT topics that this device should publish/subscribe
#define AWS_IOT_PUBLISH_TOPIC   "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"
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

// the OLED used
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

WiFiClientSecure net = WiFiClientSecure();
MQTTClient client = MQTTClient(256);

void connectWifi()
{
  delay(500);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting to Wi-Fi");
  u8x8.drawString(0, 0, "Connecting wifi");

  uint32_t notConnectedCounter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
    notConnectedCounter++;
    if(notConnectedCounter > 100) { // Reset board if not connected after 10s
      Serial.println("Resetting due to Wifi not connecting...");
      ESP.restart();
    }
  }

  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);
}

void connectAWS()
{
  connectWifi();
  connectMQTT();
}

void connectMQTT()
{
  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.begin(AWS_IOT_ENDPOINT, 8883, net);
  client.setKeepAlive(120);

  // Create a message handler
  client.onMessage(messageHandler);

  Serial.println("Connecting to AWS IOT");
  u8x8.clearLine(0);
  u8x8.drawString(0, 0, "Connecting AWS");

  int i = 0;
  while (!client.connect(THINGNAME)) {
    Serial.print(".");
    u8x8.drawString(i, 1, ".");
    delay(100);
    i = i + 1;
    if (i = 3) {
      i = 0;
      u8x8.clearLine(1);
    }
  }

  if (!client.connected()) {
    Serial.println("AWS IoT Timeout!");
    return;
  }

  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);

  Serial.println("AWS IoT Connected!");
  u8x8.clearLine(0);
  u8x8.clearLine(1);
  u8x8.drawString(0, 0, "AWS connected");
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
  doc["sensor_depth_in"] = depth / MM_PER_INCH;
  doc["sensor_analog"] = analog;
  // doc["sensor_readings"] = readings;
  doc["sensor_current"] = dataCurrent;

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client
  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
  
  Serial.println(jsonBuffer);  // print to serial
  u8x8.clearLine(0);
  u8x8.drawString(0, 0, strcat(itoa(depth / MM_PER_INCH, buf, 10), "\" depth"));
}

void messageHandler(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);

  // deserialize json
  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);

  if (doc.containsKey("pub_frequency")) {
    pub_frequency = int(doc["pub_frequency"]);
  }
}

void setup() {
  Serial.begin(9600);
  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  connectAWS();
}

void loop() {
  client.loop();
  publishMessage();
  delay(pub_frequency);
}
