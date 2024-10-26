#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <WiFi.h>
#include <PubSubClient.h>  // MQTT Library

// OLED display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

Adafruit_MPU6050 mpu;

// Variables for step counting
int stepCounter = 0;
float previousGyroMagnitude = 0;
float walkThreshold = 30.0;  // Lower threshold for walking
float runThreshold = 80.0;   // Higher threshold for running
unsigned long lastStepTime = 0;
unsigned long stepDelayWalk = 300;
unsigned long stepDelayRun = 150; // Faster step delay for running

// Variables for fall detection
float fallThreshold = 500.0;
bool fallDetected = false;
unsigned long fallDetectedTime = 0;
unsigned long fallResetDelay = 2000;  // Keep fall detection for 2 seconds

// WiFi and MQTT credentials
const char* ssid = "vivo 1904";
const char* password = "Dilanka@1998";
const char* mqtt_server = "broker.emqx.io";

// WiFi and MQTT objects
WiFiClient espClient;
PubSubClient client(espClient);

void connectToWiFi() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Connecting to WiFi...");
  display.display();

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("WiFi Connected!");
  display.setCursor(0, 16);
  display.print("IP: ");
  display.println(WiFi.localIP());  // Display IP address on OLED
  display.display();
  delay(2000);
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32StepCounter")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}

void setupMQTT() {
  client.setServer(mqtt_server, 1883);
}

void setup(void) {
  Serial.begin(115200);

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }
  display.display();
  delay(1000);
  display.clearDisplay();

  Serial.println("MPU6050 test");

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (true) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Set accelerometer and gyroscope ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Connect to WiFi and setup MQTT
  connectToWiFi();
  setupMQTT();

  // Display ready status
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("System Ready!");
  display.display();
  delay(1000);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Get sensor readings
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Capture gyroscope readings
  float gx = gyro.gyro.x * 57.2958;  // Convert rad/s to degrees/s
  float gy = gyro.gyro.y * 57.2958;
  float gz = gyro.gyro.z * 57.2958;

  // Calculate the magnitude of the gyroscope vector
  float gyroMagnitude = sqrt(gx * gx + gy * gy + gz * gz);

  // Detect walking or running based on thresholds
  unsigned long currentTime = millis();
  if (gyroMagnitude - previousGyroMagnitude > walkThreshold && (currentTime - lastStepTime > stepDelayWalk)) {
    stepCounter++;
    lastStepTime = currentTime;
    Serial.print("Walking step detected! Total steps: ");
    Serial.println(stepCounter);
  } else if (gyroMagnitude - previousGyroMagnitude > runThreshold && (currentTime - lastStepTime > stepDelayRun)) {
    stepCounter++;
    lastStepTime = currentTime;
    Serial.print("Running step detected! Total steps: ");
    Serial.println(stepCounter);
  }

  // Publish step count to MQTT topic
  char stepCountStr[10];
  itoa(stepCounter, stepCountStr, 10);
  client.publish("esp32/stepCount", stepCountStr);

  // Fall detection
  if (gyroMagnitude > fallThreshold && !fallDetected) {
    fallDetected = true;
    fallDetectedTime = millis();
    Serial.println("Fall detected!");

    // Publish fall detection to MQTT
    client.publish("esp32/fallDetected", "Fall Detected");
    delay(500);
    client.publish("esp32/fallDetected", " ");

    // Display fall detection on OLED
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("FALL!");
    display.setTextSize(1);
    display.setCursor(0, 24);
    display.println("Fall Detected!");
    display.display();
  }

  if (fallDetected && (millis() - fallDetectedTime > fallResetDelay)) {
    fallDetected = false;
  }

  // Display step count on OLED (when no fall is detected)
  if (!fallDetected) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("Steps: ");
    display.println(stepCounter);
    display.display();
  }

  // Update previous gyro magnitude
  previousGyroMagnitude = gyroMagnitude;

  delay(100);
}
