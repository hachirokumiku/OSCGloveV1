#include <WiFi.h>
#include <ArduinoOSC.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>

// === Wi-Fi and OSC Settings ===
const char* ssid = "YOUR SSID";
const char* password = "YOUR PASSWORD";
const char* hostIP = "YOUR IP";
const int oscPort = 8000;

// === OSC ===
ArduinoOSC osc;

// === Flex sensors ===
const int flexPins[5] = {32, 33, 25, 26, 27};
float flexValues[5];
int flexMin[5], flexMax[5];
const int smoothWindow = 5;
float flexBuffer[5][smoothWindow];
int bufferIndex = 0;

// === MPU6050 ===
Adafruit_MPU6050 mpu;
Madgwick filter;
unsigned long lastUpdate = 0;
float q[4];

// === Button & LED ===
const int ledPin = 2;        // Built-in LED
const int buttonPin = 15;    // Physical button or touch sensor

// === Setup ===
void setup() {
  Serial.begin(115200);

  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP); // assumes button pulls LOW when pressed

  // Wi-Fi connect
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi!");
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  // Start OSC
  osc.begin(WiFi.localIP());

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) { delay(10); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initial flex sensor calibration
  Serial.println("Calibrating flex sensors. Keep hand relaxed...");
  for (int i = 0; i < 5; i++) {
    flexMin[i] = 4095;
    flexMax[i] = 0;
    for (int j = 0; j < 100; j++) {
      int val = analogRead(flexPins[i]);
      if (val < flexMin[i]) flexMin[i] = val;
      if (val > flexMax[i]) flexMax[i] = val;
      delay(5);
    }
    Serial.print("Finger "); Serial.print(i); 
    Serial.print(" min/max: "); Serial.print(flexMin[i]); Serial.print("/"); Serial.println(flexMax[i]);
  }

  Serial.println("Setup complete.");
}

// === Normalize and smooth flex sensor reading 0–1 ===
float normalizeFlex(int finger, int raw) {
  float val = (float)(raw - flexMin[finger]) / (flexMax[finger] - flexMin[finger]);
  if (val < 0) val = 0;
  if (val > 1) val = 1;

  // Moving average
  flexBuffer[finger][bufferIndex] = val;
  float sum = 0;
  for (int i = 0; i < smoothWindow; i++) sum += flexBuffer[finger][i];
  return sum / smoothWindow;
}

// === Loop ===
void loop() {
  bufferIndex = (bufferIndex + 1) % smoothWindow;

  // Read flex sensors
  for (int i = 0; i < 5; i++) {
    int raw = analogRead(flexPins[i]);
    flexValues[i] = normalizeFlex(i, raw);
  }

  // Check for button press for recalibration
  if (digitalRead(buttonPin) == LOW) { // button pressed
    digitalWrite(ledPin, HIGH);
    Serial.println("Button pressed! Recalibrating flex sensors...");

    // Recalibrate flex min/max
    for (int i = 0; i < 5; i++) {
      flexMin[i] = 4095;
      flexMax[i] = 0;
      for (int j = 0; j < 100; j++) {
        int val = analogRead(flexPins[i]);
        if (val < flexMin[i]) flexMin[i] = val;
        if (val > flexMax[i]) flexMax[i] = val;
        delay(5);
      }
      Serial.print("Finger "); Serial.print(i); 
      Serial.print(" min/max: "); Serial.print(flexMin[i]); Serial.print("/"); Serial.println(flexMax[i]);
    }

    // Blink LED 3 times to signal completion
    for (int b = 0; b < 3; b++) {
      digitalWrite(ledPin, LOW);
      delay(150);
      digitalWrite(ledPin, HIGH);
      delay(150);
    }
    digitalWrite(ledPin, LOW);

    // Wait until button released to avoid repeated recalibration
    while (digitalRead(buttonPin) == LOW) { delay(10); }
  }

  // Read MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Update Madgwick filter
  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0f;
  lastUpdate = now;
  filter.updateIMU(g.gyro.x, g.gyro.y, g.gyro.z,
                   a.acceleration.x, a.acceleration.y, a.acceleration.z, dt);

  // Get quaternion
  q[0] = filter.q0;
  q[1] = filter.q1;
  q[2] = filter.q2;
  q[3] = filter.q3;

  // Send OSC bundle
  osc.send(hostIP, oscPort,
           "/glove",
           flexValues[0], flexValues[1], flexValues[2], flexValues[3], flexValues[4],
           q[0], q[1], q[2], q[3]);

  delay(20); // 50 Hz
}
