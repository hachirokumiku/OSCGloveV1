/*  ESP32 - 3x MPU6050 (wrist, index, ring) + Madgwick + OSC gesture detection
    Wiring:
      - Wire (default) SDA=21, SCL=22 -> wrist (AD0=GND -> 0x68) and index (AD0=VCC -> 0x69)
      - Wire1 SDA=17, SCL=16 -> ring (AD0=GND -> 0x68)
    Libraries (Library Manager):
      - ArduinoOSC (include ArduinoOSCWiFi.h)
      - Adafruit MPU6050
      - Adafruit Unified Sensor
      - MadgwickAHRS
*/

#include <WiFi.h>
#include <ArduinoOSCWiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>

// ---------- WiFi / OSC ----------
const char* WIFI_SSID = "YOURWIFI";
const char* WIFI_PASS = "YOURPASS";

const char* OSC_HOST = "YOUR LOCAL IP";
const int   OSC_SEND_PORT = 8000;
const int   OSC_RECV_PORT = 9000; // unused but kept for tests

// ---------- I2C / IMU addresses ----------
const uint8_t ADDR_WRIST  = 0x68; // Wire, AD0=GND
const uint8_t ADDR_INDEX  = 0x69; // Wire, AD0=VCC
const uint8_t ADDR_RING   = 0x68; // Wire1, AD0=GND

// ---------- Objects ----------
Adafruit_MPU6050 mpuWrist;
Adafruit_MPU6050 mpuIndex;
Adafruit_MPU6050 mpuRing;

Madgwick filterWrist;
Madgwick filterIndex;
Madgwick filterRing;

// ---------- gyro bias storage ----------
struct Bias { float x=0, y=0, z=0; };
Bias biasW, biasI, biasR;

// neutral pitch offsets (deg) captured during calibration
float neutralPitchW = 0.0f;
float neutralPitchI = 0.0f;
float neutralPitchR = 0.0f;

// axis sign corrections (flip if your mounting is rotated)
const int SIGN_INDEX =  1; // set -1 if index pitch sign is inverted
const int SIGN_RING  =  1; // set -1 if ring pitch sign is inverted

// gesture thresholds (degrees)
const float THRESH_INDEX_EXTENDED = 15.0f; // degrees = considered "straight"
const float THRESH_FLEX_LOW = 30.0f;       // finger considered bent
const float THRESH_FLEX_GRIP = 35.0f;      // both bent -> grip

// hysteresis margins (degrees)
const float HYST = 6.0f;

enum Gesture { NONE=0, POINT=1, GRIP=2 };
Gesture currentGesture = NONE;

// timing
unsigned long lastMicros = 0;

// ---------------- utilities ----------------
static void quatToEuler(float q0, float q1, float q2, float q3,
                        float &roll, float &pitch, float &yaw)
{
  // ZYX (yaw-pitch-roll)
  float sinr_cosp = 2.0f * (q0*q1 + q2*q3);
  float cosr_cosp = 1.0f - 2.0f * (q1*q1 + q2*q2);
  roll = atan2(sinr_cosp, cosr_cosp);

  float sinp = 2.0f * (q0*q2 - q3*q1);
  if (fabs(sinp) >= 1.0f) pitch = copysign(M_PI/2.0f, sinp);
  else pitch = asin(sinp);

  float siny_cosp = 2.0f * (q0*q3 + q1*q2);
  float cosy_cosp = 1.0f - 2.0f * (q2*q2 + q3*q3);
  yaw = atan2(siny_cosp, cosy_cosp);
}

void calcGyroBias(Adafruit_MPU6050 &mpu, Bias &b, int samples=300) {
  sensors_event_t a, g, t;
  b = {};
  for (int i=0; i<samples; ++i) {
    mpu.getEvent(&a, &g, &t);
    b.x += g.gyro.x;
    b.y += g.gyro.y;
    b.z += g.gyro.z;
    delay(3);
  }
  b.x /= samples; b.y /= samples; b.z /= samples;
}

float rad2deg(float r) { return r * 180.0f / PI; }

// ----------------- setup -------------------
void setup() {
  Serial.begin(115200);
  delay(10);

  // I2C buses
  Wire.begin(21, 22, 400000);     // default Wire: SDA=21, SCL=22
  Wire1.begin(17, 16, 400000);    // Wire1: SDA=17, SCL=16

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi connected, IP: ");
  Serial.println(WiFi.localIP());

  // OSC receiver test (optional)
  OscWiFi.subscribe(OSC_RECV_PORT, "/ping", [](const OscMessage&) {
    Serial.println("OSC ping received");
  });

  // initialize IMUs
  if (!mpuWrist.begin(ADDR_WRIST))  { Serial.println("ERR: wrist MPU not found"); while(1) delay(10); }
  if (!mpuIndex.begin(ADDR_INDEX))  { Serial.println("ERR: index MPU not found"); while(1) delay(10); }
  if (!mpuRing.begin(ADDR_RING, &Wire1)) { Serial.println("ERR: ring MPU not found on Wire1"); while(1) delay(10); }

  auto initMPU=[&](Adafruit_MPU6050 &m){
    m.setAccelerometerRange(MPU6050_RANGE_8_G);
    m.setGyroRange(MPU6050_RANGE_500_DEG);
    m.setFilterBandwidth(MPU6050_BAND_21_HZ);
  };
  initMPU(mpuWrist); initMPU(mpuIndex); initMPU(mpuRing);

  delay(200);

  // calibration step: keep hand still, fingers straight
  Serial.println("CALIBRATION: keep the hand still and fingers straight for a few seconds...");
  delay(1500);
  calcGyroBias(mpuWrist, biasW);
  calcGyroBias(mpuIndex, biasI);
  calcGyroBias(mpuRing, biasR);
  Serial.println("Gyro bias done.");

  // Initialize filters
  filterWrist.begin(100.0f);
  filterIndex.begin(100.0f);
  filterRing.begin(100.0f);

  // capture neutral pitch offsets (average a bit)
  const int N = 50;
  float accPitchW = 0, accPitchI = 0, accPitchR = 0;
  for (int i=0;i<N;i++){
    sensors_event_t a, g, t;
    mpuWrist.getEvent(&a, &g, &t);
    float p = atan2(a.acceleration.y, a.acceleration.z);
    accPitchW += p;

    mpuIndex.getEvent(&a, &g, &t);
    p = atan2(a.acceleration.y, a.acceleration.z);
    accPitchI += p;

    mpuRing.getEvent(&a, &g, &t);
    p = atan2(a.acceleration.y, a.acceleration.z);
    accPitchR += p;
    delay(10);
  }
  neutralPitchW = rad2deg(accPitchW / N);
  neutralPitchI = rad2deg(accPitchI / N);
  neutralPitchR = rad2deg(accPitchR / N);

  Serial.printf("Neutral pitch (deg) - wrist: %.2f  index: %.2f  ring: %.2f\n",
                neutralPitchW, neutralPitchI, neutralPitchR);

  lastMicros = micros();
}

// ----------------- main loop ----------------
void loop() {
  // dt
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1e6f;
  if (dt <= 0) dt = 1.0f/200.0f;
  lastMicros = now;

  sensors_event_t aW, gW, tW;
  sensors_event_t aI, gI, tI;
  sensors_event_t aR, gR, tR;

  mpuWrist.getEvent(&aW, &gW, &tW);
  mpuIndex.getEvent(&aI, &gI, &tI);
  mpuRing.getEvent(&aR, &gR, &tR);

  // remove gyro bias (deg/s)
  float gxW = gW.gyro.x - biasW.x;
  float gyW = gW.gyro.y - biasW.y;
  float gzW = gW.gyro.z - biasW.z;

  float gxI = gI.gyro.x - biasI.x;
  float gyI = gI.gyro.y - biasI.y;
  float gzI = gI.gyro.z - biasI.z;

  float gxR = gR.gyro.x - biasR.x;
  float gyR = gR.gyro.y - biasR.y;
  float gzR = gR.gyro.z - biasR.z;

  // Madgwick update (note: Madgwick expects rad/s for gyro)
  filterWrist.updateIMU(gxW * DEG_TO_RAD, gyW * DEG_TO_RAD, gzW * DEG_TO_RAD,
                        aW.acceleration.x, aW.acceleration.y, aW.acceleration.z, dt);

  filterIndex.updateIMU(gxI * DEG_TO_RAD, gyI * DEG_TO_RAD, gzI * DEG_TO_RAD,
                        aI.acceleration.x, aI.acceleration.y, aI.acceleration.z, dt);

  filterRing.updateIMU(gxR * DEG_TO_RAD, gyR * DEG_TO_RAD, gzR * DEG_TO_RAD,
                       aR.acceleration.x, aR.acceleration.y, aR.acceleration.z, dt);

  // euler angles (rad)
  float rW,pW,yW, rI,pI,yI, rR,pR,yR;
  quatToEuler(filterWrist.q0, filterWrist.q1, filterWrist.q2, filterWrist.q3, rW, pW, yW);
  quatToEuler(filterIndex.q0, filterIndex.q1, filterIndex.q2, filterIndex.q3, rI, pI, yI);
  quatToEuler(filterRing.q0, filterRing.q1, filterRing.q2, filterRing.q3, rR, pR, yR);

  // convert pitch to degrees and remove neutral offsets; apply sign flips
  float pitchWdeg = rad2deg(pW) - neutralPitchW;
  float pitchIdeg = SIGN_INDEX * (rad2deg(pI) - neutralPitchI);
  float pitchRdeg = SIGN_RING  * (rad2deg(pR) - neutralPitchR);

  // flex = finger pitch - hand pitch (positive = bent if mounting matches)
  float flexIndex = pitchIdeg - pitchWdeg;
  float flexRing  = pitchRdeg - pitchWdeg;

  // clamp reasonable ranges
  if (flexIndex > 180) flexIndex -= 360;
  if (flexRing  > 180) flexRing  -= 360;

  // --- Gesture detection with hysteresis ---
  Gesture next = NONE;

  // Grip: both bent past GRIP threshold
  static bool latGrip = false;
  if (!latGrip) {
    if (flexIndex > THRESH_FLEX_GRIP && flexRing > THRESH_FLEX_GRIP) { latGrip = true; next = GRIP; }
  } else {
    if (!(flexIndex > THRESH_FLEX_GRIP - HYST && flexRing > THRESH_FLEX_GRIP - HYST)) latGrip = false;
    else next = GRIP;
  }

  // Point: index extended, ring bent
  static bool latPoint = false;
  if (!latPoint && next == NONE) {
    if (flexIndex < THRESH_INDEX_EXTENDED && flexRing > THRESH_FLEX_LOW) { latPoint = true; next = POINT; }
  } else if (latPoint) {
    // stay in point until index starts to bend or ring unbends
    if (!(flexIndex < THRESH_INDEX_EXTENDED + HYST && flexRing > THRESH_FLEX_LOW - HYST)) latPoint = false;
    else next = POINT;
  }

  // update currentGesture if changed
  if (next != currentGesture) {
    currentGesture = next;
    // send an OSC announcement
    if (currentGesture == POINT) {
      OscWiFi.send(OSC_HOST, OSC_SEND_PORT, "/gesture", (int)POINT, "POINT");
      Serial.println("Gesture: POINT");
    } else if (currentGesture == GRIP) {
      OscWiFi.send(OSC_HOST, OSC_SEND_PORT, "/gesture", (int)GRIP, "GRIP");
      Serial.println("Gesture: GRIP");
    } else {
      OscWiFi.send(OSC_HOST, OSC_SEND_PORT, "/gesture", (int)NONE, "NONE");
      Serial.println("Gesture: NONE");
    }
  }

  // --- Send orientation / flex values continually ---
  // Hand orient (deg)
  OscWiFi.send(OSC_HOST, OSC_SEND_PORT, "/hand/orient", rad2deg(rW), pitchWdeg, rad2deg(yW));
  // Index / ring orient
  OscWiFi.send(OSC_HOST, OSC_SEND_PORT, "/index/orient", rad2deg(rI), pitchIdeg, rad2deg(yI));
  OscWiFi.send(OSC_HOST, OSC_SEND_PORT, "/ring/orient",  rad2deg(rR), pitchRdeg, rad2deg(yR));
  // flex values (deg)
  OscWiFi.send(OSC_HOST, OSC_SEND_PORT, "/flex/index", flexIndex);
  OscWiFi.send(OSC_HOST, OSC_SEND_PORT, "/flex/ring",  flexRing);

  OscWiFi.update();

  // Serial debug occasionally
  static unsigned long lastDbg = 0;
  if (millis() - lastDbg > 250) {
    lastDbg = millis();
    Serial.printf("Flex I: %.1f  R: %.1f  Gesture: %d\n", flexIndex, flexRing, (int)currentGesture);
  }

  delay(8); // ~100-120Hz loop
}
