#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <QMC5883LCompass.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SimpleKalmanFilter.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_NeoPixel.h>
#include <Servo.h>

// --- Servo Setup ---
Servo servo1;
Servo servo2;
Servo servo3;
#define SERVO1_PIN 11
#define SERVO2_PIN 12
#define SERVO3_PIN 24

// ==============================
// DEPLOY CONFIGURATION
// ==============================
bool ENABLE_TILT_CHECK     = true;
bool ENABLE_ACCEL_DROP     = false;
bool ENABLE_ALTITUDE_DROP  = false;
bool ENABLE_TIME_CHECK     = true;

#define TILT_THRESHOLD_DEG 45.0
#define ACCEL_DROP_THRESHOLD 15
#define ALTITUDE_DROP_MARGIN 5.0
#define MIN_TIME_SINCE_DEPLOY 10000
#define LAUNCH_ACCEL_THRESHOLD 30.0
#define MAX_TIME_BEFORE_DEPLOY 3000

bool CHECK_MAX_TIME_BEFORE_DEPLOY = true;

// Runtime states
bool hasLaunched = false;
float maxAltitude = -9999.0;
unsigned long deployTime = 0;

// ================================================================
// Shooting Target (GPS Lock)
float targetLat = 0.0;
float targetLon = 0.0;
#define BUTTON_PIN 10
bool buttonPressed = false;
unsigned long targetSetTime = 0;
bool targetSelected = false;
bool steeringStarted = false;
const unsigned long STEER_DELAY = 45000;

String currentPhase = "Waiting";

// --- PD Control Constants ---
float Kp = 0.6;
float Kd = 0.0;
float lastError = 0.0;

// --- OLED Setup ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- QMC5883L (Compass) ---
QMC5883LCompass qmc;

// --- GPS ---
TinyGPSPlus gps;
static const uint32_t GPSBaud = 9600;

// --- BMP280 (Barometer) ---
Adafruit_BMP280 bmp;
const float SEALEVELPRESSURE_HPA = 1013.25;

// --- MPU6050 (IMU) ---
#define MPU6050_ADDRESS 0x68
Adafruit_MPU6050 mpu;
SimpleKalmanFilter rollKalmanFilter(1.0, 1.0, 0.01);
SimpleKalmanFilter pitchKalmanFilter(1.0, 1.0, 0.01);

// --- SD Card ---
#define SD_CS_PIN 25

// --- Logging ---
const unsigned long LOG_INTERVAL = 5000;
unsigned long lastLogTime = 0;
unsigned long fileStartTime = 0;
unsigned long logCounter = 0;
unsigned long fileNumber = 0;

// --- NeoPixel ---
#define NEOPIXEL_PIN 4
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
const uint32_t PXRED = pixels.Color(255, 0, 0);
const uint32_t PXGREEN = pixels.Color(0, 255, 0);
const uint32_t PXBLUE = pixels.Color(0, 0, 255);
const uint32_t PXBLACK = pixels.Color(0, 0, 0);
const uint32_t PXWHITE = pixels.Color(255, 255, 255);
const uint32_t PXYELLOW = pixels.Color(255, 255, 0);
bool yellowBlinkState = false;
unsigned long lastBlinkTime = 0;
const unsigned long BLINK_INTERVAL = 500;

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

#define GPS_RX_PIN 1
#define GPS_TX_PIN 0



void setup() {
  

  pixels.begin();
  pixels.setBrightness(50);
  pixels.fill(PXBLUE);
  pixels.show();

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    pixels.fill(PXRED);
    pixels.show();
    while (1) delay(10);
  }

  display.display();
  delay(1000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  Wire.begin();
  qmc.init();

  if (!bmp.begin()) {
    pixels.fill(PXRED);
    pixels.show();
    while (1) delay(10);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  if (!mpu.begin(MPU6050_ADDRESS)) {
    pixels.fill(PXRED);
    pixels.show();
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial1.setRX(GPS_RX_PIN);
  Serial1.setTX(GPS_TX_PIN);
  Serial1.begin(GPSBaud);

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo1.write(180);
  servo2.write(180);
  servo3.write(0);

  pixels.fill(PXGREEN);
  pixels.show();

  
}

void loop() {
  // bool passed = checkApogeeAndTilt();

  // if (passed) {
  //   servo3.write(60);
  //   delay(1000); // prevent re-trigger
  // }

  // // Determine color based on state
  // uint32_t blinkColor = hasLaunched ? PXYELLOW : PXWHITE;

  // // Blink LED if check has not passed
  // if (millis() - lastBlinkTime >= BLINK_INTERVAL && !passed) {
  //   lastBlinkTime = millis();
  //   yellowBlinkState = !yellowBlinkState;
  //   pixels.fill(yellowBlinkState ? blinkColor : PXBLACK);
  //   pixels.show();
  // }
  delay(12000);
  servo3.write(60);

}
