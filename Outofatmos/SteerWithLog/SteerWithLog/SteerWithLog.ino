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
#include <RH_RF95.h>
#include <Servo.h>

// --- Servo Setup ---
Servo servoLeft;
Servo servoRight;
#define SERVO_LEFT_PIN 11
#define SERVO_RIGHT_PIN 12

// --- Target GPS Coordinates ---
const float targetLat = 18.796168;  // Example target
const float targetLon = 98.979263;

// --- PD Control Constants ---
float Kp = 1.2;
float Kd = 0.6;
float lastError = 0;

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

// --- BMP280 ---
Adafruit_BMP280 bmp;
const float SEALEVELPRESSURE_HPA = 1013.25;

// --- MPU6050 ---
#define MPU6050_ADDRESS 0x68
Adafruit_MPU6050 mpu;
SimpleKalmanFilter kalmanPitch(1, 1, 0.01);
SimpleKalmanFilter kalmanRoll(1, 1, 0.01);

// --- SD Card ---
#define SD_CS_PIN 25

// --- NeoPixel ---
#define NEOPIXEL_PIN 4
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
const uint32_t PXRED = pixels.Color(255, 0, 0);
const uint32_t PXGREEN = pixels.Color(0, 255, 0);
const uint32_t PXBLUE = pixels.Color(0, 0, 255);
const uint32_t PXBLACK = pixels.Color(0, 0, 0);

// --- RFM95 ---
#define RFM95_CS 16
#define RFM95_RST 17
#define RFM95_INT 21
#define RFM95_FREQ 915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// --- Logging ---
const unsigned long LOG_INTERVAL = 5000;
unsigned long lastLogTime = 0;

void setup() {
  pixels.begin();
  pixels.setBrightness(50);
  pixels.fill(PXBLUE); pixels.show();
  Serial.begin(115200);

  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while (1);
  display.display(); delay(1000); display.clearDisplay();

  qmc.init();
  if (!bmp.begin()) while (1);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  if (!mpu.begin(MPU6050_ADDRESS)) while (1);
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial1.setRX(1); Serial1.setTX(0); Serial1.begin(GPSBaud);
  if (!SD.begin(SD_CS_PIN)) while (1);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW); delay(10);
  digitalWrite(RFM95_RST, HIGH); delay(10);
  if (!rf95.init()) while (1);
  rf95.setFrequency(RFM95_FREQ);
  rf95.setTxPower(23, false);

  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);
  servoLeft.write(90);
  servoRight.write(90);

  pixels.fill(PXGREEN); pixels.show();
}

void loop() {
  readSensors();
  displaySensorData();
  delay(50000);
  steerToTarget(); // <- PD Steering logic

  if (millis() - lastLogTime >= LOG_INTERVAL) {
    lastLogTime = millis();
    // logData(); // Optional
  }

  smartDelay(200);
}



void readSensors() {
  qmc.read();
  while (Serial1.available()) gps.encode(Serial1.read());
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
}

static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (Serial1.available()) gps.encode(Serial1.read());
  } while (millis() - start < ms);
}
