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
#include <RH_RF95.h> //y tytytty
#include <Servo.h>

// --- Servo Setup ---
Servo servoLeft;
Servo servoRight;
#define SERVO_LEFT_PIN 11
#define SERVO_RIGHT_PIN 12
// ================================================================
// Shooting Target aka. lockheed martin please fund me so that we could invade russia with this one
float targetLat = 0;
float targetLon = 0;
#define BUTTON_PIN 10
bool buttonPressed = false;

// Target Init configurations

unsigned long targetSetTime = 0;
bool targetSelected = false;
bool steeringStarted = false;
const unsigned long STEER_DELAY = 45000; 

// ================================================================
// Servo bearing
float lastTargetBearing = 0;
int lastServoAngle = 90; // default straight

// --- PD Control Constants ---
float Kp = 0.6;
float Kd = 0.0;
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

// --- File Naming and Logging Interval ---
const unsigned long LOG_INTERVAL = 30000;     // Log every 5 seconds
const unsigned long FILE_INTERVAL = 10000;     // Log every 5 seconds
unsigned long lastLogTime = 0;
unsigned long fileStartTime = 0;
unsigned long logCounter = 0;
unsigned long fileNumber = 0; // For creating unique filenames

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
// blinking ting tong
bool yellowBlinkState = false;
unsigned long lastBlinkTime = 0;
const unsigned long BLINK_INTERVAL = 500;  // Blink every 500ms


// --- RFM95 ---
#define RFM95_CS 16
#define RFM95_RST 17
#define RFM95_INT 21
#define RFM95_FREQ 921.325



// --- Network Configuration ---
#define CLIENT_ADDRESS     1  // Address of this client (RP2040)
#define SERVER_ADDRESS     2  // Address of the server (ESP32)

RH_RF95 rf95(RFM95_CS, RFM95_INT);  // Create RFM95 instance

// --- for receive data from server ---
String inputString = "";
bool stringComplete = false;


void setup() {
  pixels.begin();
  pixels.setBrightness(50); // Set brightness
  pixels.fill(PXBLUE);      // Blue: Initializing
  pixels.show();

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // --- Initialize OLED ---
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    pixels.fill(PXRED); // Red for OLED failure
    pixels.show();
    while (1) delay(10);
  }
  display.display();
  delay(1000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // --- Initialize I2C ---
  Wire.begin();

  // --- Initialize QMC5883L ---
  qmc.init();

  // --- Initialize BMP280 ---
  if (!bmp.begin()) {
    pixels.fill(PXRED); // Red for BMP280 failure
    pixels.show();
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  // --- Initialize MPU6050 ---
  if (!mpu.begin(MPU6050_ADDRESS)) {
    pixels.fill(PXRED); // Red for MPU6050 failure
    pixels.show();
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // --- Initialize GPS (Serial1 on GPIO0/1) ---
  Serial1.setRX(1);
  Serial1.setTX(0);
  Serial1.begin(GPSBaud);

  // --- Initialize SD Card ---
  if (!SD.begin(SD_CS_PIN)) {
    pixels.fill(PXRED); // Red for SD Card failure
    pixels.show();
    while (1) delay(10);
  }

// --- RFM95 Initialization ---
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    oledDisplay("RFM95 Init Fail"); // Display error on OLED
    pixels.fill(PXRED); // Red for RFM95 failure
    pixels.show();
    while (1);
  }

  rf95.setFrequency(RFM95_FREQ); // 921.325 MHz
  rf95.setTxPower(23, false);    // Max power
  rf95.setSpreadingFactor(9);    // SF9 (can be 6–12)
  rf95.setSignalBandwidth(125000); // 125 kHz
  rf95.setCodingRate4(5);        // 4/5 coding rate


  // rf95.spiWrite(0x39, 0x34);

  // servoLeft.attach(SERVO_LEFT_PIN);
  // servoRight.attach(SERVO_RIGHT_PIN);
  // servoLeft.write(90);
  // servoRight.write(90);

  pixels.fill(PXGREEN); pixels.show();
}

void loop() {
  // checkButtonAndSetTarget();
  // // readSensors();
  // displaySensorData();

  // Blinking Yellow while waiting for second press to start steering
  // if (targetSelected && !steeringStarted) {
  //   if (millis() - lastBlinkTime >= BLINK_INTERVAL) {
  //     lastBlinkTime = millis();
  //     yellowBlinkState = !yellowBlinkState;
  //     pixels.fill(yellowBlinkState ? PXYELLOW : PXBLACK);
  //     pixels.show();
  //   }
  // }

  // // Start steering
  // if (steeringStarted) {
  //   steerToTarget();
  // }

  if (millis() - lastLogTime >= LOG_INTERVAL) {
    lastLogTime = millis();
    sendData(createDataString());

    // logData(); // Optional
  }
  pixels.fill(PXBLACK); pixels.show();

  smartDelay(200);
}




static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (Serial1.available()) gps.encode(Serial1.read());
  } while (millis() - start < ms);
}


