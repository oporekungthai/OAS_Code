#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SimpleKalmanFilter.h>
#include <math.h>

// Deployment Apogee Configurations Test ==========


#define TILT_THRESHOLD_DEG 45
#define ACCEL_DROP_THRESHOLD 3.0     // Adjust based on test data
#define ALTITUDE_DROP_MARGIN 5.0     // Meters
#define MIN_TIME_SINCE_DEPLOY 3000   // Milliseconds

// Flags to enable/disable each condition
bool ENABLE_TILT_CHECK     = true;
bool ENABLE_ACCEL_DROP     = true;
bool ENABLE_ALTITUDE_DROP  = true;
bool ENABLE_TIME_CHECK     = true;

// Runtime state
float maxAltitude = 0;
unsigned long deployTime = 0;

// --- OLED Setup ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- MPU6050 Setup ---
Adafruit_MPU6050 mpu;
#define MPU6050_ADDRESS 0x68

// --- Kalman Filter for Pitch ---
SimpleKalmanFilter kalmanPitch(1, 1, 0.01);

// --- State Variables ---
bool isDeployed = false;
bool isTiltedDown = false;
float accelMag = 0;
float filteredPitch = 0;

// --- Setup ---
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // OLED Init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while (1);  // OLED failed
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // MPU Init
  if (!mpu.begin(MPU6050_ADDRESS)) {
    while (1);  // MPU failed
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(1000); // Allow sensor to settle
}

// --- Loop ---
void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate acceleration magnitude
  accelMag = sqrt(a.acceleration.x * a.acceleration.x +
                  a.acceleration.y * a.acceleration.y +
                  a.acceleration.z * a.acceleration.z);
  isDeployed = accelMag > 30.0;

  // Calculate filtered pitch
  float rawPitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  filteredPitch = kalmanPitch.updateEstimate(rawPitch);
  isTiltedDown = abs(filteredPitch) > 45;

  // Display to OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Deployed: ");
  display.println(isDeployed ? "YES" : "NO");

  display.print("Tilt Down: ");
  display.println(isTiltedDown ? "YES" : "NO");

  display.print("Accel: ");
  display.println(accelMag, 2);

  display.print("Pitch: ");
  display.println(filteredPitch, 1);

  display.display();
  delay(200);
}