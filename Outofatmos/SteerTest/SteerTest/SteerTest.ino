#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <SimpleKalmanFilter.h>


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// start ada and dean kalmen
Adafruit_MPU6050 mpu;
SimpleKalmanFilter rollKalmanFilter(1.0, 1.0, 0.01);

Servo servo1;
Servo servo2;
Servo servo3;

void setup() {
  Serial.begin(115200);

  // Servos
  servo1.attach(11);
  servo2.attach(12);
  servo3.attach(24);
  servo1.write(0);
  servo2.write(0);
  servo3.write(180);

  // MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // 0x3C is default I2C address
    Serial.println("OLED not found");
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.display();
}

void loop() {
  servo1.write(180);
  delay(2000);
  servo1.write(0);
  delay(2000);
}


