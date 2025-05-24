#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include < .h>
#include <TinyGPS++.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SimpleKalmanFilter.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_NeoPixel.h>
#include <RH_RF95.h> // For RFM95
#include <Servo.h>

// --- OLED Setup ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET     -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- QMC5883L (Compass) ---
QMC5883LCompass qmc;

// --- GPS (Neo-M10Q) -  Serial1 on GPIO0/1 - NO SERIAL MONITOR! ---
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;

// --- BMP280 (Barometer) ---
Adafruit_BMP280 bmp;
const float SEALEVELPRESSURE_HPA = 1013.25;

// --- MPU6050 (Gyro/Accelerometer) ---
#define MPU6050_ADDRESS 0x68 // or 0x69
Adafruit_MPU6050 mpu;
SimpleKalmanFilter kalmanPitch(1, 1, 0.01);
SimpleKalmanFilter kalmanRoll(1, 1, 0.01);

// --- SD Card ---
#define SD_CS_PIN 25   //  Change this to your SD card's CS pin!

// --- File Naming and Logging Interval ---
const unsigned long LOG_INTERVAL = 5000;     // Log every 5 seconds
const unsigned long FILE_INTERVAL = 10000;     // Log every 5 seconds
unsigned long lastLogTime = 0;
unsigned long fileStartTime = 0;
unsigned long logCounter = 0;
unsigned long fileNumber = 0; // For creating unique filenames

// --- NeoPixel ---
#define NEOPIXEL_PIN 4
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
const uint32_t PXRED    = pixels.Color(255, 0, 0);
const uint32_t PXGREEN  = pixels.Color(0, 255, 0);
const uint32_t PXBLUE   = pixels.Color(0, 0, 255);
const uint32_t PXWHITE  = pixels.Color(255, 255, 255);
const uint32_t PXYELLOW = pixels.Color(255, 255, 0);
const uint32_t PXBLACK  = pixels.Color(0, 0, 0);
uint32_t PIXEL_COLOR = PXBLACK;

// --- RFM95 Configuration (Client - RP2040) ---
// Adafruit Feather RP2040 RFM95 Pinout
#define RFM95_CS      16  // GPIO16
#define RFM95_RST     17  // GPIO17
#define RFM95_INT     21  // GPIO21 (DIO0)
#define RFM95_FREQ 915.0

// --- Network Configuration ---
#define CLIENT_ADDRESS     1  // Address of this client (RP2040)
#define SERVER_ADDRESS     2  // Address of the server (ESP32)

RH_RF95 rf95(RFM95_CS, RFM95_INT);  // Create RFM95 instance

// --- for receive data from server ---
String inputString = "";
bool stringComplete = false;

void setup() {
  // --- Initialize NeoPixel, start BLUE ---
  pixels.begin();
  pixels.setBrightness(50); // Set brightness
  pixels.fill(PXBLUE);      // Blue: Initializing
  pixels.show();
  Serial.begin(115200);

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

  rf95.setFrequency(RFM95_FREQ); // Set frequency
  rf95.setTxPower(23, false);     // Set transmit power (+23 dBm, high power)
  rf95.spiWrite(0x1D, 0x78);      // BW 125 kHz, CR 4/5 (Explicit Header)
  rf95.spiWrite(0x1E, 0x94);      // SF9, CRC OFF if on then 94
  rf95.spiWrite(0x39, 0x34); // Set Sync Word High Byte to 0x34
  rf95.spiWrite(0x3A, 0x44); // Set Sync Word Low Byte to 0x44

  // --- All initializations done: Green light ---
  pixels.fill(PXGREEN);  // Green = ready
  pixels.show();
}

void loop() {
  // Read all sensors
  readSensors();

  // Display data on OLED
  displaySensorData();

  // Log data to SD card
  if (millis() - lastLogTime >= LOG_INTERVAL) {
    logData();
    lastLogTime = millis();
    sendData(createDataString()); //=================================

  }

    // Check for incoming RFM95 messages (commands)
    if (stringComplete) {
       // In single-core mode, we do nothing with commands
       // other than clearing the flag.  We removed step changes.
        inputString = ""; //clear
        stringComplete = false; // Reset
    }
    // Send Data Regularly
    

  // Feed the GPS (using smartDelay)
  smartDelay(200); // Update every 200ms, gives time for other tasks
}

void readSensors() {
    // --- Read QMC5883L Data ---
    qmc.read();

    // --- Read GPS Data ---
    while (Serial1.available() > 0) {
        gps.encode(Serial1.read());
    }
    // --- Read MPU6050 Data ---
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // --- Read BMP280 Data ---
    // The BMP280 library functions are blocking, which isn't ideal,
    // but in single-core mode, there's no way around it.

    // Check for valid BMP280 data
    if (isnan(bmp.readAltitude(SEALEVELPRESSURE_HPA)) || isinf(bmp.readAltitude(SEALEVELPRESSURE_HPA))) {
      return; // Don't use invalid BMP readings
    }
}

// Helper function to format data as a string
String createDataString() {
    // Read all the sensor values *before* creating the string.
    int azimuth = qmc.getAzimuth();
    float latitude = gps.location.isValid() ? gps.location.lat() : 0.0;
    float longitude = gps.location.isValid() ? gps.location.lng() : 0.0;
    float altitude = gps.location.isValid() ? gps.altitude.meters() : 0.0;
    bool gpsFix = gps.location.isValid();
     sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float roll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
    float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
    float filteredPitch = kalmanPitch.updateEstimate(pitch);
    float filteredRoll = kalmanRoll.updateEstimate(roll);
    float bmpAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

    // Format the data string
    String dataString = "";
     if (gpsFix) {  //Only send GPS data if we have a fix
       dataString += String(latitude, 6) + ",";
       dataString += String(longitude, 6) + ",";
       dataString += String(altitude) + ",";
    } else {
        dataString += "N/A,N/A,N/A,";  // Indicate no GPS fix
    }
    dataString += String(azimuth) + ",";
    dataString += String(filteredPitch) + ",";
    dataString += String(filteredRoll) + ",";
    dataString += String(bmpAltitude);

    return dataString;
}

void displaySensorData() {

    // Read all sensor data *before* displaying.  This is important
    // to minimize delays *within* the OLED update process.
    int azimuth = qmc.getAzimuth();

    float latitude = 0.0;
    float longitude = 0.0;
    float altitude = 0.0;
    bool gpsFix = gps.location.isValid(); // Get fix status

    if (gpsFix) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        altitude = gps.altitude.meters();
          // Set NeoPixel to GREEN if we have a fix
         pixels.fill(PXGREEN);
         pixels.show();
    }
     else
    {
        pixels.fill(PXBLACK); // Set NeoPixel to off if no fix
        pixels.show();
    }

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float roll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
    float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
    float filteredPitch = kalmanPitch.updateEstimate(pitch);
    float filteredRoll = kalmanRoll.updateEstimate(roll);

    float bmpAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

    // --- Display Data on OLED ---
    display.clearDisplay();
    display.setCursor(0, 0);

    // Display QMC5883L data
    display.print(F("Az: ")); display.print(azimuth);

     if (gpsFix) {
        display.print(F(" (")); display.print(latitude, 2);  display.print(F(","));
        display.print(longitude, 2); display.println(F(")"));
    } else {
        display.println(F(" NoFix"));
    }
    // MPU
    display.print(F("Pit: ")); display.print(filteredPitch);
    Serial.print("Pit: "); Serial.print(filteredPitch);
    display.print(F(" Rol: ")); display.println(filteredRoll);
    Serial.print(" Roll: "); Serial.print(filteredRoll);
    Serial.println("");
    // BMP
    display.print(F("Alt: ")); display.println(bmpAltitude);
    

    display.display(); // Update the display

}

void logData() {
    char filename[15];

    // Check if it's time to create a new file
    if (millis() - fileStartTime >= FILE_INTERVAL * 1000) {
        fileNumber++; // Increment file number
         if(fileNumber > 9999){
            fileNumber = 0; // Reset
        }
        fileStartTime = millis();  // Reset file start time
        logCounter = 0; // Reset log entry counter within the file
    }

    snprintf(filename, sizeof(filename), "data%04lu.txt", fileNumber);

    File dataFile = SD.open(filename, FILE_WRITE); // Open in append mode

    if (dataFile) {
        // Write the data string to the file
        dataFile.println(createDataString());
        dataFile.close(); // *** IMPORTANT: Close the file ***
        logCounter++;


    } else {
        // OLED error display
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(F("File Error!"));
        display.display();
        delay(2000);
    }
}
// void sendData(String data) {
//   // Convert String to char array.
//   char messageToSend[data.length() + 1];
//   data.toCharArray(messageToSend, sizeof(messageToSend));

//   // Send message
//   rf95.send((uint8_t *)messageToSend, sizeof(messageToSend));
//   rf95.waitPacketSent(); // Wait for transmission to complete
//   Serial.print("Sent");
//   display.clearDisplay();
//   display.setCursor(0, 0);
//   display.println("Sent: " + String(messageToSend));
  
//   display.display();
// }

static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}


void serialEvent() { // Receives input from RFM.
 while (Serial1.available()) {
   char inChar = (char)Serial1.read();
   inputString += inChar;
   if (inChar == '\n') {
     stringComplete = true;
   }
 }
}

// Helper function for simple OLED display
void oledDisplay(const char* text) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.println(F(text));
    display.display();
}