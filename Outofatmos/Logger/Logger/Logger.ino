#include <SD.h>
#include <SPI.h>

const int chipSelect = 25; // Replace with your CS pin

void setup() {
  Serial.begin(115200);
  while (!Serial) ; // Wait for serial connection

  if (!SD.begin(chipSelect)) {
    Serial.println("SD card failed!");
    return;
  }

  Serial.println("SD card initialized.");

  File root = SD.open("/");
  listFiles(root);
}

void loop() {
  // nothing here
}

void listFiles(File dir) {
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) break;

    if (!entry.isDirectory() && String(entry.name()).startsWith("data")) {
      Serial.print("Reading: ");
      Serial.println(entry.name());

      while (entry.available()) {
        Serial.write(entry.read());
      }
      Serial.println(); // Space between files
    }
    entry.close();
  }
}
