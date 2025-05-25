
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


