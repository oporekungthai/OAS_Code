
void readSensors() {
  qmc.read();
  while (Serial1.available()) gps.encode(Serial1.read());
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
}
