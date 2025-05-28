void sendData(String data) {
  // Convert String to char array.
  char messageToSend[data.length() + 1];
  data.toCharArray(messageToSend, sizeof(messageToSend));

  // Send message
  rf95.send((uint8_t *)messageToSend, sizeof(messageToSend));
  rf95.waitPacketSent(); // Wait for transmission to complete

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Sent: " + String(messageToSend));
  
  display.display();
}