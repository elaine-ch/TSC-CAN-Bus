// We use Serial2 for ESP32. It uses Pin 16 (RX) and Pin 17 (TX) by default.
#define NEXTION_SERIAL Serial2 

int counter = 0; 
//int second_counter = 0;

void setup() {
  // FORCE Serial2 to specifically use Pin 16 (RX) and Pin 17 (TX)
  NEXTION_SERIAL.begin(9600, SERIAL_8N1, 16, 17); 
  
  // Start the laptop monitor
  Serial.begin(115200); 
  Serial.println("ESP32 Started. Sending to Nextion...");
}

void loop() {
  // 1. Send the command to update the specific object (n0) and its value (.val)
  // We want the Nextion to see: n0.val=X
  NEXTION_SERIAL.print("n0.val=");
  NEXTION_SERIAL.print(counter);
  
  // 2. Send the three "End of Message" bytes (0xFF)
  NEXTION_SERIAL.write(0xff);
  NEXTION_SERIAL.write(0xff);
  NEXTION_SERIAL.write(0xff);

  //TO ADD ANOTHER COUNTER ********************************
  /*
  NEXTION_SERIAL.print("n1.val=");
  NEXTION_SERIAL.print(second_counter);
  
  
  NEXTION_SERIAL.write(0xff);
  NEXTION_SERIAL.write(0xff);
  NEXTION_SERIAL.write(0xff);
*/
  // ****************************************



  // Print to laptop serial monitor for debugging
  Serial.print("Sent counter: ");
  Serial.println(counter);
/*
  Serial.print("Sent counter: ");
  Serial.println(second_counter);
*/

  
  //Increment and delay
  counter++;
//  second_counter++;
  delay(1000); 
}