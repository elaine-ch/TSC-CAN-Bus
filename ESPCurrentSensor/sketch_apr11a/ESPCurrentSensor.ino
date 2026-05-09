const int sensorPin = 34;
const float VCC = 3.3;
const float sensitivity = 0.043; // V/A for WCS8001
float zeroPoint = 0;             // This will be calculated at startup

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  Serial.println("Calibrating sensor... Ensure no current is flowing.");
  
  // Take 100 readings to find the baseline "Zero"
  float totalVoltage = 0;
  for (int i = 0; i < 100; i++) {
    totalVoltage += (analogRead(sensorPin) / 4095.0) * VCC;
    delay(10);
  }
  zeroPoint = totalVoltage / 100.0;
  
  Serial.print("Calibration Complete. Zero Point set to: ");
  Serial.print(zeroPoint);
  Serial.println("V");
}

void loop() {
  float totalCurrent = 0;
  int samples = 50; // Smooth out the jumpy values

  for (int i = 0; i < samples; i++) {
    float voltage = (analogRead(sensorPin) / 4095.0) * VCC;
    float current = (voltage - zeroPoint) / sensitivity;
    totalCurrent += current;
  }

  float averageCurrent = totalCurrent / samples;

  Serial.print("Current: ");
  Serial.print(averageCurrent, 3); // Show 3 decimal places
  Serial.println(" A");

  delay(500);
}