#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

unsigned long lastUpdate = 0;
unsigned long counter = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  // Backlight
  pinMode(16, OUTPUT);
  digitalWrite(16, HIGH);

  // Initialize display
  tft.init();
  tft.setRotation(1);

  // Clear screen
  tft.fillScreen(TFT_BLACK);

  // Hello World
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(3);
  tft.setCursor(20, 30);
  tft.println("Hello World");

  // Counter label
  tft.setTextSize(2);
  tft.setCursor(20, 100);
  tft.println("Counter:");

  Serial.println("Display initialized.");
}

void loop() {
  if (millis() - lastUpdate >= 1000) {
    lastUpdate = millis();
    counter++;

    // Clear previous counter
    tft.fillRect(20, 140, 300, 40, TFT_BLACK);

    // Draw counter
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextSize(3);
    tft.setCursor(20, 140);
    tft.print(counter);

    Serial.print("Counter: ");
    Serial.println(counter);
  }
}