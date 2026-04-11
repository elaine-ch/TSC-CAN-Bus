#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Initialize GPS parser and Hardware Serial on UART2
TinyGPSPlus gps;
HardwareSerial GPS_Serial(2); 

// --- Odometer Variables ---
double totalDistance = 0.0;  // Cumulative distance in meters
double lastLat = 0.0;
double lastLng = 0.0;
bool hasValidHistory = false;

// --- Settings ---
const float DISTANCE_THRESHOLD = 4.0; // Minimum meters moved to count (filters drift)
const float SPEED_THRESHOLD = 1.5;    // Minimum km/h to count (filters jitter)

void setup() {
  Serial.begin(115200);
  // GPS Baud rate is 9600. RX=16, TX=17
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);
  
  Serial.println("========================================");
  Serial.println("      ESP32 GPS ACCUMULATOR START       ");
  Serial.println("========================================");
  Serial.println("Waiting for satellite lock...");
}

void loop() {
  // Feed raw data from GPS module into TinyGPS++
  while (GPS_Serial.available() > 0) {
    if (gps.encode(GPS_Serial.read())) {
      // Only process distance if the location just updated
      if (gps.location.isUpdated()) {
        updateOdometer();
      }
    }
  }

  // Refresh the Serial Monitor display every 2 seconds
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 2000) {
    printDashboard();
    lastPrint = millis();
  }
}

void updateOdometer() {
  // Requirement 1: Must have a valid location
  // Requirement 2: Must see at least 4 satellites for accuracy
  if (!gps.location.isValid() || gps.satellites.value() < 4) {
    return;
  }

  double currentLat = gps.location.lat();
  double currentLng = gps.location.lng();

  // If this is our very first valid point, just save it and return
  if (!hasValidHistory) {
    lastLat = currentLat;
    lastLng = currentLng;
    hasValidHistory = true;
    return;
  }

  // Calculate distance from the previous point to the current point
  double distanceStep = TinyGPSPlus::distanceBetween(currentLat, currentLng, lastLat, lastLng);

  // LOGIC: Only accumulate if we moved more than the threshold AND we are actually moving (speed)
  // This prevents the "distance climbing while sitting on a desk" issue.
  if (distanceStep > DISTANCE_THRESHOLD && gps.speed.kmph() > SPEED_THRESHOLD) {
    totalDistance += distanceStep;
    
    // Move the "starting point" to our new current location
    lastLat = currentLat;
    lastLng = currentLng;
  }
}

void printDashboard() {
  Serial.println("\n--- GPS LIVE DASHBOARD ---");
  
  if (gps.location.isValid()) {
    Serial.print("STATUS:    FIXED");
    Serial.print("  | Satellites: "); Serial.println(gps.satellites.value());
    
    Serial.print("LAT:       "); Serial.println(gps.location.lat(), 6);
    Serial.print("LNG:       "); Serial.println(gps.location.lng(), 6);
    
    Serial.print("SPEED:     "); Serial.print(gps.speed.kmph(), 1); Serial.println(" km/h");
    
    Serial.println("--------------------------");
    Serial.print("TRIP DIST: "); 
    if (totalDistance < 1000) {
      Serial.print(totalDistance, 1); Serial.println(" meters");
    } else {
      Serial.print(totalDistance / 1000.0, 3); Serial.println(" kilometers");
    }
    
    // Google Maps Link for easy clicking
    Serial.print("LOCATION:  https://www.google.com/maps?q=");
    Serial.print(gps.location.lat(), 6);
    Serial.print(",");
    Serial.println(gps.location.lng(), 6);

  } else {
    Serial.print("STATUS:    SEARCHING...");
    Serial.print(" | Sats in View: "); Serial.println(gps.satellites.value());
    Serial.println("Note: Take device outside for faster lock.");
  }
  Serial.println("--------------------------");
}