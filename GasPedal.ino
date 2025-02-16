#include <Wire.h>
#include <avr/wdt.h>  // Watchdog Timer

const int forceSensorPin = A0; // Force sensor connected to A0
unsigned long previousPrintTime = 0;  
const int printInterval = 333;  // Print every 333ms (3 times per second)

float forceValue;
float forceBaseline = 0;  // Baseline force value for calibration
float forceFiltered = 0;   // Filtered force value
const float alpha = 0.9;   // Smoothing factor for filtering

void setup() {
  Serial.begin(19200);
  wdt_enable(WDTO_2S);  // Enable watchdog timer (resets if unresponsive)
  
  pinMode(forceSensorPin, INPUT);
  
  // **Calibrate the Force Sensor**
  calibrateForceSensor();
}

void loop() {
  wdt_reset();  // Reset watchdog timer to prevent auto-reset

  // **Read Force Sensor (FSR) Data**
  int rawForce = analogRead(forceSensorPin);
  forceValue = map(rawForce, 0, 1023, 0, 1000);  // Convert 0-1023 to 0-1000 range

  // **Apply Smoothing Filter**
  forceFiltered = alpha * forceFiltered + (1 - alpha) * forceValue;

  // **Print Data at Intervals**
  if (millis() - previousPrintTime >= printInterval) {
    previousPrintTime = millis();  // Update last print time
    
    Serial.flush();
    Serial.print("FORCE: ");
    Serial.println(forceFiltered);
  }

  delay(5);
}

// **Calibrate Force Sensor to Find Baseline**
void calibrateForceSensor() {
  Serial.println("Calibrating force sensor...");
  int total = 0;

  for (int i = 0; i < 50; i++) {
    total += analogRead(forceSensorPin);
    delay(10);
  }

  forceBaseline = total / 50.0;  // Average baseline force
  Serial.print("Calibration Complete. Baseline Force: ");
  Serial.println(forceBaseline);
}