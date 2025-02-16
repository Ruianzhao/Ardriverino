#include <Wire.h>
#include <avr/wdt.h>  // Include the watchdog timer library
#include <math.h>

const int MPU = 0x68; // MPU6050 I2C address
unsigned long previousPrintTime = 0;  // Stores the last print timestamp
const int printInterval = 333;  // Print every 333ms (3 times per second)

float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;


const int forceSensorPin = A0; // Force sensor connected to A0

float forceValue;
float forceBaseline = 0;  // Baseline force value for calibration
float forceFiltered = 0;   // Filtered force value
const float alpha = 0.9;   // Smoothing factor for filtering


void setup() {
  Serial.begin(19200);
  Wire.begin();                      // Initialize communication
  
  // Enable watchdog timer (resets if no response within 2 seconds)
  wdt_enable(WDTO_2S);

  Wire.beginTransmission(MPU);       // Start communication with MPU6050
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);

    pinMode(forceSensorPin, INPUT);
  
  // **Calibrate the Force Sensor**
  calibrateForceSensor();

  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  // Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  // Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);

  delay(20);
  
  // Call this function to calculate IMU error values
  calculate_IMU_error();
  delay(20);
}

void loop() {
  wdt_reset(); 

  // === Read accelerometer data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  if (Wire.available() < 6) {
    Serial.println("I2C Error: No response from MPU6050");
    resetMPU6050();
    return;
  }

  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

  float accPitch = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 180 / PI;

  // === Read gyroscope data === //
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;  

  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  if (Wire.available() < 6) {
    Serial.println("I2C Error: No response from MPU6050");
    resetMPU6050();
    return;
  }

  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0 - GyroErrorY;

  // Apply Complementary Filter to Correct Drift
  const float alpha = 0.96;
  pitch = alpha * (pitch + GyroY * elapsedTime) + (1 - alpha) * accPitch;

  // **Simulate force sensor data (Replace this with actual force sensor if needed)**
  int force = analogRead(A0);  // Assume force sensor is on A0

  // **Print PITCH and FORCE every loop iteration**
  Serial.flush();
  Serial.print("PITCH: ");
  Serial.print(pitch);
  Serial.print(" | FORCE: ");
  Serial.println(force);

  delay(333);  // Print rate = 3 times per second
}





void calculate_IMU_error() {
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

    AccErrorX += (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI);
    AccErrorY += (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI);
    c++;
  }
  
  AccErrorX /= 200;
  AccErrorY /= 200;
  c = 0;
}

void resetMPU6050() {
  Serial.println("Resetting MPU6050...");
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00); // Reset MPU6050
  Wire.endTransmission(true);
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
