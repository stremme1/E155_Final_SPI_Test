// arduino_sensor_bridge.ino
// Arduino Sensor Bridge: Reads BNO085 via I2C and sends data to STM32 via SPI
//
// Hardware Connections:
// BNO085 - Arduino (I2C):
//   Sensor SDA → Arduino SDA (A4)
//   Sensor SCL → Arduino SCL (A5)
//   Sensor 3.3V → Arduino 3.3V
//   Sensor GND → Arduino GND
//
// Arduino → STM32 (SPI):
//   Arduino D13 (SCK) → STM32 PB3 (SCK)
//   Arduino D11 (MOSI) → STM32 PB5 (MOSI)
//   Arduino D12 (MISO) → STM32 PB4 (MISO)
//   Arduino D7 (CS) → STM32 PA11 (NSS)
//   Arduino GND → STM32 GND (common ground)

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <SPI.h>
#include <Wire.h>  // For I2C communication

// BNO085 I2C configuration (no CS or INT needed for I2C)
#define BNO08X_RESET -1  // Not used for I2C

// SPI configuration for STM32 communication
#define SPI_CS_PIN 7     // Chip select pin for STM32 (D7)
#define SPI_CLOCK_SPEED 1000000  // 1 MHz SPI clock (fast enough, not too fast)

// Sensor report configuration
#define REPORT_INTERVAL_US 10000  // 100Hz (10000 microseconds = 10ms)

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Euler angles structure (from quaternion conversion)
struct euler_t {
  float yaw;
  float pitch;
  float roll;
} euler;

// Gyroscope data storage
int16_t gyro_x = 0, gyro_y = 0, gyro_z = 0;
bool has_quaternion = false;
bool has_gyroscope = false;

// Packet structure: [Sync(0xAA)][SensorID][Timestamp(4)][Roll(4)][Pitch(4)][Yaw(4)][GyroX(2)][GyroY(2)][GyroZ(2)]
// Total: 24 bytes (matches Code_for_C_imp data requirements)
#define PACKET_SIZE 24
#define SYNC_BYTE 0xAA

void setup(void) {
  // Initialize Serial for debugging
  Serial.begin(115200);
  while (!Serial) delay(10);  // Wait for serial console to open
  
  Serial.println("Arduino Sensor Bridge Starting...");
  Serial.println("BNO085 I2C + SPI to STM32");
  
  // Initialize I2C bus first
  Wire.begin();  // Initialize I2C (SDA=A4, SCL=A5 for most Arduino boards)
  
  // Scan I2C bus to see what devices are connected
  Serial.println("Scanning I2C bus...");
  byte error, address;
  int nDevices = 0;
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found!");
    Serial.println("Check connections:");
    Serial.println("  SDA → A4 (or SDA pin for your board)");
    Serial.println("  SCL → A5 (or SCL pin for your board)");
    Serial.println("  3.3V and GND connected");
    Serial.println("  I2C pull-up resistors (usually 4.7kΩ on SDA/SCL)");
  } else {
    Serial.print("Found ");
    Serial.print(nDevices);
    Serial.println(" device(s)");
  }
  Serial.println();
  
  // Initialize BNO085 via I2C (default address 0x4A)
  Serial.println("Initializing BNO085 via I2C (address 0x4A)...");
  if (!bno08x.begin_I2C()) {
    Serial.println("ERROR: Failed to find BNO08x chip at address 0x4A!");
    Serial.println("Trying alternative address 0x4B...");
    // Some BNO085 boards use 0x4B instead
    if (!bno08x.begin_I2C(0x4B)) {
      Serial.println("ERROR: Failed to find BNO08x chip at address 0x4B!");
      Serial.println();
      Serial.println("Troubleshooting:");
      Serial.println("1. Check I2C connections:");
      Serial.println("   SDA → A4 (or your board's SDA pin)");
      Serial.println("   SCL → A5 (or your board's SCL pin)");
      Serial.println("2. Check power:");
      Serial.println("   VIN/3.3V → 3.3V");
      Serial.println("   GND → GND");
      Serial.println("3. I2C pull-up resistors:");
      Serial.println("   Usually 4.7kΩ on SDA and SCL to 3.3V");
      Serial.println("   (Some boards have these built-in)");
      Serial.println("4. Check if sensor appears in I2C scan above");
      while (1) { delay(10); }  // Halt on error
    } else {
      Serial.println("BNO08x Found at address 0x4B!");
    }
  } else {
    Serial.println("BNO08x Found at address 0x4A!");
  }
  
  // Enable quaternion report (Game Rotation Vector for better performance)
  Serial.println("Enabling quaternion reports...");
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, REPORT_INTERVAL_US)) {
    Serial.println("ERROR: Could not enable quaternion report!");
    while (1) { delay(10); }  // Halt on error
  }
  Serial.println("Quaternion reports enabled");
  
  // Enable gyroscope report (needed for drum trigger detection)
  Serial.println("Enabling gyroscope reports...");
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, REPORT_INTERVAL_US)) {
    Serial.println("ERROR: Could not enable gyroscope report!");
    while (1) { delay(10); }  // Halt on error
  }
  Serial.println("Gyroscope reports enabled");
  
  // Initialize SPI as Master
  Serial.println("Initializing SPI Master...");
  pinMode(SPI_CS_PIN, OUTPUT);
  digitalWrite(SPI_CS_PIN, HIGH);  // CS high (idle, STM32 not selected)
  
  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_CLOCK_SPEED, MSBFIRST, SPI_MODE0));  // Mode 0: CPOL=0, CPHA=0
  
  Serial.println("SPI Master initialized");
  Serial.print("  Clock: "); Serial.print(SPI_CLOCK_SPEED / 1000); Serial.println(" kHz");
  Serial.print("  CS Pin: "); Serial.println(SPI_CS_PIN);
  Serial.print("  Mode: 0 (CPOL=0, CPHA=0)");
  Serial.println();
  
  Serial.println("Setup complete! Starting sensor data transmission...");
  Serial.println();
  delay(100);
}

// Convert quaternion to Euler angles (from Adafruit example)
// Returns angles in degrees
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = true) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

// Convert BNO085 gyroscope (rad/s, float) to int16_t format (matching Code_for_C_imp)
// BNO085 gyro is in rad/s, Code_for_C_imp expects int16_t values
// Scale factor: multiply by ~400 to get similar range to BNO055
// Thresholds: gyro_y < -2500, gyro_z > -2000
int16_t convertGyroToInt16(float gyro_rad_per_sec) {
  // Convert rad/s to a scaled int16_t value
  // BNO085: typically -2 to +2 rad/s range
  // Code_for_C_imp: uses raw int16_t values with thresholds around -2500
  // Scale by 2000 to match the threshold range
  float scaled = gyro_rad_per_sec * 2000.0f;
  
  // Clamp to int16_t range
  if (scaled > 32767.0f) scaled = 32767.0f;
  if (scaled < -32768.0f) scaled = -32768.0f;
  
  return (int16_t)scaled;
}

void sendSensorPacket(float roll, float pitch, float yaw, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, uint8_t sensor_id) {
  uint8_t packet[PACKET_SIZE];
  uint32_t timestamp = micros();  // Get current timestamp in microseconds
  
  // Build packet
  packet[0] = SYNC_BYTE;  // Sync byte
  packet[1] = sensor_id;  // Sensor ID
  
  // Timestamp (4 bytes, little-endian)
  packet[2] = (uint8_t)(timestamp & 0xFF);
  packet[3] = (uint8_t)((timestamp >> 8) & 0xFF);
  packet[4] = (uint8_t)((timestamp >> 16) & 0xFF);
  packet[5] = (uint8_t)((timestamp >> 24) & 0xFF);
  
  // Euler angles (12 bytes total: 3 floats)
  // Roll (4 bytes, IEEE 754 float)
  memcpy(&packet[6], &roll, 4);
  
  // Pitch (4 bytes, IEEE 754 float)
  memcpy(&packet[10], &pitch, 4);
  
  // Yaw (4 bytes, IEEE 754 float)
  memcpy(&packet[14], &yaw, 4);
  
  // Gyroscope data (6 bytes total: 3 int16_t, little-endian)
  // Gyro X (2 bytes, little-endian)
  packet[18] = (uint8_t)(gyro_x & 0xFF);
  packet[19] = (uint8_t)((gyro_x >> 8) & 0xFF);
  
  // Gyro Y (2 bytes, little-endian)
  packet[20] = (uint8_t)(gyro_y & 0xFF);
  packet[21] = (uint8_t)((gyro_y >> 8) & 0xFF);
  
  // Gyro Z (2 bytes, little-endian)
  packet[22] = (uint8_t)(gyro_z & 0xFF);
  packet[23] = (uint8_t)((gyro_z >> 8) & 0xFF);
  
  // Send packet via SPI
  digitalWrite(SPI_CS_PIN, LOW);  // Select STM32 slave
  
  for (int i = 0; i < PACKET_SIZE; i++) {
    SPI.transfer(packet[i]);
  }
  
  digitalWrite(SPI_CS_PIN, HIGH);  // Deselect STM32 slave
  
  // Debug output (can be disabled for higher performance)
  static unsigned long last_debug = 0;
  if (millis() - last_debug > 1000) {  // Print once per second
    Serial.print("Sent packet: ID=0x");
    Serial.print(sensor_id, HEX);
    Serial.print(", Euler=[Roll=");
    Serial.print(roll, 2);
    Serial.print(", Pitch=");
    Serial.print(pitch, 2);
    Serial.print(", Yaw=");
    Serial.print(yaw, 2);
    Serial.print("], Gyro=[X=");
    Serial.print(gyro_x);
    Serial.print(", Y=");
    Serial.print(gyro_y);
    Serial.print(", Z=");
    Serial.print(gyro_z);
    Serial.println("]");
    last_debug = millis();
  }
}

void loop() {
  // Check if sensor was reset
  if (bno08x.wasReset()) {
    Serial.println("BNO085 was reset - re-enabling reports");
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, REPORT_INTERVAL_US)) {
      Serial.println("ERROR: Could not re-enable quaternion report!");
    }
    if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, REPORT_INTERVAL_US)) {
      Serial.println("ERROR: Could not re-enable gyroscope report!");
    }
  }
  
  // Check for new sensor data
  while (bno08x.getSensorEvent(&sensorValue)) {
    // Process quaternion data
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      sh2_RotationVector_t* quat = &sensorValue.un.gameRotationVector;
      
      // Convert quaternion to Euler angles (using proven Adafruit function)
      quaternionToEuler(quat->real, quat->i, quat->j, quat->k, &euler, true);
      has_quaternion = true;
    }
    
    // Process gyroscope data
    if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
      // BNO085 gyroscope is in rad/s (float)
      // Convert to int16_t format matching Code_for_C_imp
      gyro_x = convertGyroToInt16(sensorValue.un.gyroscope.x);
      gyro_y = convertGyroToInt16(sensorValue.un.gyroscope.y);
      gyro_z = convertGyroToInt16(sensorValue.un.gyroscope.z);
      has_gyroscope = true;
    }
    
    // Send packet when we have both quaternion and gyroscope data
    // This ensures we send synchronized data
    if (has_quaternion && has_gyroscope) {
      sendSensorPacket(
        euler.roll,
        euler.pitch,
        euler.yaw,
        gyro_x,
        gyro_y,
        gyro_z,
        SH2_GAME_ROTATION_VECTOR  // Sensor ID
      );
      // Reset flags (will be set again on next sensor readings)
      has_quaternion = false;
      has_gyroscope = false;
    }
  }
  
  // Small delay to prevent overwhelming the system
  // The sensor report interval (10ms) already limits the rate
  delayMicroseconds(100);
}

