/*// arduino_sensor_bridge.ino
// Reads BNO085 via I2C and sends data to STM32 via SPI
//
// ============================================================================
// PIN CONNECTIONS - Nano ESP32 to STM32 MCU (SPI Communication)
// ============================================================================
// Nano ESP32 Pin  →  STM32 Pin    →  Function
// ----------------------------------------------------------------------------
// D10 (GPIO21)    →  PA11          →  SPI_CE (Chip Select, active LOW)
// D11 (GPIO38)    →  PB5           →  SPI_MOSI/COPI (Controller Out - data to MCU)
// D12 (GPIO47)    →  (not used)    →  SPI_MISO/CIPO (not needed, MCU doesn't send data)
// D13 (GPIO48)    →  PB3           →  SPI_SCK (SPI Clock)
// GND              →  GND           →  Common Ground (REQUIRED!)
//
// ============================================================================
// PIN CONNECTIONS - BNO085 Sensor to Nano ESP32 (I2C Communication)
// ============================================================================
// BNO085 Pin   →  Nano ESP32 Pin  →  Function
// ----------------------------------------------------------------------------
// SDA          →  A4 (GPIO11)      →  I2C Data (default I2C SDA)
// SCL          →  A5 (GPIO12)      →  I2C Clock (default I2C SCL)
// RESET        →  D2 (GPIO5)       →  Reset pin
// 3.3V         →  3.3V             →  Power
// GND          →  GND               →  Ground
// ADR          →  (GND or 3.3V)    →  I2C Address select (LOW=0x4A, HIGH=0x4B)
//
// ============================================================================

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// BNO085 Reset pin (D2 = GPIO5 on Nano ESP32)
#define BNO08X_RESET 2

// MCU SPI Chip Select pin (D10 = GPIO21 on Nano ESP32 → STM32 PA11)
#define MCU_SPI_CS 10

sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

struct sensor_data_t {
  float quat_w, quat_x, quat_y, quat_z;
  float gyro_x, gyro_y, gyro_z;
  bool quat_valid;
  bool gyro_valid;
} latest_sensor_data;

// Euler angles structure (in degrees)
struct euler_t {
  float roll;
  float pitch;
  float yaw;
} latest_euler;

void setReports() {
  bno08x.enableReport(reportType, reportIntervalUs);
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, reportIntervalUs);
}

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Wire.begin();
  delay(100);
  
  if (!bno08x.begin_I2C()) {
    Serial.println("BNO08x not found");
    while (1) delay(10);
  }
  
  latest_sensor_data.quat_valid = false;
  latest_sensor_data.gyro_valid = false;
  setReports();
  
  // Initialize SPI for MCU communication using default Nano ESP32 SPI pins
  // Default SPI pins: D11=COPI/MOSI (GPIO38), D12=CIPO/MISO (GPIO47), D13=SCK (GPIO48)
  // CS pin: D10 (GPIO21) → STM32 PA11
  pinMode(MCU_SPI_CS, OUTPUT);
  digitalWrite(MCU_SPI_CS, HIGH);  // CS high = deselected
  SPI.begin();
}

// Convert quaternion to Euler angles (degrees)
// Based on Code_for_C_imp/lib/src/bno055.c bno055_quaternion_to_euler()
void quaternion_to_euler(float w, float x, float y, float z, struct euler_t *euler) {
  // Convert quaternion to Euler angles (same formula as bno055_quaternion_to_euler)
  euler->roll = atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
  euler->pitch = asin(2.0f * (w * y - z * x));
  euler->yaw = atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
  
  // Convert to degrees
  euler->roll = euler->roll * 180.0f / M_PI;
  euler->pitch = euler->pitch * 180.0f / M_PI;
  euler->yaw = euler->yaw * 180.0f / M_PI;
}

// Convert Euler angle (degrees) to int16_t scaled by 100 (0.01 degree resolution)
int16_t eulerToInt16(float euler_deg) {
  float scaled = euler_deg * 100.0f;
  if (scaled > 32767.0f) scaled = 32767.0f;
  if (scaled < -32768.0f) scaled = -32768.0f;
  return (int16_t)scaled;
}

// Convert gyroscope (rad/s) to int16_t scaled by 2000
int16_t gyroToInt16(float gyro_rad_per_sec) {
  float scaled = gyro_rad_per_sec * 2000.0f;
  if (scaled > 32767.0f) scaled = 32767.0f;
  if (scaled < -32768.0f) scaled = -32768.0f;
  return (int16_t)scaled;
}

void sendSensorPacketToMCU(void) {
  if (!latest_sensor_data.quat_valid) return;
  
  // Convert quaternion to Euler angles (done on Arduino, not MCU)
  quaternion_to_euler(
    latest_sensor_data.quat_w,
    latest_sensor_data.quat_x,
    latest_sensor_data.quat_y,
    latest_sensor_data.quat_z,
    &latest_euler
  );
  
  uint8_t packet[16];
  packet[0] = 0xAA;  // Header
  
  // Pack Euler angles (int16_t scaled by 100, so 1 = 0.01 degrees)
  int16_t roll = eulerToInt16(latest_euler.roll);
  int16_t pitch = eulerToInt16(latest_euler.pitch);
  int16_t yaw = eulerToInt16(latest_euler.yaw);
  
  packet[1] = (roll >> 8) & 0xFF;
  packet[2] = roll & 0xFF;
  packet[3] = (pitch >> 8) & 0xFF;
  packet[4] = pitch & 0xFF;
  packet[5] = (yaw >> 8) & 0xFF;
  packet[6] = yaw & 0xFF;
  
  // Pack gyroscope data
  int16_t gyro_x = latest_sensor_data.gyro_valid ? gyroToInt16(latest_sensor_data.gyro_x) : 0;
  int16_t gyro_y = latest_sensor_data.gyro_valid ? gyroToInt16(latest_sensor_data.gyro_y) : 0;
  int16_t gyro_z = latest_sensor_data.gyro_valid ? gyroToInt16(latest_sensor_data.gyro_z) : 0;
  
  packet[7] = (gyro_x >> 8) & 0xFF;
  packet[8] = gyro_x & 0xFF;
  packet[9] = (gyro_y >> 8) & 0xFF;
  packet[10] = gyro_y & 0xFF;
  packet[11] = (gyro_z >> 8) & 0xFF;
  packet[12] = gyro_z & 0xFF;
  
  // Flags: bit 0 = Euler valid, bit 1 = Gyro valid
  uint8_t flags = 0;
  if (latest_sensor_data.quat_valid) flags |= 0x01;  // Euler valid (converted from quat)
  if (latest_sensor_data.gyro_valid) flags |= 0x02;
  packet[13] = flags;
  
  // Reserved bytes (14-15)
  packet[14] = 0x00;
  packet[15] = 0x00;
  
  // Send packet via SPI to MCU using default SPI pins
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(MCU_SPI_CS, LOW);  // Select MCU (CS low)
  for (int i = 0; i < 16; i++) {
    SPI.transfer(packet[i]);
  }
  digitalWrite(MCU_SPI_CS, HIGH);  // Deselect MCU (CS high)
  SPI.endTransaction();
}

void loop() {
  static unsigned long last_packet_send = 0;
  unsigned long current_time = millis();
  
  if (bno08x.wasReset()) {
    setReports();
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
      latest_sensor_data.quat_w = sensorValue.un.arvrStabilizedRV.real;
      latest_sensor_data.quat_x = sensorValue.un.arvrStabilizedRV.i;
      latest_sensor_data.quat_y = sensorValue.un.arvrStabilizedRV.j;
      latest_sensor_data.quat_z = sensorValue.un.arvrStabilizedRV.k;
      latest_sensor_data.quat_valid = true;
    }
    else if (sensorValue.sensorId == SH2_GYRO_INTEGRATED_RV) {
      latest_sensor_data.quat_w = sensorValue.un.gyroIntegratedRV.real;
      latest_sensor_data.quat_x = sensorValue.un.gyroIntegratedRV.i;
      latest_sensor_data.quat_y = sensorValue.un.gyroIntegratedRV.j;
      latest_sensor_data.quat_z = sensorValue.un.gyroIntegratedRV.k;
      latest_sensor_data.quat_valid = true;
    }
    else if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
      latest_sensor_data.gyro_x = sensorValue.un.gyroscope.x;
      latest_sensor_data.gyro_y = sensorValue.un.gyroscope.y;
      latest_sensor_data.gyro_z = sensorValue.un.gyroscope.z;
      latest_sensor_data.gyro_valid = true;
    }
  }
  
  if (current_time - last_packet_send >= 50) {
    sendSensorPacketToMCU();
    last_packet_send = current_time;
  }
}*/


#include <Arduino.h>
// This demo explores two reports (SH2_ARVR_STABILIZED_RV and SH2_GYRO_INTEGRATED_RV) both can be used to give 
// quartenion and euler (yaw, pitch roll) angles.  Toggle the FAST_MODE define to see other report.  
// Note sensorValue.status gives calibration accuracy (which improves over time)
#include <Adafruit_BNO08x.h>
#include <SPI.h>

// For SPI mode, we need a CS pin
//#define BNO08X_CS 10
#define BNO08X_INT 9

// MCU SPI Chip Select pin (D10 = GPIO21 on Nano ESP32 → STM32 PA11)
#define MCU_SPI_CS 10


// #define FAST_MODE

// For SPI mode, we also need a RESET 
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
  // Also enable gyroscope reports
  if (! bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, report_interval)) {
    Serial.println("Could not enable gyroscope");
  }
}

void setup(void) {

  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");


  setReports(reportType, reportIntervalUs);

  Serial.println("Reading events");
  delay(100);

  // Initialize SPI for MCU communication using default Nano ESP32 SPI pins
  // Default SPI pins: D11=COPI/MOSI (GPIO38), D12=CIPO/MISO (GPIO47), D13=SCK (GPIO48)
  // CS pin: D10 (GPIO21) → STM32 PA11
  pinMode(MCU_SPI_CS, OUTPUT);
  digitalWrite(MCU_SPI_CS, HIGH);  // CS high = deselected
  SPI.begin();
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

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

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

// Convert Euler angle (degrees) to int16_t scaled by 100 (0.01 degree resolution)
int16_t eulerToInt16(float euler_deg) {
  float scaled = euler_deg * 100.0f;
  if (scaled > 32767.0f) scaled = 32767.0f;
  if (scaled < -32768.0f) scaled = -32768.0f;
  return (int16_t)scaled;
}

// Convert gyroscope (rad/s) to int16_t scaled by 2000
int16_t gyroToInt16(float gyro_rad_per_sec) {
  float scaled = gyro_rad_per_sec * 2000.0f;
  if (scaled > 32767.0f) scaled = 32767.0f;
  if (scaled < -32768.0f) scaled = -32768.0f;
  return (int16_t)scaled;
}

void loop() {
  static unsigned long last_packet_send = 0;
  static bool euler_valid = false;
  static bool gyro_valid = false;
  static float current_roll = 0.0f;
  static float current_pitch = 0.0f;
  static float current_yaw = 0.0f;
  static float current_gyro_x = 0.0f;
  static float current_gyro_y = 0.0f;
  static float current_gyro_z = 0.0f;

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        current_roll = ypr.roll;
        current_pitch = ypr.pitch;
        current_yaw = ypr.yaw;
        euler_valid = true;
        break;
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        current_roll = ypr.roll;
        current_pitch = ypr.pitch;
        current_yaw = ypr.yaw;
        euler_valid = true;
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        current_gyro_x = sensorValue.un.gyroscope.x;
        current_gyro_y = sensorValue.un.gyroscope.y;
        current_gyro_z = sensorValue.un.gyroscope.z;
        gyro_valid = true;
        break;
    }
    static long last = 0;
    long now = micros();
    Serial.print(now - last);             Serial.print("\t");
    last = now;
    Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
    Serial.print(ypr.roll);               Serial.print("\t");
    Serial.print(ypr.pitch);              Serial.print("\t");
    Serial.println(ypr.yaw);
  }

  // Send full packet via SPI to MCU (throttled to ~20Hz)
  unsigned long current_time = millis();
  if (euler_valid && (current_time - last_packet_send >= 50)) {
    uint8_t packet[16];
    packet[0] = 0xAA;  // Header
    
    // Pack Euler angles in order: Roll, Pitch, Yaw (matching print statement order)
    // int16_t scaled by 100, so 1 = 0.01 degrees
    // Bytes 1-2: Roll
    int16_t roll = eulerToInt16(current_roll);
    packet[1] = (roll >> 8) & 0xFF;   // Roll MSB
    packet[2] = roll & 0xFF;           // Roll LSB
    
    // Bytes 3-4: Pitch
    int16_t pitch = eulerToInt16(current_pitch);
    packet[3] = (pitch >> 8) & 0xFF;  // Pitch MSB
    packet[4] = pitch & 0xFF;          // Pitch LSB
    
    // Bytes 5-6: Yaw
    int16_t yaw = eulerToInt16(current_yaw);
    packet[5] = (yaw >> 8) & 0xFF;    // Yaw MSB
    packet[6] = yaw & 0xFF;            // Yaw LSB
    
    // Pack gyroscope data (int16_t scaled by 2000)
    int16_t gyro_x = gyro_valid ? gyroToInt16(current_gyro_x) : 0;
    int16_t gyro_y = gyro_valid ? gyroToInt16(current_gyro_y) : 0;
    int16_t gyro_z = gyro_valid ? gyroToInt16(current_gyro_z) : 0;
    
    packet[7] = (gyro_x >> 8) & 0xFF;
    packet[8] = gyro_x & 0xFF;
    packet[9] = (gyro_y >> 8) & 0xFF;
    packet[10] = gyro_y & 0xFF;
    packet[11] = (gyro_z >> 8) & 0xFF;
    packet[12] = gyro_z & 0xFF;
    
    // Flags: bit 0 = Euler valid, bit 1 = Gyro valid
    uint8_t flags = 0;
    if (euler_valid) flags |= 0x01;
    if (gyro_valid) flags |= 0x02;
    packet[13] = flags;
    
    // Reserved bytes (14-15)
    packet[14] = 0x00;
    packet[15] = 0x00;
    
    // Send packet via SPI to MCU using default SPI pins
    SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
    digitalWrite(MCU_SPI_CS, LOW);  // Select MCU (CS low)
    for (int i = 0; i < 16; i++) {
      SPI.transfer(packet[i]);
    }
    digitalWrite(MCU_SPI_CS, HIGH);  // Deselect MCU (CS high)
    SPI.endTransaction();
    
    last_packet_send = current_time;
  }
}
