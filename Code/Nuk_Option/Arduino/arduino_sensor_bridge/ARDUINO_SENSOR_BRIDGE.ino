#include <Arduino.h>
// This demo explores two reports (SH2_ARVR_STABILIZED_RV and SH2_GYRO_INTEGRATED_RV) both can be used to give 
// quartenion and euler (yaw, pitch roll) angles.  Toggle the FAST_MODE define to see other report.  
// Note sensorValue.status gives calibration accuracy (which improves over time)
#include <Adafruit_BNO08x.h>
#include <SPI.h>

// For SPI mode, we need a CS pin
//#define BNO08X_CS 10
#define BNO08X_INT 9

// FPGA SPI Chip Select pin (D10 = GPIO21 on Nano ESP32 → FPGA arduino_cs_n)
// Note: Arduino sends to FPGA first, FPGA then forwards to MCU
// Architecture: Arduino (Master) → FPGA (Slave) → MCU (Master reads from FPGA)
#define FPGA_SPI_CS 10


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

  // Initialize SPI for FPGA communication using default Nano ESP32 SPI pins
  // Default SPI pins: D11=COPI/MOSI (GPIO38), D12=CIPO/MISO (GPIO47), D13=SCK (GPIO48)
  // CS pin: D10 (GPIO21) → FPGA arduino_cs_n input
  // Architecture: Arduino (Master) → FPGA (Slave) → MCU (Master reads from FPGA)
  pinMode(FPGA_SPI_CS, OUTPUT);
  digitalWrite(FPGA_SPI_CS, HIGH);  // CS high = deselected
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

// Convert quaternion (Q14 format) to int16_t
// Q14 format: multiply by 16384 to get integer representation
int16_t quatToInt16(float quat) {
  float scaled = quat * 16384.0f;
  if (scaled > 32767.0f) scaled = 32767.0f;
  if (scaled < -32768.0f) scaled = -32768.0f;
  return (int16_t)scaled;
}

void loop() {
  static unsigned long last_packet_send = 0;
  static bool quat_valid = false;
  static bool gyro_valid = false;
  static float current_quat_w = 1.0f;
  static float current_quat_x = 0.0f;
  static float current_quat_y = 0.0f;
  static float current_quat_z = 0.0f;
  static float current_gyro_x = 0.0f;
  static float current_gyro_y = 0.0f;
  static float current_gyro_z = 0.0f;

  // Auto timer based calibration commented out
  // if (bno08x.wasReset()) {
  //   Serial.print("sensor was reset ");
  //   setReports(reportType, reportIntervalUs);
  // }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        // Store quaternion data directly (FPGA will convert to Euler using DSP)
        current_quat_w = sensorValue.un.arvrStabilizedRV.real;
        current_quat_x = sensorValue.un.arvrStabilizedRV.i;
        current_quat_y = sensorValue.un.arvrStabilizedRV.j;
        current_quat_z = sensorValue.un.arvrStabilizedRV.k;
        quat_valid = true;
        // Also compute Euler for serial debug output
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        break;
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        // Store quaternion data directly (FPGA will convert to Euler using DSP)
        current_quat_w = sensorValue.un.gyroIntegratedRV.real;
        current_quat_x = sensorValue.un.gyroIntegratedRV.i;
        current_quat_y = sensorValue.un.gyroIntegratedRV.j;
        current_quat_z = sensorValue.un.gyroIntegratedRV.k;
        quat_valid = true;
        // Also compute Euler for serial debug output
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
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

  // Send full packet via SPI to FPGA (throttled to ~20Hz)
  // Data Pipeline: Arduino (this code) → FPGA (converts quaternion to Euler using DSP) → MCU
  // FPGA receives quaternion data and converts to Euler angles using DSP blocks
  // Packet format (16 bytes): [Header][Quat W][Quat X][Quat Y][Quat Z][Gyro X][Gyro Y][Gyro Z][Flags][Reserved]
  unsigned long current_time = millis();
  if (quat_valid && (current_time - last_packet_send >= 50)) {
    uint8_t packet[16];
    packet[0] = 0xAA;  // Header (Byte 0)
    
    // Pack quaternion in Q14 format (int16_t, scaled by 16384)
    // Bytes 1-2: Quaternion W
    int16_t quat_w = quatToInt16(current_quat_w);
    packet[1] = (quat_w >> 8) & 0xFF;   // W MSB
    packet[2] = quat_w & 0xFF;           // W LSB
    
    // Bytes 3-4: Quaternion X
    int16_t quat_x = quatToInt16(current_quat_x);
    packet[3] = (quat_x >> 8) & 0xFF;  // X MSB
    packet[4] = quat_x & 0xFF;          // X LSB
    
    // Bytes 5-6: Quaternion Y
    int16_t quat_y = quatToInt16(current_quat_y);
    packet[5] = (quat_y >> 8) & 0xFF;    // Y MSB
    packet[6] = quat_y & 0xFF;            // Y LSB
    
    // Bytes 7-8: Quaternion Z
    int16_t quat_z = quatToInt16(current_quat_z);
    packet[7] = (quat_z >> 8) & 0xFF;    // Z MSB
    packet[8] = quat_z & 0xFF;            // Z LSB
    
    // Pack gyroscope data (int16_t scaled by 2000)
    int16_t gyro_x = gyro_valid ? gyroToInt16(current_gyro_x) : 0;
    int16_t gyro_y = gyro_valid ? gyroToInt16(current_gyro_y) : 0;
    int16_t gyro_z = gyro_valid ? gyroToInt16(current_gyro_z) : 0;
    
    packet[9] = (gyro_x >> 8) & 0xFF;
    packet[10] = gyro_x & 0xFF;
    packet[11] = (gyro_y >> 8) & 0xFF;
    packet[12] = gyro_y & 0xFF;
    packet[13] = (gyro_z >> 8) & 0xFF;
    packet[14] = gyro_z & 0xFF;
    
    // Flags: bit 0 = Quaternion valid, bit 1 = Gyro valid
    uint8_t flags = 0;
    if (quat_valid) flags |= 0x01;
    if (gyro_valid) flags |= 0x02;
    packet[15] = flags;
    
  // Send packet via SPI to FPGA using default SPI pins
  // SPI Mode 0 (CPOL=0, CPHA=0): Clock idle LOW, sample on rising edge
  // MSB First: Most significant bit sent first
  // 100kHz clock rate (10us period per bit, 80us per byte, ~1.28ms per 16-byte packet)
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
  digitalWrite(FPGA_SPI_CS, LOW);  // Select FPGA (CS low)
  
  // Send all 16 bytes of the packet
  // FPGA samples data on SCK rising edge (SPI Mode 0)
  for (int i = 0; i < 16; i++) {
    SPI.transfer(packet[i]);
  }
  
  digitalWrite(FPGA_SPI_CS, HIGH);  // Deselect FPGA (CS high) - transaction complete
  SPI.endTransaction();
    
    last_packet_send = current_time;
  }
}
