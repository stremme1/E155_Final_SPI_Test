// arduino_sensor_bridge.ino
// Arduino Sensor Bridge: Reads BNO085 via SPI and sends data to STM32 via SPI
//
// Hardware Connections:
// BNO085 - ESP32 (SPI):
//   Sensor CS → ESP32 Pin 10
//   Sensor INT → ESP32 Pin 9
//   Sensor RESET → ESP32 Pin 5
//   Sensor SCK → ESP32 SCK D13
//   Sensor MOSI → ESP32 MOSI D11
//   Sensor MISO → ESP32 MISO D12
//   Sensor 3.3V → ESP32 3.3V
//   Sensor GND → ESP32 GND
//
// ESP32 → STM32 (SPI):
//   ESP32 D2 (SCK) → STM32 PB3 (SPI_SCK)
//   ESP32 D4 (MOSI) → STM32 PB5 (SPI_MOSI)
//   ESP32 D7 (MISO) → STM32 PB4 (SPI_MISO)
//   ESP32 D3 (CS) → STM32 PA11 (SPI_NSS)
//   ESP32 GND → STM32 GND (common ground)
// NOTE: Using free pins (D2, D3, D4, D7) to avoid any conflicts with BNO08x SPI

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <SPI.h>

// BNO08x SPI pins (from starter code)
#define BNO08X_CS 10
#define BNO08X_INT 9
#define BNO08X_RESET 5

// MCU SPI pins (ESP32 to STM32 communication)
// Using free pins to avoid any conflicts with BNO08x SPI
// On ESP32: D2=GPIO2, D3=GPIO0, D4=GPIO4, D7=GPIO7
// Note: D3 (GPIO0) is often used for boot mode, so using D2, D4, D7, and another free pin
#define MCU_SPI_SCK 2   // D2 - SCK for MCU SPI
#define MCU_SPI_MOSI 4  // D4 - MOSI for MCU SPI  
#define MCU_SPI_MISO 7  // D7 - MISO for MCU SPI
#define MCU_SPI_CS 3     // D3 - CS for MCU SPI (or use another free pin if D3 causes issues)

// Simple software SPI implementation for MCU communication
// This avoids conflicts with BNO08x hardware SPI
// BNO08x uses hardware SPI (default pins D13/D11/D12)
// MCU uses software SPI (custom pins D2/D4/D7/D3)
// Mode 0: CPOL=0, CPHA=0 (clock idle LOW, sample on rising edge)

void mcuSPI_beginTransaction(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {
  // Software SPI - pins already configured in setup
  // For Mode 0: CPOL=0, so clock starts LOW
  digitalWrite(MCU_SPI_SCK, LOW);
}

uint8_t mcuSPI_transfer(uint8_t data) {
  uint8_t received = 0;
  
  // Mode 0 (CPOL=0, CPHA=0): Clock idle LOW, sample on rising edge
  for (int i = 7; i >= 0; i--) {
    // Set MOSI before clock edge
    digitalWrite(MCU_SPI_MOSI, (data >> i) & 0x01);
    delayMicroseconds(1);  // Setup time
    
    // Rising edge - sample MISO here
    digitalWrite(MCU_SPI_SCK, HIGH);
    delayMicroseconds(1);
    received |= (digitalRead(MCU_SPI_MISO) << i);
    
    // Falling edge
    digitalWrite(MCU_SPI_SCK, LOW);
    delayMicroseconds(1);
  }
  
  return received;
}

void mcuSPI_endTransaction(void) {
  // Ensure clock is idle (LOW for Mode 0)
  digitalWrite(MCU_SPI_SCK, LOW);
}

// SPI configuration for MCU communication
#define MCU_SPI_CLOCK_SPEED 625000  // 625kHz to match MCU's expected speed
#define MCU_SPI_MODE SPI_MODE0      // CPOL=0, CPHA=0

// Sensor report configuration
#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Euler angles structure (from quaternion conversion)
struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

// Latest sensor data for MCU transmission
struct sensor_data_t {
  float quat_w, quat_x, quat_y, quat_z;  // Quaternion from BNO08x
  float gyro_x, gyro_y, gyro_z;          // Gyroscope from BNO08x (rad/s)
  bool quat_valid;
  bool gyro_valid;
} latest_sensor_data;

void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
  // Also enable gyroscope report for MCU communication
  if (! bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, report_interval)) {
    Serial.println("Could not enable gyroscope report");
  }
}

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  Serial.println("Adafruit BNO08x test!");
  
  // Try to initialize via SPI! (exactly like starter code)
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  // Initialize sensor data structure
  latest_sensor_data.quat_valid = false;
  latest_sensor_data.gyro_valid = false;

  setReports(reportType, reportIntervalUs);
  Serial.println("Reading events");
  
  // MCU SPI initialization - COMMENTED OUT FOR TESTING
  // Uncomment after BNO08x is working
  /*
  Serial.println("Initializing MCU SPI communication (software SPI)...");
  pinMode(MCU_SPI_CS, OUTPUT);
  digitalWrite(MCU_SPI_CS, HIGH);  // CS high (idle, MCU not selected)
  
  // Configure MCU SPI pins for software SPI
  pinMode(MCU_SPI_SCK, OUTPUT);
  pinMode(MCU_SPI_MOSI, OUTPUT);
  pinMode(MCU_SPI_MISO, INPUT);
  
  // Set initial clock state (Mode 0: CPOL=0, so clock starts LOW)
  digitalWrite(MCU_SPI_SCK, LOW);
  
  Serial.print("MCU SPI (software) pins: SCK=D");
  Serial.print(MCU_SPI_SCK); Serial.print(", MOSI=D"); Serial.print(MCU_SPI_MOSI);
  Serial.print(", MISO=D"); Serial.print(MCU_SPI_MISO); Serial.print(", CS=D");
  Serial.println(MCU_SPI_CS);
  */
  
  delay(100);
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

// Convert quaternion float (-1.0 to 1.0) to int16_t format for MCU
// Scale by 16384 to use full int16_t range
int16_t quaternionToInt16(float q) {
  float scaled = q * 16384.0f;
  if (scaled > 32767.0f) scaled = 32767.0f;
  if (scaled < -32768.0f) scaled = -32768.0f;
  return (int16_t)scaled;
}

// Convert gyroscope rad/s to int16_t format for MCU
// Scale by 2000 to match expected range (similar to other Arduino code)
int16_t gyroToInt16(float gyro_rad_per_sec) {
  float scaled = gyro_rad_per_sec * 2000.0f;
  if (scaled > 32767.0f) scaled = 32767.0f;
  if (scaled < -32768.0f) scaled = -32768.0f;
  return (int16_t)scaled;
}

// Send 16-byte sensor data packet to MCU via SPI
// Packet format: [Header(0xAA)][Quat W MSB][Quat W LSB][Quat X MSB][Quat X LSB]...
//                [Quat Y MSB][Quat Y LSB][Quat Z MSB][Quat Z LSB]
//                [Gyro X MSB][Gyro X LSB][Gyro Y MSB][Gyro Y LSB][Gyro Z MSB][Gyro Z LSB]
//                [Flags]
void sendSensorPacketToMCU(void) {
  uint8_t packet[16];
  int16_t quat_w, quat_x, quat_y, quat_z;
  int16_t gyro_x, gyro_y, gyro_z;
  uint8_t flags;
  
  // Only send if we have valid data
  if (!latest_sensor_data.quat_valid) {
    return;
  }
  
  // Build packet header
  packet[0] = 0xAA;  // Header byte
  
  // Convert quaternion to int16_t and pack (MSB first, LSB second)
  quat_w = quaternionToInt16(latest_sensor_data.quat_w);
  quat_x = quaternionToInt16(latest_sensor_data.quat_x);
  quat_y = quaternionToInt16(latest_sensor_data.quat_y);
  quat_z = quaternionToInt16(latest_sensor_data.quat_z);
  
  packet[1] = (uint8_t)((quat_w >> 8) & 0xFF);  // MSB
  packet[2] = (uint8_t)(quat_w & 0xFF);          // LSB
  packet[3] = (uint8_t)((quat_x >> 8) & 0xFF);   // MSB
  packet[4] = (uint8_t)(quat_x & 0xFF);          // LSB
  packet[5] = (uint8_t)((quat_y >> 8) & 0xFF);   // MSB
  packet[6] = (uint8_t)(quat_y & 0xFF);          // LSB
  packet[7] = (uint8_t)((quat_z >> 8) & 0xFF);   // MSB
  packet[8] = (uint8_t)(quat_z & 0xFF);          // LSB
  
  // Convert gyroscope to int16_t and pack (MSB first, LSB second)
  if (latest_sensor_data.gyro_valid) {
    gyro_x = gyroToInt16(latest_sensor_data.gyro_x);
    gyro_y = gyroToInt16(latest_sensor_data.gyro_y);
    gyro_z = gyroToInt16(latest_sensor_data.gyro_z);
  } else {
    gyro_x = 0;
    gyro_y = 0;
    gyro_z = 0;
  }
  
  packet[9] = (uint8_t)((gyro_x >> 8) & 0xFF);    // MSB
  packet[10] = (uint8_t)(gyro_x & 0xFF);         // LSB
  packet[11] = (uint8_t)((gyro_y >> 8) & 0xFF);  // MSB
  packet[12] = (uint8_t)(gyro_y & 0xFF);        // LSB
  packet[13] = (uint8_t)((gyro_z >> 8) & 0xFF); // MSB
  packet[14] = (uint8_t)(gyro_z & 0xFF);        // LSB
  
  // Flags byte: bit 0 = quat_valid, bit 1 = gyro_valid
  flags = 0;
  if (latest_sensor_data.quat_valid) flags |= 0x01;
  if (latest_sensor_data.gyro_valid) flags |= 0x02;
  packet[15] = flags;
  
  // Send packet via SPI to MCU using software SPI
  // CS protocol: CS LOW → send all 16 bytes → CS HIGH
  // CS must stay LOW for entire transaction (critical per MCU code)
  mcuSPI_beginTransaction(MCU_SPI_CLOCK_SPEED, MSBFIRST, MCU_SPI_MODE);
  digitalWrite(MCU_SPI_CS, LOW);  // CS low
  
  // Small setup delay to allow MCU to prepare (similar to MCU's 100-cycle delay)
  delayMicroseconds(2);
  
  // Send all 16 bytes while CS stays low
  for (int i = 0; i < 16; i++) {
    mcuSPI_transfer(packet[i]);
  }
  
  // Small hold delay after transmission
  delayMicroseconds(2);
  
  // Pull CS high to end transaction
  digitalWrite(MCU_SPI_CS, HIGH);  // CS high
  mcuSPI_endTransaction();
  
  // Debug output (can be disabled for performance)
  static unsigned long last_debug = 0;
  if (millis() - last_debug > 1000) {  // Print once per second
    Serial.print("[MCU] Sent packet: Quat=[");
    Serial.print(quat_w); Serial.print(",");
    Serial.print(quat_x); Serial.print(",");
    Serial.print(quat_y); Serial.print(",");
    Serial.print(quat_z); Serial.print("] Gyro=[");
    Serial.print(gyro_x); Serial.print(",");
    Serial.print(gyro_y); Serial.print(",");
    Serial.print(gyro_z); Serial.print("] Flags=0x");
    Serial.println(flags, HEX);
    last_debug = millis();
  }
}

void loop() {
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        // Store quaternion for MCU transmission (when MCU SPI is enabled)
        latest_sensor_data.quat_w = sensorValue.un.arvrStabilizedRV.real;
        latest_sensor_data.quat_x = sensorValue.un.arvrStabilizedRV.i;
        latest_sensor_data.quat_y = sensorValue.un.arvrStabilizedRV.j;
        latest_sensor_data.quat_z = sensorValue.un.arvrStabilizedRV.k;
        latest_sensor_data.quat_valid = true;
        break;
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        // Store quaternion for MCU transmission (when MCU SPI is enabled)
        latest_sensor_data.quat_w = sensorValue.un.gyroIntegratedRV.real;
        latest_sensor_data.quat_x = sensorValue.un.gyroIntegratedRV.i;
        latest_sensor_data.quat_y = sensorValue.un.gyroIntegratedRV.j;
        latest_sensor_data.quat_z = sensorValue.un.gyroIntegratedRV.k;
        latest_sensor_data.quat_valid = true;
        break;
    }
    
    // Store gyroscope data if available (when MCU SPI is enabled)
    if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
      latest_sensor_data.gyro_x = sensorValue.un.gyroscope.x;
      latest_sensor_data.gyro_y = sensorValue.un.gyroscope.y;
      latest_sensor_data.gyro_z = sensorValue.un.gyroscope.z;
      latest_sensor_data.gyro_valid = true;
    }
    
    // Send data to MCU - COMMENTED OUT FOR TESTING
    // Uncomment after BNO08x is working
    /*
    if (latest_sensor_data.quat_valid) {
      sendSensorPacketToMCU();
    }
    */
    
    // Debug output (original serial output from starter code)
    static long last = 0;
    long now = micros();
    Serial.print(now - last);             Serial.print("\t");
    last = now;
    Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
    Serial.print(ypr.yaw);                Serial.print("\t");
    Serial.print(ypr.pitch);              Serial.print("\t");
    Serial.println(ypr.roll);
  }
}


