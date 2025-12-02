#include <Arduino.h>
// This demo explores two reports (SH2_ARVR_STABILIZED_RV and SH2_GYRO_INTEGRATED_RV) both can be used to give 
// quartenion and euler (yaw, pitch roll) angles.  Toggle the FAST_MODE define to see other report.  
// Note sensorValue.status gives calibration accuracy (which improves over time)
#include <Adafruit_BNO08x.h>
#include <SPI.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9

// #define FAST_MODE

// For SPI mode, we also need a RESET pin
// Connect RESET pin to Arduino pin 5 (or set to -1 if not using hardware reset)
#define BNO08X_RESET 5

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
}

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");
  Serial.println("Initializing BNO08x via SPI...");
  
  // Give sensor time to power up and stabilize
  delay(100);
  
  // Configure INT pin as input with pull-up (important for reliable operation)
  pinMode(BNO08X_INT, INPUT_PULLUP);
  
  // If using hardware reset, configure RESET pin
  if (BNO08X_RESET >= 0) {
    pinMode(BNO08X_RESET, OUTPUT);
    digitalWrite(BNO08X_RESET, LOW);
    delay(10);
    digitalWrite(BNO08X_RESET, HIGH);
    delay(50);  // Give sensor time to reset
  }
  
  // Try to initialize via SPI
  Serial.print("Attempting SPI initialization...");
  int retry_count = 0;
  bool initialized = false;
  
  while (retry_count < 5 && !initialized) {
    initialized = bno08x.begin_SPI(BNO08X_CS, BNO08X_INT);
    if (!initialized) {
      retry_count++;
      Serial.print(".");
      delay(100);
    }
  }
  
  if (!initialized) {
    Serial.println();
    Serial.println("Failed to find BNO08x chip after 5 attempts");
    Serial.println("Check connections:");
    Serial.print("  CS → Pin "); Serial.println(BNO08X_CS);
    Serial.print("  INT → Pin "); Serial.println(BNO08X_INT);
    if (BNO08X_RESET >= 0) {
      Serial.print("  RESET → Pin "); Serial.println(BNO08X_RESET);
    }
    Serial.println("  SCK, MOSI, MISO → Default SPI pins (13, 11, 12)");
    Serial.println("  3.3V and GND connected");
    while (1) { delay(10); }
  }
  
  Serial.println();
  Serial.println("BNO08x Found!");

  setReports(reportType, reportIntervalUs);
  Serial.println("Reading events");
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
        break;
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
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
