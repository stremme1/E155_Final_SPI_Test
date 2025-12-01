// bno085_driver.h
// BNO085 Sensor Driver for STM32L432KC
// Handles SHTP (Sensor Hub Transport Protocol) communication over SPI
// Ported from fpga/bno085_controller.sv

#ifndef BNO085_DRIVER_H
#define BNO085_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

// SHTP Protocol constants (per datasheet Section 1.3.1)
#define CHANNEL_COMMAND        0x00      // SHTP command channel
#define CHANNEL_EXECUTABLE     0x01      // Executable channel
#define CHANNEL_CONTROL        0x02      // Sensor hub control channel
#define CHANNEL_REPORTS        0x03      // Input sensor reports (non-wake, not gyroRV)
#define CHANNEL_WAKE_REPORTS   0x04      // Wake input sensor reports
#define CHANNEL_GYRO_RV        0x05      // Gyro rotation vector

// Report IDs (per datasheet Section 1.3.2, Figure 1-34)
#define REPORT_ID_ROTATION_VECTOR  0x05
#define REPORT_ID_GYROSCOPE        0x02  // Calibrated gyroscope per Fig 1-34

// BNO085 pin definitions (user must configure these in main.c)
// These are GPIO pins for control signals
extern int BNO085_CS_PIN;      // Chip Select (active low) - GPIO output
extern int BNO085_PS0_WAKE_PIN; // PS0/WAKE (active low) - GPIO output
extern int BNO085_INT_PIN;     // INT (active low) - GPIO input
extern int BNO085_RST_PIN;      // Reset (active low, optional) - GPIO output

// Sensor data structure
typedef struct {
    bool quat_valid;
    int16_t quat_w;
    int16_t quat_x;
    int16_t quat_y;
    int16_t quat_z;
    
    bool gyro_valid;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} bno085_data_t;

// Driver state
typedef struct {
    bool initialized;
    bool error;
} bno085_status_t;

// Initialize BNO085 sensor
// Returns true on success, false on error
bool bno085_init(void);

// Check if sensor is initialized
bool bno085_is_initialized(void);

// Check if sensor has error
bool bno085_has_error(void);

// Read sensor data (non-blocking, polls INT pin)
// Returns true if new data was read, false otherwise
bool bno085_read_data(bno085_data_t *data);

// Service function - call this periodically in main loop
// Handles initialization state machine and data reading
void bno085_service(void);

#endif // BNO085_DRIVER_H

