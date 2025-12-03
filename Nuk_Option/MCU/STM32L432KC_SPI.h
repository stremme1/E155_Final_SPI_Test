// STM32L432KC_SPI.h
// Header for SPI functions
// Modified for SPI Slave mode (Arduino as master)

#ifndef STM32L4_SPI_H
#define STM32L4_SPI_H

#include <stdint.h>
#include "STM32L4xx/Device/Include/stm32l432xx.h"

// SPI pin definitions
#define SPI_CE PA11  // NSS (chip select) pin
#define SPI_SCK PB3  // SPI clock
#define SPI_MOSI PB5 // Master Out Slave In (data from Arduino)
#define SPI_MISO PB4 // Master In Slave Out (not used in this application)

///////////////////////////////////////////////////////////////////////////////
// Function prototypes
///////////////////////////////////////////////////////////////////////////////

/* Enables the SPI peripheral as SLAVE and initializes polarity and phase.
 *    -- br: Not used in slave mode (slave follows master's clock)
 *    -- cpol: clock polarity (0: inactive state is logical 0, 1: inactive state is logical 1).
 *    -- cpha: clock phase (0: data captured on leading edge of clk and changed on next edge, 
 *          1: data changed on leading edge of clk and captured on next edge)
 * Refer to the datasheet for more low-level details. */ 
void initSPI(int br, int cpol, int cpha);

/* Receives a character (1 byte) over SPI in slave mode.
 *    -- return: the character received over SPI */
char spiReceive(void);

/* Read 16-byte sensor data packet from Arduino/ESP32 via SPI (SLAVE mode)
 * Packet format (Roll, Pitch, Yaw order):
 *   Byte 0:  Header (0xAA)
 *   Bytes 1-2:  Roll (int16_t, MSB first, scaled by 100)
 *   Bytes 3-4:  Pitch (int16_t, MSB first, scaled by 100)
 *   Bytes 5-6:  Yaw (int16_t, MSB first, scaled by 100)
 *   Bytes 7-8:  Gyro X (int16_t, MSB first, scaled by 2000)
 *   Bytes 9-10: Gyro Y (int16_t, MSB first, scaled by 2000)
 *   Bytes 11-12: Gyro Z (int16_t, MSB first, scaled by 2000)
 *   Byte 13: Flags (bit 0 = Euler valid, bit 1 = Gyro valid)
 *   Bytes 14-15: Reserved (0x00)
 * All 16-bit values are MSB-first (big-endian per value)
 */
void readSensorDataPacket(uint8_t *packet);

/* Parse Arduino packet format (24 bytes) into Euler angles and gyroscope data
 * Packet format: [Sync(0xAA)][SensorID][Timestamp(4)][Roll(4)][Pitch(4)][Yaw(4)][GyroX(2)][GyroY(2)][GyroZ(2)]
 * All float values are IEEE 754 single precision (4 bytes)
 * Gyroscope values are int16_t (2 bytes, little-endian)
 *    -- packet: pointer to received packet data
 *    -- roll, pitch, yaw: output Euler angles in degrees (float)
 *    -- gyro_x, gyro_y, gyro_z: output gyroscope data (int16_t)
 *    -- sensor_id: output sensor ID
 *    -- valid: output validity flag (1 if valid, 0 if invalid)
 */
void parseArduinoPacket(const uint8_t *packet,
                       float *roll, float *pitch, float *yaw,
                       int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z,
                       uint8_t *sensor_id, uint8_t *valid);

/* Receive a packet from Arduino via UART (USART1)
 * Packet format: [Header(0xAA)][Quat][Gyro][Flags] - 16 bytes total
 *    -- packet: buffer to store received data
 *    -- length: number of bytes to receive (should be 16)
 *    -- return: 1 if successful, 0 if timeout or error
 */
uint8_t receiveUARTPacket(uint8_t *packet, uint16_t length);

///////////////////////////////////////////////////////////////////////////////
// Software SPI (Bit-banged) Functions
///////////////////////////////////////////////////////////////////////////////

/* Initialize pins for software SPI (GPIO mode)
 * Configures CS, SCK, and MOSI as inputs with pull-up resistors
 */
void initSoftwareSPI(void);

/* Receive one byte via software SPI
 * Reads 8 bits on rising clock edges (MSB first)
 * Returns the received byte, or 0xFF on error/timeout
 */
uint8_t softwareSPIReceiveByte(void);

/* Read 16-byte packet via software SPI
 * Waits for CS to go LOW, reads 16 bytes, waits for CS to go HIGH
 * Packet format matches hardware SPI version
 */
void softwareSPIReadPacket(uint8_t *packet);

#endif
