# BNO08X SPI Controller Audit Report
## Based on BNO08X Datasheet Revision 1.17

### Executive Summary
This audit reviews the `bno085_controller.sv` and `spi_master.sv` modules against the BNO08X datasheet specifications. Several issues and recommendations are identified.

---

## 1. SPI Mode and Timing (Section 1.2.4.2, 6.5.2)

### ✅ CORRECT: SPI Mode 3 Implementation
- **Datasheet**: CPOL=1, CPHA=1 (clock idles high, data captured on rising edge)
- **Code**: `spi_master.sv` correctly implements Mode 3
  - Line 46: `sclk_reg <= 1'b1;  // CPOL=1: idle high`
  - Lines 107-110: Samples MISO on rising edge (CPHA=1)

### ✅ CORRECT: MSB First
- **Datasheet**: "SPI transmits data MSB first"
- **Code**: Line 132: `assign mosi = tx_shift[7];` (MSB first)

### ⚠️ POTENTIAL ISSUE: SPI Clock Frequency
- **Datasheet Section 6.5.2**: Maximum SPI clock frequency = 3 MHz
- **Code**: `spi_test_top.sv` line 41 uses HSOSC with CLKHF_DIV(2'b11) = divide by 16
  - If HSOSC is 48MHz: 48MHz/16 = 3MHz ✅ CORRECT
  - **Recommendation**: Verify actual HSOSC frequency matches assumption

### ⚠️ TIMING PARAMETERS: Missing Verification
- **Datasheet Section 6.5.2** specifies:
  - `tcssu` (CS setup to CLK) = 0.1 µs min
  - `tcssh` (CS hold) = 16.83 ns
  - `tcsso` (CS to MISO out) = 31 ns
  - `tsov` (CLK to MISO valid) = 35 ns
  - `tsisu` (MOSI setup) = 25 ns
  - `tsih` (MOSI hold) = 5.4 ns
- **Code**: No explicit timing verification
- **Recommendation**: Add timing constraints or verify with timing analysis

---

## 2. Wake Operation (Section 1.2.4.3)

### ✅ CORRECT: Wake Signal Assertion
- **Datasheet**: PS0/WAKE pin is active low, drive low to wake processor
- **Code**: `bno085_controller.sv` line 224: `ps0_wake <= 1'b0;` ✅

### ✅ CORRECT: Wake Response
- **Datasheet**: BNO08X responds by asserting H_INTN (interrupt line)
- **Code**: Line 227-233: Waits for `int_n_sync` to go low ✅

### ⚠️ TIMING: Wake Response Time
- **Datasheet Section 6.5.4**: `twk` (BNO08X wakeup from wake signal assert) = 150 µs max
- **Code**: No timeout mechanism - could wait indefinitely if sensor fails
- **Recommendation**: Add timeout (e.g., 200 µs) in `INIT_WAKE` state

### ✅ CORRECT: CS Detection After Wake
- **Datasheet**: H_INTN deasserts when H_CSN is detected (tcsid = 800 ns max)
- **Code**: Line 239: Asserts `cs_n` after INT goes low ✅

---

## 3. H_INTN (Interrupt) Handling (Section 1.2.4.1, 6.5.4)

### ✅ CORRECT: Interrupt Synchronization
- **Code**: Lines 162-170: Properly synchronizes `int_n` signal with double flip-flop

### ⚠️ CRITICAL: Response Time Requirement
- **Datasheet Section 1.2.4.1**: "If the host fails to respond to the assertion of H_INTN within approximately 10 ms, the BNO085/BNO086 will timeout, deassert H_INTN and retry the operation."
- **Code**: No explicit response time guarantee
- **Recommendation**: Ensure state machine responds to INT within 1ms (1/10 of fastest sensor period per datasheet)

### ✅ CORRECT: CS Detection Deasserts INT
- **Datasheet Section 6.5.4**: H_INTN deasserts when H_CSN is detected (tcsid = 800 ns)
- **Code**: Line 310: Asserts `cs_n` when INT is low, which should deassert INT ✅

---

## 4. Startup Timing (Section 6.5.3)

### ✅ CORRECT: Reset Wait Period
- **Datasheet**: t1 (Internal Initialization) = 90 ms typical
- **Code**: Line 213: Waits 300,000 cycles
  - At 3MHz: 300,000 / 3,000,000 = 0.1s = 100ms ✅ Adequate

### ⚠️ MISSING: Post-Reset INT Wait
- **Datasheet Section 5.2.1**: "After power up or reset the BNO08X will assert the interrupt (HOST_INTN) indicating that the reset routine has completed"
- **Code**: `INIT_WAIT_RESET` doesn't check for INT assertion
- **Recommendation**: After delay, wait for INT assertion before proceeding

---

## 5. SHTP Protocol (Section 1.3.1)

### ✅ CORRECT: SHTP Header Structure
- **Datasheet**: 4-byte header: Length LSB, Length MSB, Channel, SeqNum
- **Code**: Lines 323-340: Correctly reads all 4 header bytes ✅

### ✅ CORRECT: Channel Definitions
- **Datasheet Section 1.3.1**:
  - Channel 0: SHTP command channel
  - Channel 1: Executable
  - Channel 2: Sensor hub control channel
  - Channel 3: Input sensor reports (non-wake, not gyroRV)
  - Channel 4: Wake input sensor reports
  - Channel 5: Gyro rotation vector
- **Code**: Lines 44-49: Correctly defines all channels ✅

### ✅ CORRECT: Channel Filtering
- **Code**: Line 359: Only processes Channel 3 (REPORTS) and Channel 5 (GYRO_RV) ✅

### ⚠️ ISSUE: Length Field Continuation Bit
- **Datasheet Section 1.3.1**: "Bit 15 of the length field is used to indicate if a transfer is a continuation of a previous transfer"
- **Code**: No handling of continuation bit
- **Impact**: May fail on fragmented messages (though datasheet says BNO08X doesn't send fragmented messages)
- **Recommendation**: Mask bit 15 when reading length: `packet_length[14:0]`

### ⚠️ ISSUE: Maximum Packet Length
- **Datasheet**: Maximum cargo is 32766 bytes minus header
- **Code**: Line 355: Limits to 64 bytes (`byte_cnt < 64`)
- **Impact**: May truncate large packets
- **Recommendation**: Increase buffer size or handle larger packets

---

## 6. Report IDs (Section 1.3.2, 1.3.5.2)

### ⚠️ CRITICAL ISSUE: Gyroscope Report ID
- **Datasheet Figure 1-34**: Calibrated gyroscope input report has Report ID = **0x02**
- **Code Line 53**: Uses `REPORT_ID_GYROSCOPE = 8'h01`
- **Impact**: Will not recognize calibrated gyroscope reports
- **Recommendation**: Change to `8'h02` OR verify which gyroscope report type is being used

### ✅ CORRECT: Rotation Vector Report ID
- **Code**: Uses `0x05` for Rotation Vector
- **Note**: Datasheet doesn't explicitly list this, but it's a common value in SH-2 protocol

### ✅ CORRECT: Product ID Request
- **Datasheet Figure 1-28**: Product ID Request = 0xF9
- **Code Line 104**: Correctly uses `8'hF9` ✅

---

## 7. Set Feature Command (Section 1.3.5, Figure 1-33)

### ✅ CORRECT: Set Feature Report ID
- **Datasheet Figure 1-33**: Set Feature = 0xFD
- **Code Lines 116, 136**: Correctly uses `8'hFD` ✅

### ⚠️ ISSUE: Report Interval Format
- **Datasheet Figure 1-33**: Report Interval is 4 bytes (LSB to MSB), units are microseconds
- **Code Lines 121-124, 141-144**: Sets `0x00000032` = 50 decimal = 50 microseconds
- **Issue**: 50 microseconds = 20,000 Hz - this is extremely high!
- **Datasheet Section 6.9**: Maximum rates are:
  - Rotation Vector: 400 Hz
  - Gyroscope: 400 Hz
- **Recommendation**: Use 0x000061A8 (25,000 µs = 40 Hz) or 0x00004E20 (20,000 µs = 50 Hz)

### ✅ CORRECT: Command Structure
- **Datasheet**: 17 bytes total (4 header + 13 payload)
- **Code**: Correctly sends 17 bytes ✅

### ✅ CORRECT: Feature Flags
- **Code**: Sets flags to 0x00 (default) ✅

### ✅ CORRECT: Change Sensitivity
- **Code**: Sets to 0x0000 (disabled) ✅

---

## 8. Data Parsing (Section 1.3.5.2, Figure 1-34)

### ⚠️ ISSUE: Rotation Vector Data Format
- **Datasheet**: Rotation Vector is a quaternion with Q-point 14 (per SH-2 Reference Manual)
- **Code Lines 362-375**: Parses quaternion data
- **Issue**: Byte order may be incorrect - datasheet says "little-endian format – least-significant byte first"
- **Code**: Line 365: `quat_x <= {spi_rx_data, temp_byte_lsb};` - This assumes MSB comes first, but should be LSB first
- **Recommendation**: Verify byte order matches datasheet (little-endian)

### ⚠️ ISSUE: Gyroscope Data Format
- **Datasheet Figure 1-34**: Gyroscope report format:
  - Byte 4-5: X-axis LSB, MSB
  - Byte 6-7: Y-axis LSB, MSB
  - Byte 8-9: Z-axis LSB, MSB
- **Code Lines 377-387**: Parses correctly assuming LSB first ✅
- **But**: Report ID is wrong (see issue #6)

### ⚠️ MISSING: Status and Delay Fields
- **Datasheet Figure 1-34**: Report includes:
  - Byte 1: Sequence number
  - Byte 2: Status (accuracy bits 1:0)
  - Byte 3: Delay (lower 8 bits)
- **Code**: Doesn't parse or use these fields
- **Recommendation**: Extract status to indicate data quality

### ⚠️ MISSING: Sequence Number Validation
- **Datasheet**: Sequence number is monotonically increasing, used to detect dropped samples
- **Code**: Doesn't track or validate sequence numbers
- **Recommendation**: Add sequence number tracking to detect dropped packets

---

## 9. Chip Select (CS) Timing

### ⚠️ ISSUE: CS Assertion Timing
- **Datasheet Section 6.5.2**: `tcssu` (CS setup to CLK) = 0.1 µs min
- **Code**: CS is asserted in same cycle as SPI start
- **Recommendation**: Assert CS at least 1 clock cycle before starting SPI transaction

### ✅ CORRECT: CS Deassertion
- **Code**: Properly deasserts CS after transactions ✅

---

## 10. Protocol Selection (Section 1.2.4)

### ✅ CORRECT: PS0/PS1 Configuration
- **Datasheet**: For SPI mode, PS1=1, PS0=1 (both high)
- **Code**: `ps0_wake` is controlled dynamically (used as WAKE signal after reset)
- **Note**: PS1 should be tied high in hardware (not shown in code, but should be in constraints)

---

## 11. Error Handling

### ⚠️ MISSING: Comprehensive Error Handling
- **Code**: Has `ERROR_STATE` but never transitions to it
- **Recommendations**:
  - Timeout in wake operation
  - Invalid packet length detection
  - Invalid channel detection
  - Sequence number mismatch detection

---

## Summary of Critical Issues

1. **CRITICAL**: Gyroscope Report ID is 0x01 but should be 0x02 (or verify correct report type)
2. **CRITICAL**: Report interval is 50 µs (20,000 Hz) - exceeds maximum of 400 Hz
3. **HIGH**: Missing timeout in wake operation
4. **HIGH**: Length field continuation bit not masked
5. **MEDIUM**: Byte order for quaternion may be incorrect (verify little-endian)
6. **MEDIUM**: Missing status/delay field parsing
7. **MEDIUM**: No sequence number validation
8. **LOW**: CS setup timing not guaranteed
9. **LOW**: Limited error handling

---

## Recommendations Priority

### Immediate Fixes:
1. Change gyroscope report ID to 0x02 OR verify which report type is configured
2. Fix report interval to reasonable value (e.g., 0x00004E20 = 20,000 µs = 50 Hz)
3. Add timeout to wake operation (200 µs)
4. Mask continuation bit in length field: `packet_length[14:0]`

### Important Improvements:
5. Verify byte order for quaternion data (little-endian)
6. Add status field parsing
7. Add sequence number tracking
8. Improve error handling with timeouts

### Nice to Have:
9. Add timing constraints
10. Increase packet buffer size
11. Add comprehensive error states

