# Critical Review of BNO085 Controller FPGA Code
## Based on BNO08X Datasheet Revision 1.17

### Executive Summary
This review identifies several critical issues and areas for improvement in the FPGA controller implementation when compared to the BNO08X datasheet specifications.

---

## 1. CRITICAL ISSUES

### 1.1 Reset Timing Violation
**Datasheet Section 6.5.3:**
- `tnrst` (reset pulse width): **10ns minimum**
- `t1` (internal initialization): **90ms**
- `t2` (internal configuration): **4ms**
- **Total: 94ms minimum after reset release**

**Current Implementation:**
- Holds reset low for 1ms (OK - exceeds 10ns minimum)
- Releases reset after 100ms total (OK - exceeds 94ms requirement)
- **ISSUE**: Controller waits additional 10ms in `INIT_WAIT_RESET` state after reset is already released
- **Total delay: 110ms** - This is acceptable but inefficient

**Recommendation:** Remove the 10ms delay in `INIT_WAIT_RESET` since reset was already released 100ms earlier. The sensor should be ready immediately.

---

### 1.2 SPI CS Setup Time (tcssu)
**Datasheet Section 6.5.2:**
- `tcssu` (CS setup to CLK): **0.1 µs minimum**

**Current Implementation:**
- `READ_HEADER_START` asserts CS and immediately transitions to `READ_HEADER`
- Comment says "Wait one cycle for CS to settle" but this is just a state transition
- At 3MHz, one cycle = 333ns, which exceeds 0.1µs requirement ✓
- **However**: No explicit delay is implemented - relies on state machine timing

**Recommendation:** Add explicit 1-cycle delay after CS assertion to ensure tcssu is met. Current implementation may work but is not guaranteed.

---

### 1.3 INT Deassertion Timing (tcsid)
**Datasheet Section 6.5.4:**
- `tcsid` (H_CSN to H_INTN de-assert): **800ns maximum**
- Datasheet states: "The BNO08X will de-assert the interrupt line as soon as the chip select is detected"

**Current Implementation:**
- Code waits for INT before asserting CS (correct per Adafruit library)
- Once CS is asserted, INT should deassert within 800ns
- **ISSUE**: Code doesn't account for this 800ns delay. After CS goes low, INT may still be asserted for up to 800ns
- This could cause issues if code checks INT immediately after CS assertion

**Recommendation:** After asserting CS, don't check INT state for at least 1µs (3 cycles @ 3MHz) to account for tcsid.

---

### 1.4 SPI Wake Operation Timing
**Datasheet Section 1.2.4.3 & 6.5.4:**
- `twk` (wake signal assert to INT assert): **150 µs maximum**
- PS0/WAKE is active low

**Current Implementation:**
- Holds PS0 low for 150µs (450 cycles @ 3MHz) ✓
- Then waits for INT with 200ms timeout
- **ISSUE**: Datasheet says INT should assert within 150µs, but code waits up to 200ms
- This is overly conservative and may mask real problems

**Recommendation:** Reduce timeout to 1ms (3000 cycles) after wake signal. If INT doesn't assert within 1ms, something is wrong.

---

### 1.5 Missing CS Hold Time (tcssh)
**Datasheet Section 6.5.2:**
- `tcssh` (CS hold after last clock): **16.83 Ns minimum** (Note: This seems like a typo in datasheet, likely means 16.83ns or possibly µs)

**Current Implementation:**
- CS is released immediately after last byte transfer
- No explicit hold time implemented
- At 3MHz, state transitions provide some delay, but not guaranteed

**Recommendation:** Add explicit 1-cycle delay after last byte before releasing CS to ensure tcssh is met.

---

## 2. PROTOCOL ISSUES

### 2.1 SHTP Sequence Number Tracking
**Datasheet Section 1.3.1:**
- "The sequence number is a monotonically incrementing number that increments once for each cargo sent"
- "Each channel and each direction has its own sequence number"
- Used to detect duplicate or missing cargoes

**Current Implementation:**
- Code captures `shtp_seq_num` from received packets but **NEVER USES IT**
- No sequence number validation
- No duplicate/missing packet detection
- **CRITICAL**: This violates SHTP protocol requirements

**Recommendation:** 
- Track expected sequence numbers per channel
- Validate received sequence numbers
- Detect and handle missing/duplicate packets
- This is especially important for initialization commands

---

### 2.2 Advertisement Packet Processing
**Datasheet Section 5.2.1:**
- "After power up or reset the BNO08X will assert the interrupt (HOST_INTN) indicating that the reset routine has completed and that the BNO08X is ready for communication. A read from the BNO08X will return the initial SHTP advertisement packet."

**Current Implementation:**
- Code waits for advertisements but **DOES NOT PARSE THEM**
- Comment says "For simplicity, we just mark it as processed when we finish reading"
- **CRITICAL**: Advertisement packets contain critical information:
  - Channel assignments
  - Maximum transfer sizes
  - Application names
- Not parsing advertisements means code doesn't know actual channel numbers or packet size limits

**Recommendation:**
- Implement TLV (Tag-Length-Value) parser for advertisement packets
- Extract channel assignments (especially Channel 2 for control)
- Extract maximum transfer sizes
- Validate that channels match expected values

---

### 2.3 Command Response Validation
**Datasheet Section 1.3.2:**
- Product ID Response (0xF8) contains: Reset Cause, SW Version, Part Number, Build Number
- Get Feature Response (0xFC) contains: Feature Report ID, Flags, Report Interval, etc.

**Current Implementation:**
- Code checks for expected report IDs but doesn't validate response contents
- Accepts any response with correct ID, even if malformed
- **ISSUE**: No validation that response actually matches the command sent

**Recommendation:**
- Validate response packet length matches expected size
- For Product ID Response: Verify it's actually a Product ID response (check structure)
- For Get Feature Response: Verify Feature Report ID matches what was requested

---

### 2.4 Missing Executable Channel Reset Message
**Datasheet Section 5.2.1:**
- "Following the SHTP advertisement packet, the individual applications built in to the BNO08X will send a packet indicating they have left the reset state: The executable will issue a reset message on SHTP channel 1"

**Current Implementation:**
- Code doesn't handle Channel 1 (executable) reset messages
- May miss critical initialization status

**Recommendation:**
- Add handling for Channel 1 reset messages
- Verify sensor has completed reset before proceeding

---

## 3. TIMING AND STATE MACHINE ISSUES

### 3.1 SPI Clock Frequency
**Datasheet Section 6.5.2:**
- Maximum SPI clock frequency: **3 MHz**

**Current Implementation:**
- System clock: 3MHz
- SPI clock divider: CLK_DIV=2
- SPI clock frequency: 3MHz / (2*2) = 750 kHz ✓ (within spec)

**Status:** OK

---

### 3.2 SPI Mode
**Datasheet Section 1.2.4.2:**
- CPOL = 1, CPHA = 1 (SPI Mode 3)
- Clock idles high
- Data captured on rising edge

**Current Implementation:**
- `sclk_reg` starts high ✓
- Code comments indicate Mode 3 ✓
- **VERIFY**: SPI master implementation matches Mode 3 exactly

**Status:** Appears correct, but verify SPI master timing

---

### 3.3 State Machine Complexity
**Current Implementation:**
- 14 states in state machine
- Multiple flags (`waiting_for_advert`, `waiting_for_response`, `advert_done`, etc.)
- Complex state transitions

**Issues:**
- State machine may get stuck if flags aren't cleared properly
- No recovery mechanism from stuck states
- Error state is terminal (no recovery)

**Recommendation:**
- Add watchdog timer to detect stuck states
- Implement recovery mechanism (e.g., reset sensor and retry)
- Simplify state machine by reducing number of flags

---

## 4. DATA PARSING ISSUES

### 4.1 Little-Endian Byte Order
**Datasheet Section 1.2.2.2:**
- "For multi-byte words the data is presented in little-endian format – least-significant byte first"

**Current Implementation:**
- Code correctly handles little-endian for quaternion and gyroscope data ✓
- Uses `temp_byte_lsb` to store LSB before MSB ✓

**Status:** OK

---

### 4.2 Rotation Vector Report Format
**Datasheet Section 2.2.4:**
- Rotation Vector is a quaternion (4 components: W, X, Y, Z)
- Each component is 16-bit signed integer

**Current Implementation:**
- Code correctly extracts W, X, Y, Z components ✓
- Uses signed 16-bit format ✓

**Status:** OK

---

### 4.3 Missing Report Validation
**Current Implementation:**
- Code doesn't validate report lengths
- Doesn't check if report ID matches expected format
- Doesn't verify sequence numbers are sequential

**Recommendation:**
- Validate report length matches expected size for each report type
- Check report ID is valid before parsing
- Verify sequence numbers increment correctly

---

## 5. INITIALIZATION SEQUENCE ISSUES

### 5.1 Missing Delay Between Commands
**Datasheet Section 5.2.2:**
- No explicit requirement for delay between commands
- However, sensor needs time to process each command

**Current Implementation:**
- 10ms delay between commands (30,000 cycles @ 3MHz)
- This is reasonable but not based on datasheet

**Status:** Acceptable, but verify with testing

---

### 5.2 Command Sequence
**Current Implementation:**
1. Product ID Request (0xF9)
2. Set Feature - Rotation Vector (0xFD, Report ID 0x05)
3. Set Feature - Gyroscope (0xFD, Report ID 0x02)

**Datasheet:**
- Doesn't specify required command sequence
- But per Section 5.2.1, should wait for advertisements first ✓
- Should wait for reset message on Channel 1 (MISSING)

**Recommendation:**
- Add handling for Channel 1 reset message
- Verify sensor is ready before sending commands

---

## 6. ERROR HANDLING

### 6.1 Error State Recovery
**Current Implementation:**
- Error state is terminal - no recovery
- Comment says "Could add recovery logic here"

**Recommendation:**
- Implement recovery: reset sensor and retry initialization
- Add retry counter (max 3 attempts)
- If all retries fail, set error flag permanently

---

### 6.2 Timeout Values
**Current Implementation:**
- INT wait timeout: 200ms
- Advertisement timeout: 200ms
- Response timeout: 100ms

**Issues:**
- Timeouts are very long (may mask real problems)
- No differentiation between critical and non-critical timeouts

**Recommendation:**
- Reduce timeouts to more reasonable values:
  - INT wait after wake: 1ms (datasheet says 150µs max)
  - Advertisement wait: 50ms (sensor should send immediately after reset)
  - Response wait: 20ms (commands should respond quickly)

---

## 7. MISSING FEATURES

### 7.1 No Support for Fragmented Packets
**Datasheet Section 1.3.1:**
- "The BNO08X does not support receiving fragmented messages but it does support sending them"
- Continuation bit (bit 15 of length field) indicates fragmented packets

**Current Implementation:**
- Code clears continuation bit but doesn't handle fragmented packets
- If sensor sends fragmented packet, code will fail

**Recommendation:**
- Add support for reading fragmented packets
- Track continuation across multiple reads
- Reassemble fragmented payloads

---

### 7.2 No Support for Multiple Advertisement Packets
**Datasheet Section 5.2.1:**
- Sensor may send multiple advertisement packets

**Current Implementation:**
- Code attempts to handle multiple advertisements
- Returns to `INIT_WAIT_ADVERT` after reading each one ✓
- But doesn't parse them, so can't tell when all are received

**Status:** Partially implemented

---

## 8. RECOMMENDATIONS SUMMARY

### Critical (Must Fix):
1. **Add SHTP sequence number validation** - Protocol violation
2. **Parse advertisement packets** - Need channel assignments and packet sizes
3. **Add CS hold time** - May violate timing spec
4. **Handle Channel 1 reset messages** - Missing initialization step
5. **Reduce timeout values** - Current values may mask problems

### Important (Should Fix):
1. Remove unnecessary 10ms delay in INIT_WAIT_RESET
2. Add explicit CS setup delay
3. Account for INT deassertion delay (tcsid)
4. Implement error recovery mechanism
5. Add support for fragmented packets

### Nice to Have:
1. Simplify state machine
2. Add report validation
3. Better error messages/debugging
4. Add watchdog timer

---

## 9. TESTING RECOMMENDATIONS

1. **Timing Verification:**
   - Verify all SPI timing parameters with oscilloscope
   - Check CS setup/hold times
   - Verify INT timing

2. **Protocol Compliance:**
   - Verify sequence numbers increment correctly
   - Test with fragmented packets
   - Test with multiple advertisement packets

3. **Error Cases:**
   - Test timeout scenarios
   - Test with sensor disconnected
   - Test with corrupted packets
   - Test recovery from error state

4. **Initialization:**
   - Verify initialization completes successfully
   - Check that all commands are sent correctly
   - Verify responses are received and validated

---

## Conclusion

The current implementation has several critical issues that violate the BNO08X datasheet specifications, particularly around SHTP protocol compliance (sequence numbers, advertisement parsing) and timing (CS hold time, INT deassertion delay). While the code may work in some cases, it is not robust and may fail under certain conditions. The recommendations above should be implemented to ensure reliable operation.

