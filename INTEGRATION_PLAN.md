# Integration Plan: FPGA-to-MCU SPI Drum Trigger System

## Executive Summary

**Goal**: Connect existing FPGA drum detection system to MCU for audio playback via SPI.

**Key Components**:
1. **FPGA** (already has drum trigger processor) → Sends drum codes (0-7) via SPI
2. **MCU** (Lab4 has DAC playback) → Receives codes via SPI, plays drum sounds

**Integration Points**:
- **SPI Protocol**: Use Lab07 SPI code (MCU as master, FPGA as slave)
- **Drum Codes**: FPGA outputs 0-7, MCU maps to 8 drum samples
- **Communication**: 1-byte commands, simple handshaking with LOAD/DONE

**Quick Start**:
1. Copy `STM32L432KC_SPI.*` from Lab07 to Lab4 project
2. Modify Lab4 `main.c` to poll SPI and handle drum commands
3. Create `drum_spi_slave.sv` in FPGA project
4. Connect existing `drum_trigger_processor` to SPI slave

---

## Overview
This document outlines the plan to integrate:
- **Lab07 SPI Communication** (MCU ↔ FPGA communication)
- **Lab4 Drum Sound Playback** (MCU audio output)
- **Existing FPGA Drum Trigger System** (from `PLAN_INTEGRATED_DRUM_TRIGGER.md`)

The system will have the **FPGA act as a peripheral** that sends drum trigger commands to the **MCU via SPI**, and the MCU will play the corresponding drum sounds using the DAC.

---

## System Architecture

```
┌─────────────────┐         SPI          ┌─────────────────┐
│                 │ ◄──────────────────► │                 │
│   FPGA          │   (FPGA as Slave)    │   MCU           │
│   (Peripheral)  │                       │   (Master)      │
│                 │                       │                 │
│  - Detects      │                       │  - Receives     │
│    drum hits    │                       │    commands     │
│  - Sends trigger│                       │  - Plays sounds │
│    commands     │                       │  - DAC output   │
└─────────────────┘                       └─────────────────┘
```

---

## SPI Pin Assignments (from Lab07)

### MCU Side (Master)
- **PA5**: LOAD (GPIO output) - Used to signal FPGA to process data
- **PA6**: DONE (GPIO input) - FPGA signals completion
- **PA11**: CE (GPIO output) - Chip select (artificial, for debugging)
- **PB3**: SCK (SPI1_SCK) - SPI clock
- **PB5**: MOSI (SPI1_MOSI) - Master Out, Slave In
- **PB4**: MISO (SPI1_MISO) - Master In, Slave Out

### FPGA Side (Slave)
- **sck**: SPI clock (from MCU)
- **sdi**: SPI data in (MOSI from MCU)
- **sdo**: SPI data out (MISO to MCU)
- **load**: Load signal (from MCU PA5)
- **done**: Done signal (to MCU PA6)

---

## Existing FPGA Drum Trigger System

Based on `PLAN_INTEGRATED_DRUM_TRIGGER.md`, the FPGA already has:
- **Drum Trigger Processor** (`drum_trigger_processor.sv`) that outputs:
  - `drum_trigger_valid`: Single-cycle pulse when drum is detected
  - `drum_code[3:0]`: Drum code (0-7) matching C code exactly:
    - `0` = Snare drum
    - `1` = Hi-hat (left hand only)
    - `2` = Kick drum
    - `3` = High tom
    - `4` = Mid tom
    - `5` = Crash cymbal
    - `6` = Ride cymbal
    - `7` = Floor tom

**Integration Point**: Connect `drum_trigger_processor` outputs to SPI slave module.

---

## SPI Protocol Adaptation

### Original Lab07 Protocol (AES Encryption)
1. MCU sends 16 bytes plaintext
2. MCU sends 16 bytes key
3. MCU sets LOAD low
4. MCU waits for DONE high
5. MCU reads 16 bytes ciphertext

### New Protocol (Drum Trigger Commands)

**Command Format:**
- **1 byte command**: Drum type identifier (matches FPGA drum_code)
  - `0x00` = Snare drum
  - `0x01` = Hi-Hat
  - `0x02` = Kick drum
  - `0x03` = High tom
  - `0x04` = Mid tom
  - `0x05` = Crash cymbal
  - `0x06` = Ride cymbal
  - `0x07` = Floor tom
  - `0xFF` = No command / Idle (or use 0x00 when no trigger)

**Simplified Protocol:**
1. FPGA sends 1 byte command when drum is detected
2. MCU receives command and plays corresponding sound
3. No LOAD/DONE handshake needed (simpler than AES)

**Alternative Protocol (with handshake):**
1. FPGA sends 1 byte command
2. MCU sets LOAD high (acknowledge)
3. MCU plays sound
4. MCU sets LOAD low (ready for next command)
5. FPGA waits for LOAD low before sending next command

---

## Implementation Steps

### Phase 1: MCU Setup - Integrate SPI and DAC

#### Step 1.1: Copy SPI Library Files
- Copy from `lab07_mcu/lib/`:
  - `STM32L432KC_SPI.h`
  - `STM32L432KC_SPI.c`
- Add to Lab4 project

#### Step 1.2: Modify Main.c
- Include SPI library
- Initialize SPI (same config as Lab07: `initSPI(1, 0, 0)`)
- Initialize DAC (from Lab4: `DAC_InitAudio(DAC_CHANNEL_1)`)
- Set up GPIO pins:
  - PA5 as output (LOAD)
  - PA6 as input (DONE)
  - PA11 as output (CE)

#### Step 1.3: Create Drum Command Handler
**Note**: Map FPGA drum codes (0-7) to Lab4 drum samples. Lab4 has 8 samples but they may not match exactly:
- Lab4 samples: kick, snare, hihat_closed, hihat_open, crash, ride, tom_high, tom_low
- FPGA codes: 0=snare, 1=hihat, 2=kick, 3=high_tom, 4=mid_tom, 5=crash, 6=ride, 7=floor_tom

```c
void handle_drum_command(uint8_t command) {
    switch(command) {
        case 0x00:  // Snare drum
            play_drum_sample(snare_sample_data, snare_sample_length, snare_sample_sample_rate);
            break;
        case 0x01:  // Hi-hat (use closed for now, can add logic for open/closed)
            play_drum_sample(hihat_closed_sample_data, hihat_closed_sample_length, hihat_closed_sample_sample_rate);
            break;
        case 0x02:  // Kick drum
            play_drum_sample(kick_sample_data, kick_sample_length, kick_sample_sample_rate);
            break;
        case 0x03:  // High tom
            play_drum_sample(tom_high_sample_data, tom_high_sample_length, tom_high_sample_sample_rate);
            break;
        case 0x04:  // Mid tom (use tom_high for now, or add mid_tom sample)
            play_drum_sample(tom_high_sample_data, tom_high_sample_length, tom_high_sample_sample_rate);
            break;
        case 0x05:  // Crash cymbal
            play_drum_sample(crash_sample_data, crash_sample_length, crash_sample_sample_rate);
            break;
        case 0x06:  // Ride cymbal
            play_drum_sample(ride_sample_data, ride_sample_length, ride_sample_sample_rate);
            break;
        case 0x07:  // Floor tom
            play_drum_sample(tom_low_sample_data, tom_low_sample_length, tom_low_sample_sample_rate);
            break;
        default:
            // Invalid command, ignore
            break;
    }
}
```
```

#### Step 1.4: Main Loop - Poll SPI for Commands
```c
int main(void) {
    // Initialize system
    configureFlash();
    configureClock();
    
    // Initialize SPI (from Lab07)
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN);
    initSPI(1, 0, 0);  // br=1, cpol=0, cpha=0
    
    // Initialize GPIO pins
    pinMode(PA5, GPIO_OUTPUT);  // LOAD
    pinMode(PA6, GPIO_INPUT);   // DONE
    pinMode(PA11, GPIO_OUTPUT); // CE
    digitalWrite(PA5, 0);       // LOAD low initially
    digitalWrite(PA11, 1);      // CE high initially
    
    // Initialize DAC (from Lab4)
    DAC_InitAudio(DAC_CHANNEL_1);
    
    // Main loop: poll for drum commands
    while(1) {
        // Check if FPGA has data (DONE signal high)
        if(digitalRead(PA6)) {
            // Read command byte from FPGA
            digitalWrite(PA11, 0);  // CE low
            uint8_t command = spiSendReceive(0x00);  // Send dummy, receive command
            digitalWrite(PA11, 1);  // CE high
            
            // Process command
            if(command != 0x00) {
                handle_drum_command(command);
            }
        }
        
        // Small delay to prevent excessive polling
        ms_delay(1);
    }
}
```

---

### Phase 2: FPGA Setup - Integrate SPI Slave with Existing Drum Trigger

#### Step 2.1: Connect Existing Drum Trigger to SPI
The FPGA already has `drum_trigger_processor.sv` that outputs:
- `drum_trigger_valid`: Pulse when drum detected
- `drum_code[3:0]`: Drum code (0-7)

**Integration**: Add SPI slave module that sends `drum_code` to MCU when `drum_trigger_valid` is asserted.

#### Step 2.2: Create FPGA SPI Slave Module
Based on `aes_starter.sv` from Lab07, create a simpler SPI slave:

**File: `drum_spi_slave.sv`**
```systemverilog
module drum_spi_slave(
    input  logic clk,        // FPGA system clock
    input  logic sck,        // SPI clock from MCU
    input  logic sdi,        // SPI data in (MOSI from MCU)
    output logic sdo,        // SPI data out (MISO to MCU)
    input  logic load,       // Load signal from MCU
    output logic done,       // Done signal to MCU
    
    // Drum detection interface
    input  logic [7:0] drum_command,  // Command from drum detection logic
    input  logic command_ready,        // Command ready signal
    output logic command_ack           // Command acknowledged
);

    logic [7:0] tx_buffer;
    logic [7:0] rx_buffer;
    logic [2:0] bit_counter;
    logic command_sent;
    
    // SPI receive (from MCU perspective: MCU sends, FPGA receives)
    always_ff @(posedge sck) begin
        if(load) begin
            rx_buffer <= {rx_buffer[6:0], sdi};
            bit_counter <= bit_counter + 1;
        end else begin
            bit_counter <= 0;
        end
    end
    
    // SPI transmit (to MCU perspective: FPGA sends, MCU receives)
    always_ff @(negedge sck) begin
        if(load && bit_counter < 8) begin
            sdo <= tx_buffer[7 - bit_counter];
        end
    end
    
    // Command handling
    always_ff @(posedge clk) begin
        if(command_ready && !command_sent) begin
            tx_buffer <= drum_command;
            done <= 1'b1;
            command_sent <= 1'b1;
        end
        
        if(!load) begin
            done <= 1'b0;
            command_sent <= 1'b0;
            command_ack <= 1'b1;
        end else begin
            command_ack <= 1'b0;
        end
    end
endmodule
```

#### Step 2.3: Integrate with Existing Top-Level Module
**Modify `spi_test_top.sv`** (or create new top-level) to connect:
- Existing `drum_trigger_processor` outputs
- New `drum_spi_slave` module
- BNO085 controller (already exists)

**File: `drum_trigger_top.sv`** (or modify existing `spi_test_top.sv`)
```systemverilog
module drum_trigger_top(
    input  logic        fpga_rst_n,
    
    // SPI Interface to MCU (NEW)
    input  logic        sck,        // From MCU PB3
    input  logic        sdi,        // From MCU PB5 (MOSI)
    output logic        sdo,        // To MCU PB4 (MISO)
    input  logic        load,       // From MCU PA5
    output logic        done,       // To MCU PA6
    
    // BNO085 SPI Interface (EXISTING)
    output logic        sclk1,
    output logic        mosi1,
    input  logic        miso1,
    output logic        cs_n1,
    output logic        ps0_1,
    output logic        bno085_rst_n1,
    input  logic        int1,
    
    // Button Inputs (EXISTING)
    input  logic        calibrate_btn_n,
    input  logic        kick_btn_n
);

    // Clock and reset
    logic clk, rst_n;
    HSOSC hf_osc(.CLKHFEN(1'b1), .CLKHFPU(1'b1), .CLKHF(clk));
    assign rst_n = fpga_rst_n;
    
    // Existing BNO085 signals
    logic quat_valid, gyro_valid;
    logic signed [15:0] quat_w, quat_x, quat_y, quat_z;
    logic signed [15:0] gyro_x, gyro_y, gyro_z;
    
    // Existing drum trigger processor
    logic drum_trigger_valid;
    logic [3:0] drum_code;
    logic drum_hand;
    
    // SPI slave signals
    logic [7:0] drum_command;
    logic command_ready;
    logic command_ack;
    
    // Existing modules
    spi_master spi_master_inst(...);
    bno085_controller bno085_ctrl_inst(...);
    button_debouncer calibrate_debouncer(...);
    button_debouncer kick_debouncer(...);
    drum_trigger_processor drum_processor(
        .clk(clk),
        .rst_n(rst_n),
        .quat1_valid(quat_valid),
        .quat1_w(quat_w), .quat1_x(quat_x), .quat1_y(quat_y), .quat1_z(quat_z),
        .gyro1_valid(gyro_valid),
        .gyro1_x(gyro_x), .gyro1_y(gyro_y), .gyro1_z(gyro_z),
        .calibrate_btn_pulse(calibrate_btn_pulse),
        .kick_btn_pulse(kick_btn_pulse),
        .drum_trigger_valid(drum_trigger_valid),
        .drum_code(drum_code),
        .drum_hand(drum_hand)
    );
    
    // NEW: SPI slave for MCU communication
    drum_spi_slave spi_slave(
        .clk(clk),
        .sck(sck),
        .sdi(sdi),
        .sdo(sdo),
        .load(load),
        .done(done),
        .drum_command(drum_command),
        .command_ready(command_ready),
        .command_ack(command_ack)
    );
    
    // Convert drum_trigger_valid pulse to command_ready
    // When drum is detected, latch the drum_code and assert command_ready
    always_ff @(posedge clk) begin
        if(!rst_n) begin
            command_ready <= 1'b0;
            drum_command <= 8'h00;
        end else begin
            if(drum_trigger_valid) begin
                // Latch drum code when trigger detected
                drum_command <= {4'h0, drum_code};
                command_ready <= 1'b1;
            end else if(command_ack) begin
                // Reset when MCU acknowledges
                command_ready <= 1'b0;
            end
        end
    end
    
endmodule
```

---

### Phase 3: Testing Strategy

#### Step 3.1: MCU Standalone Test
1. Test SPI initialization
2. Test DAC playback (use existing Lab4 test)
3. Test command handler with hardcoded commands

#### Step 3.2: FPGA Standalone Test
1. Create testbench for SPI slave
2. Verify SPI communication protocol
3. Test command generation logic

#### Step 3.3: Integration Test
1. Connect FPGA and MCU
2. Send test commands from FPGA
3. Verify MCU plays correct sounds
4. Test timing and handshaking

---

## Key Design Decisions

### 1. SPI Mode
- **Use Mode 0** (CPOL=0, CPHA=0) - same as Lab07
- Clock idle low, data sampled on rising edge

### 2. Command Protocol
- **Simple 1-byte commands** - easier than multi-byte protocol
- **No checksum/error checking** initially - can add later if needed

### 3. Handshaking
- **Option A (Simple)**: FPGA sends command, MCU processes immediately
- **Option B (With ACK)**: Use LOAD/DONE signals for handshaking
- **Recommendation**: Start with Option A, add Option B if needed

### 4. Timing Considerations
- Drum sounds are ~100-500ms long
- SPI communication is fast (~microseconds)
- MCU can handle multiple triggers (queue or ignore if busy)

### 5. Concurrent Playback
- **Current**: One sound at a time (simpler)
- **Future**: Could mix multiple sounds (more complex)

---

## File Structure

### MCU Side (Lab4-Final_Project/)
```
Lab4-Final_Project/
├── main.c                    (Modified - add SPI polling and command handler)
├── STM32L432KC_SPI.h         (NEW - Copy from lab07_mcu/lib/)
├── STM32L432KC_SPI.c         (NEW - Copy from lab07_mcu/lib/)
├── STM32L432KC_DAC.h         (Existing - from Lab4)
├── STM32L432KC_DAC.c         (Existing - from Lab4)
├── STM32L432KC_GPIO.h        (Existing - needed for SPI)
├── STM32L432KC_GPIO.c        (Existing - needed for SPI)
├── STM32L432KC_RCC.h         (Existing - needed for SPI)
├── STM32L432KC_RCC.c         (Existing - needed for SPI)
├── wav_arrays/               (Existing - drum samples)
│   ├── drum_samples.h
│   ├── kick_sample.c
│   ├── snare_sample.c
│   ├── hihat_closed_sample.c
│   ├── hihat_open_sample.c
│   ├── crash_sample.c
│   ├── ride_sample.c
│   ├── tom_high_sample.c
│   └── tom_low_sample.c
└── ... (other existing files)
```

### FPGA Side (Old_SPI_test_xa/ or new directory)
```
FPGA_Project/
├── spi_test_top.sv           (EXISTING - modify to add MCU SPI interface)
├── drum_trigger_processor.sv (EXISTING - outputs drum_code)
├── drum_spi_slave.sv         (NEW - SPI slave for MCU communication)
├── bno085_controller.sv      (EXISTING - BNO085 sensor interface)
├── spi_master.sv             (EXISTING - BNO085 SPI master)
├── button_debouncer.sv       (EXISTING)
├── quaternion_to_euler_dsp.sv (EXISTING)
├── yaw_normalizer.sv         (EXISTING)
├── drum_zone_detector.sv     (EXISTING)
├── strike_detector.sv        (EXISTING)
├── drum_selector.sv           (EXISTING)
└── ... (other existing modules)
```

### Source Locations
- **Lab07 SPI Code**: `/Users/emmettstralka/Downloads/hmc-e155-main 3/lab/lab07/lab07_mcu/lib/`
- **Lab4 Drum Playback**: `/Users/emmettstralka/Desktop/E155_SPI_Final Project/Lab4-Final_Project/`
- **Existing FPGA Code**: `/Users/emmettstralka/Desktop/E155_SPI_Final Project/Old_SPI_test_xa/`

---

## Potential Issues & Solutions

### Issue 1: SPI Timing
- **Problem**: MCU and FPGA clocks may not be synchronized
- **Solution**: Use SPI clock from MCU (sck) - FPGA samples on edges

### Issue 2: Command Loss
- **Problem**: FPGA sends command while MCU is playing sound
- **Solution**: Queue commands or ignore if MCU is busy

### Issue 3: Multiple Triggers
- **Problem**: Multiple drums hit simultaneously
- **Solution**: Priority system or command queue

### Issue 4: DAC Interference
- **Problem**: SPI communication might interfere with DAC timing
- **Solution**: Use interrupts or ensure SPI is fast enough

---

## Next Steps

### Immediate Actions:
1. **Copy SPI files** from Lab07 to Lab4 project:
   - Copy `STM32L432KC_SPI.h` and `STM32L432KC_SPI.c` from `lab07_mcu/lib/` to `Lab4-Final_Project/`
   
2. **Modify main.c** in Lab4 project:
   - Add SPI initialization (from Lab07)
   - Add GPIO setup for LOAD/DONE/CE pins
   - Add SPI polling loop
   - Add `handle_drum_command()` function
   - Map FPGA drum codes (0-7) to Lab4 drum samples

3. **Test MCU side standalone**:
   - Test SPI initialization
   - Test DAC playback (existing Lab4 test)
   - Test command handler with hardcoded commands (simulate FPGA)

### FPGA Integration:
4. **Create `drum_spi_slave.sv`** module:
   - Based on `aes_starter.sv` SPI interface from Lab07
   - Simplified to send 1 byte drum command
   - Handles LOAD/DONE handshaking

5. **Modify FPGA top-level** (`spi_test_top.sv` or create new):
   - Connect existing `drum_trigger_processor` outputs
   - Add `drum_spi_slave` instance
   - Wire drum_code to SPI slave when drum_trigger_valid

6. **Test FPGA side**:
   - Create testbench for SPI slave
   - Verify SPI communication protocol
   - Test with simulated drum triggers

### Integration:
7. **Connect FPGA and MCU**:
   - Wire SPI pins (SCK, MOSI, MISO, LOAD, DONE)
   - Power up both systems
   - Test end-to-end: FPGA drum detection → SPI → MCU playback

8. **Debug and optimize**:
   - Verify timing (SPI speed, handshaking)
   - Handle concurrent triggers (queue or ignore)
   - Test all 8 drum types

---

## References

- **Lab07**: `/Users/emmettstralka/Downloads/hmc-e155-main 3/lab/lab07/`
- **Lab4**: `/Users/emmettstralka/Desktop/E155_SPI_Final Project/Lab4-Final_Project/`
- **SPI Protocol**: See `lab07_mcu/src/lab7.c` and `lab07_fpga/src/aes_starter.sv`
- **DAC Playback**: See `Lab4-Final_Project/main.c` and `STM32L432KC_DAC.c`

