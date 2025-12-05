# DFU Upload Troubleshooting

## Error: "No DFU capable USB device available"

This error means the Arduino IDE can't find your board in DFU (Device Firmware Update) mode.

## Solutions:

### 1. Check Board Selection
- Go to **Tools → Board**
- Make sure you've selected the correct Arduino board (e.g., Arduino Nano 33 BLE, Arduino Uno, etc.)
- If using a generic board, try selecting a specific variant

### 2. Check Port Selection
- Go to **Tools → Port**
- Make sure a port is selected (should show something like `/dev/cu.usbmodem...` or `COM...`)
- If no port appears, the board may not be connected or drivers are missing

### 3. Put Board in Bootloader Mode
Some boards need to be manually put into bootloader/DFU mode:

**For Arduino boards with a reset button:**
1. Press and hold the RESET button
2. While holding RESET, press and release the BOOT button (if available)
3. Release RESET button
4. Try uploading again

**For Arduino Nano 33 BLE:**
- Double-tap the RESET button quickly to enter bootloader mode
- The LED should blink rapidly when in bootloader mode

**For generic STM32 boards:**
- Hold BOOT button, press RESET, release RESET, then release BOOT

### 4. Check USB Cable
- Try a different USB cable (some cables are power-only, not data)
- Make sure it's a data-capable USB cable
- Try a different USB port on your computer

### 5. Install/Update Drivers
- For STM32-based boards, you may need STM32 DFU drivers
- For Arduino boards, install Arduino drivers
- Check Arduino website for board-specific drivers

### 6. Try Manual DFU Mode
If your board supports it:
1. Disconnect the board
2. Put it in DFU mode (see step 3 above)
3. Connect USB cable
4. Check if it appears in Device Manager (Windows) or `lsusb` (Linux/Mac)
5. Try uploading again

### 7. Alternative Upload Methods
- Try using a different upload method in **Tools → Programmer**
- Some boards support different bootloaders (e.g., STLink, JLink)

## Quick Checklist:
- [ ] Correct board selected in Tools → Board
- [ ] Port selected in Tools → Port
- [ ] Board connected via USB
- [ ] USB cable supports data (not power-only)
- [ ] Board in bootloader/DFU mode
- [ ] Drivers installed for your board

## For Arduino Nano 33 BLE Specifically:
1. Double-tap RESET button (LED should blink rapidly)
2. Upload within the bootloader timeout window (~10 seconds)
3. If it fails, try again immediately after double-tap

