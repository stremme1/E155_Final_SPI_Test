# Arduino Library Installation Instructions

## Issue
The Adafruit BNO08x library requires two dependencies:
- Adafruit Unified Sensor
- Adafruit BusIO

## Solution

### Option 1: Install via Arduino IDE Library Manager (Recommended)

1. Open Arduino IDE
2. Go to **Tools → Manage Libraries...**
3. Search for and install:
   - **"Adafruit Unified Sensor"** (by Adafruit)
   - **"Adafruit BusIO"** (by Adafruit)
4. Restart Arduino IDE
5. The Adafruit BNO08x library should now work (already copied to your libraries folder)

### Option 2: Resolve Library Conflict

If you get an error about Adafruit_BusIO already existing:

1. Check if there's a conflicting library at:
   `/Users/emmettstralka/Documents/GitHub/E80-Team15/libraries/Adafruit_BusIO`

2. Either:
   - Remove the old library from that location, OR
   - Install the libraries in Arduino IDE which will use the standard location

### Option 3: Manual Installation (if Library Manager fails)

1. Download from GitHub:
   - Adafruit Unified Sensor: https://github.com/adafruit/Adafruit_Sensor
   - Adafruit BusIO: https://github.com/adafruit/Adafruit_BusIO

2. Extract to: `~/Documents/Arduino/libraries/`
   - Should be: `~/Documents/Arduino/libraries/Adafruit_Sensor/`
   - Should be: `~/Documents/Arduino/libraries/Adafruit_BusIO/`

3. Restart Arduino IDE

## Verify Installation

After installation, verify the libraries are found:
- Arduino IDE should compile without "No such file or directory" errors
- The `#include <Adafruit_BNO08x.h>` should work

## Current Status

✅ Adafruit BNO08x library is installed at:
   `~/Documents/Arduino/libraries/Adafruit_BNO08x/`

❌ Dependencies need to be installed:
   - Adafruit Unified Sensor
   - Adafruit BusIO

