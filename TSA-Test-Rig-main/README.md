# TSA Test Rig  
Twisted String Actuator Test Rig (ModLab)  

## TL;DR — Current state & what you need now

**Current setup:** The rig runs with **IR sensors** for cycle detection and **AS5600 encoder** for angle logging. Motor control and cycle counting are still driven by the two IR sensors (IR1, IR2). The encoder (on a separate Arduino) sends angle data to the ESP32, which is merged with cycle/current/voltage and logged.

**What you need to run a test right now:**
1. **ESP32** — Flash `test_rig_encoder.ino` (motor + IR logic, receives angle from Arduino).
2. **Arduino** — Flash `AS5600_Serial_Sender.ino`; connect AS5600 (SDA→A4, SCL→A5) and Arduino TX (Pin 1) → ESP32 Pin 12.
3. **Data logging** — Set `SERIAL_PORT` in `scripts/data_logger_encoder.py` to the ESP32’s COM port, then run `python scripts/data_logger_encoder.py`. Output CSV: `data/test_rig_data_with_angle_YYYYMMDD_HHMMSS.csv` with columns *Elapsed_s, CycleCount, Current_mA, Voltage_mV, Angle_deg*.

**Plan:** Once encoder-based cycle logic is verified, IR sensors will be removed and cycle/direction will be driven by encoder angle only.

---
## Onshape Files
https://cad.onshape.com/documents/71c3879f7e0b9d79bb76d53f/w/3467ad9f87a1c5e1faffa6f6/e/e31486eae4ef6945b7a7092c?renderMode=0&uiState=698f47f82271c575ed94ab59

## Overview  
This rig is designed to run life-cycle tests on twisted string actuators (TSAs) using Spiderwire Dyneema line (0.12 mm). The setup drives a 6 V coreless DC motor through a custom MOSFET H-bridge, cycles the string between two IR sensor endpoints, and logs current, voltage, and cycle count for wear testing.  

The system is divided into two major structures:  
- **Rig:** an 80/20 frame with the TSA mounted on the top rail.  
- **Electronics block:** contains the ESP32-S3, INA3221 current/voltage sensor, custom H-bridge, and power connections.  

## Hardware Requirements  
- ESP32-S3 WROOM-1 (this is overkill but it works, can also use a different)  
- Arduino (Uno/Nano/etc.) for AS5600 encoder reading
- AS5600 Magnetic Encoder (connected to Arduino via I2C)
- 2 × IR sensors (endpoint detection) - *Optional: encoder-based version uses AS5600 instead*
- INA3221 current/voltage sensor  
- 6 V coreless DC motor (3 A stall): NFP-D0816-4-6.0
- Custom H-bridge (IRLB8721 MOSFETs) with 9 V battery as gate-driver supply  
- 6 V regulated power supply (≥4 A capacity)  
- Spiderwire Dyneema test string (0.12 mm)  
- 100 g weight (back-driving force)  

## Setup  
The TSA is built from several linkages joined with 10-44 bolts and magnets.  

- The motor is mounted on a link and rotationally fixed with a steel wire. A cone-shaped tip on the motor shaft anchors the string.  
- The string connects between two posts and the cone, forming a Y-shaped structure. Ideally the arms of the Y form about 90°. If the angle is too wide, the string may snag.  
- The TSA is unidirectional — the motor drives in one direction only. To return, a 100 g hanging weight provides the back-driving force.  
- IR1 (lower right) triggers when the weight passes it. IR2 (upper left) triggers at full TSA extension.  
- The TSA is fixed in the top right corner with an 80/20 T-slot nut and pivots around its bolt.  

## Schematic  
*Add schematic image here once available*  

## Operation  
1. **Restring if needed**  
   - Tie a bowline knot on the top post.  
   - Measure the string length by holding it at the cone, then extending it to the lower post. Leave about 2 inches extra.  
   - Tie the string reasonably tight. The cone tip should spin at least once in either direction, but not much more.  

2. **Connect hardware**  
   - Plug your computer into the ESP32-S3 with a USB-C cable (use the COM port on the right side).  
   - Plug in the electronics block: connect the 6 V main supply *and* insert a 9 V battery into the terminal.  
   - **AS5600 Encoder Setup:**
     - Connect AS5600 to Arduino: SDA→A4, SCL→A5, VCC→5V, GND→GND
     - Connect Arduino TX (Pin 1) to ESP32 Pin 12 (RX)
     - Connect Arduino GND to ESP32 GND
     - Upload `AS5600_Serial_Sender.ino` to the Arduino
   - Flash `test_rig_encoder.ino` sketch onto the ESP32-S3
   - Double-check wiring. Pin 12 on ESP32-S3 should be connected to Arduino TX, not ground.  

3. **Start system**  
   - Power on the Arduino (with AS5600) first - it should start sending angle data
   - Press the reset button on the ESP32-S3.  
   - Start the data logger (see Data Logging section).  
   - Wave your hand in front of IR2 to trigger the first cycle (for IR-based version) or the system will auto-start (for encoder-based version).  

4. **Running**  
   - The rig will continue cycling until it times out due to a snag, snap, or blockage.  
   - Once fixed, press the reset button on pin 13 (or ground pin 13) and wave your hand in front of IR2 to restart (IR-based) or the system will auto-restart (encoder-based).  

## Data Logging  

### For Encoder-Based System (Recommended)
1. Ensure both ESP32-S3 and Arduino are powered and connected
2. Update the `SERIAL_PORT` variable in `scripts/data_logger_encoder.py` to match your ESP32-S3 COM port
3. Install required Python package: `pip install pyserial`
4. Run the logger with `python scripts/data_logger_encoder.py`
5. Data is automatically saved to `data/test_rig_data_with_angle_YYYYMMDD_HHMMSS.csv`
6. A new CSV file is created each time the ESP32 resets (power cycle)
7. Stop logging with Ctrl+C

### For IR Sensor-Based System (Legacy)
1. Update the `SERIAL_PORT` variable in `scripts/data_logger.py` to match your device
2. Run the logger with `python scripts/data_logger.py`
3. Data is saved to `data/test_rig_data.csv`
4. Stop with Ctrl+C

### Data Format (Encoder-Based)
CSV columns:
- Elapsed_s (timestamp in seconds from start)
- CycleCount (number of completed cycles)
- Current_mA (motor current in milliamps)
- Voltage_mV (motor voltage in millivolts)
- Angle_deg (AS5600 encoder angle in degrees)

### Data Format (IR-Based)
CSV columns:
- Elapsed_s (timestamp in seconds from start)
- CycleCount (number of completed cycles)
- Current_mA (motor current in milliamps)
- Voltage_mV (motor voltage in millivolts)  

## Variables in Code  

### For Encoder-Based System (test_rig_encoder.ino)
- Speed of motor → change `PWM_EXTEND` and `PWM_RETRACT` values
- Frequency → change `LEDC_FREQUENCY` (should be around 500)
- Cycle twist angle → change `CYCLE_TWIST_DEG` (degrees of rotation per cycle)
- Retract tolerance → change `RETRACT_TOLERANCE_DEG` (how close to start angle to count cycle)
- Fault timeout → change `SENSOR_STALL_TIMEOUT_MS` (encoder movement timeout)
- Data logger frequency → change `LOG_INTERVAL` (milliseconds between log entries)
- Angle timeout → change `ANGLE_TIMEOUT_MS` (timeout for Arduino angle data)

### For IR Sensor-Based System (test_rig.ino)
- Speed of motor → change the second value of the term: `ledcWrite(pinA, MAX_PWM)`  
- Frequency → change `LEDC_FREQUENCY` (should be around 500)  
- Manual vs. auto mode → `MANUAL_CONTROL = 0` is auto (default), 1 is manual (rarely needed)  
- Disable auto stop → comment out the section labeled "Fault handling"  
- Fault timeout → change `SENSOR_STALL_TIMEOUT_MS`  
- Delay between cycles → change `delay(1000)` under State retracting  
- Data logger frequency → change `LOG_INTERVAL`

## Troubleshooting  
- **Motor is weak** → Check the 9 V battery voltage first, then check motor wiring, then check TSA linkages for smoothness.  
- **Excessive noise** → Make sure bolt heads aren't hitting the 80/20 rails; adjust if needed. Apply a small amount of lithium grease.  
- **IR sensors not tripping** → Adjust sensor positions and room lighting. Close blinds to block sunlight interference. Avoid filming too close with smartphones (some autofocus systems use IR that disrupts sensors).
- **No angle data in logs** → 
  - Verify Arduino is powered and `AS5600_Serial_Sender.ino` is uploaded
  - Check wiring: Arduino TX (Pin 1) → ESP32 Pin 12, GNDs connected
  - Verify AS5600 I2C connection: SDA→A4, SCL→A5 on Arduino
  - Check Serial Monitor on Arduino to see if AS5600 is detected
  - Ensure both devices use 115200 baud rate
- **Angle values stuck at 0.0** → Arduino may not be sending data or ESP32 not receiving. Check Serial connection between devices.
- **Python script can't find serial port** → Update `SERIAL_PORT` in `data_logger_encoder.py` to match your ESP32 COM port (e.g., 'COM3' on Windows, '/dev/ttyUSB0' on Linux)
