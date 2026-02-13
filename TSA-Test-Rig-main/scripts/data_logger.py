"""
Twisted String Actuator Test Rig Data Logger
Usage:
    - Connect the test rig microcontroller to the computer via USB.
    - Change the SERIAL_PORT variable to the correct port for microcontroller.
    - Run the script, data will be logged in 'data/test_rig_data.csv'.
    - Press Ctrl+C to stop logging.
"""

try:
    import serial  # type: ignore
except ImportError as exc:
    raise SystemExit(
        "PySerial is required. Install with 'pip install pyserial'."
    ) from exc

import time
import csv
import os

# Config
SERIAL_PORT = 'COM7'  # Change this to match your microcontroller's port
BAUD_RATE = 115200
# Index of INA3221 channel reported; kept for clarity if future parsing changes
EXPECTED_FIELDS = 4  # cycle, current(mA), voltage(mV), angle(deg)

# Get the directory this script is in
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..', 'data'))
os.makedirs(DATA_DIR, exist_ok=True)


def make_log_path():
    timestamp = time.strftime('%Y%m%d_%H%M%S')
    filename = f"test_rig_data_with_angle_{timestamp}.csv"
    return os.path.join(DATA_DIR, filename)


def open_log():
    path = make_log_path()
    f = open(path, mode='w', newline='')
    writer = csv.writer(f)
    writer.writerow(['Elapsed_s', 'CycleCount', 'Current_mA', 'Voltage_mV', 'Angle_deg'])
    print(f"Logging data to {path}...")
    return f, writer, time.time()

def main():
    ser = None
    file, writer, start_time = open_log()
    last_cycle_count = None
    
    try:
        # Reduced timeout for faster polling
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
        buffer = b""

        while True:
            if ser.in_waiting:
                buffer += ser.read(ser.in_waiting)
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    line = line.decode('utf-8').strip()
                    if line:
                        print(line)
                        data = line.split(',')
                        if len(data) == EXPECTED_FIELDS:
                            try:
                                cycle_count = int(float(data[0]))
                                current_mA = float(data[1])
                                voltage_mV = float(data[2])
                                angle_deg = float(data[3])
                            except ValueError:
                                continue

                            # Detect controller restart (cycle counter reset)
                            if last_cycle_count is not None and cycle_count < last_cycle_count:
                                print("Cycle counter reset detected. Starting new CSV log.")
                                file.close()
                                file, writer, start_time = open_log()

                            elapsed = time.time() - start_time
                            formatted_elapsed = f"{elapsed:.4f}"
                            writer.writerow([
                                formatted_elapsed,
                                cycle_count,
                                f"{current_mA:.4f}",
                                f"{voltage_mV:.4f}",
                                f"{angle_deg:.4f}"
                            ])
                            file.flush()
                            last_cycle_count = cycle_count
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("Logging stopped by user.")
    finally:
        file.close()
        if ser and ser.is_open:
            ser.close()
            print("Serial connection closed.")

if __name__ == '__main__':
    main()