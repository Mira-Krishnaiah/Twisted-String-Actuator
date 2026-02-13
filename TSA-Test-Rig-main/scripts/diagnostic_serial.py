"""
Diagnostic script to see what the ESP32 is actually sending
"""

try:
    import serial
except ImportError as exc:
    raise SystemExit(
        "PySerial is required. Install with 'pip install pyserial'."
    ) from exc

import time

# Config
SERIAL_PORT = 'COM5'  # Change this to match your port
BAUD_RATE = 115200

def main():
    ser = None
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
        print("Waiting for data... (Press Ctrl+C to stop)")
        print("=" * 60)
        
        buffer = b""
        line_count = 0

        while True:
            if ser.in_waiting:
                buffer += ser.read(ser.in_waiting)
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    line_count += 1
                    
                    # Show raw bytes
                    print(f"\n[Line {line_count}] Raw bytes: {line}")
                    
                    # Try to decode
                    try:
                        decoded = line.decode('utf-8').strip()
                        print(f"[Line {line_count}] Decoded: '{decoded}'")
                        
                        # Try to split by comma
                        if ',' in decoded:
                            parts = decoded.split(',')
                            print(f"[Line {line_count}] Split into {len(parts)} parts: {parts}")
                        else:
                            print(f"[Line {line_count}] No commas found")
                    except UnicodeDecodeError as e:
                        print(f"[Line {line_count}] Decode error: {e}")
                    
                    print("-" * 60)
            
            time.sleep(0.01)
            
    except serial.SerialException as e:
        print(f"\nSerial error: {e}")
        print("\nTroubleshooting:")
        print("1. Check that COM5 is the correct port")
        print("2. Make sure no other program is using the port")
        print("3. Try unplugging and replugging the USB cable")
    except KeyboardInterrupt:
        print("\n\nDiagnostic stopped by user.")
        print(f"Total lines received: {line_count}")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial connection closed.")

if __name__ == '__main__':
    main()
