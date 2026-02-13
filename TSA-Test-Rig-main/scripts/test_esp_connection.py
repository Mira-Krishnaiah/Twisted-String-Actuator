"""
Simple test: Does the computer receive ANY data from the ESP32?
This will run for 30 seconds and show everything received.
"""

try:
    import serial
except ImportError:
    raise SystemExit("Install pyserial: pip install pyserial")

import time

SERIAL_PORT = 'COM7'
BAUD_RATE = 115200
TEST_DURATION = 30  # seconds

print("=" * 70)
print("ESP32 DATA RECEPTION TEST")
print("=" * 70)
print(f"Port: {SERIAL_PORT}")
print(f"Baud: {BAUD_RATE}")
print(f"Duration: {TEST_DURATION} seconds")
print()

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print("✓ Serial port opened successfully")
    print()
    print("Listening for data...")
    print("-" * 70)
    
    start_time = time.time()
    total_bytes = 0
    total_lines = 0
    
    while time.time() - start_time < TEST_DURATION:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            total_bytes += len(data)
            
            # Try to decode and print
            try:
                text = data.decode('utf-8')
                print(text, end='')
                total_lines += text.count('\n')
            except:
                print(f"[{len(data)} bytes of binary data]")
        
        time.sleep(0.01)
    
    print()
    print("=" * 70)
    print("TEST RESULTS:")
    print(f"  Total bytes received: {total_bytes}")
    print(f"  Total lines received: {total_lines}")
    print()
    
    if total_bytes == 0:
        print("✗ NO DATA RECEIVED")
        print()
        print("Possible issues:")
        print("  1. Wrong COM port (try COM3 or check Device Manager)")
        print("  2. ESP32 code not running (press RESET button)")
        print("  3. Wrong baud rate (verify it's 115200 in Arduino code)")
        print("  4. USB cable is data-only (try different cable)")
    else:
        print("✓ DATA IS BEING RECEIVED")
        print()
        print("The ESP32 is sending data successfully!")
        print("If data logger still doesn't work, the issue is in parsing.")
    
    ser.close()
    
except serial.SerialException as e:
    print(f"✗ ERROR: {e}")
    print()
    print("Troubleshooting:")
    print("  1. Check Device Manager for correct COM port")
    print("  2. Close any other programs using the port")
    print("  3. Unplug and replug the USB cable")
    
except KeyboardInterrupt:
    print("\n\nTest stopped early by user")
    ser.close()

print()
print("Test complete.")
