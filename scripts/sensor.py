import serial
import serial.tools.list_ports
import sys

def find_serial_port():
    # Find list of all available serial ports
    ports = serial.tools.list_ports.comports()
    
    # Filter for USB serial ports common in Linux (e.g., /dev/ttyUSB* or /dev/ttyACM*)
    usb_ports = [p.device for p in ports if "ttyUSB" in p.device or "ttyACM" in p.device]
    
    if usb_ports:
        return usb_ports[0]
    
    # Default fallback for Linux USB serial port
    print("using acm0")
    return '/dev/ttyACM0'

def main():
    port = find_serial_port()
    baudrate = 115200
    
    print(f"Connecting to {port} at {baudrate} baud...")
    
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            # Flush input buffers to start clean
            ser.reset_input_buffer()
            print("Connected successfully! Printing incoming data (Press Ctrl+C to exit):")
            
            while True:
                if ser.in_waiting > 0:
                    # Read the line of raw bytes received
                    raw_data = ser.read_all()

                    # Decode to string (using replace to handle any malformed characters)
                    decoded_data = raw_data.decode('utf-8', errors='replace').strip()
                    print(decoded_data)
                    # Flush stdout to ensure output is printed immediately
                    sys.stdout.flush()
    except KeyboardInterrupt:
        print("\nStopping data reception.")
    except Exception as e:
        print(f"\nError: {e}")

if __name__ == "__main__":
    main()
