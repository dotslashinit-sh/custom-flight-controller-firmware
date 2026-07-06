import serial
import serial.tools.list_ports
import sys
import time
import platform


def find_serial_port():
    """Find a likely serial port, working on both Windows and Linux."""
    ports = serial.tools.list_ports.comports()

    if platform.system() == "Windows":
        # On Windows, ports look like COM3, COM4, etc.
        candidates = [p.device for p in ports if p.device.upper().startswith("COM")]
        if candidates:
            return candidates[0]
        print("No COM ports found, defaulting to COM3")
        return "COM3"
    else:
        # Linux/Mac: look for USB serial devices
        candidates = [
            p.device for p in ports
            if "ttyUSB" in p.device or "ttyACM" in p.device or "usbserial" in p.device or "usbmodem" in p.device
        ]
        if candidates:
            return candidates[0]
        print("No USB serial ports found, defaulting to /dev/ttyACM0")
        return "/dev/ttyACM0"


def read_loop(ser):
    """Read from an already-open serial connection until it disconnects or Ctrl+C."""
    ser.reset_input_buffer()
    print("Connected successfully! Printing incoming data (Press Ctrl+C to exit):")

    while True:
        if ser.in_waiting > 0:
            raw_data = ser.read_all()
            decoded_data = raw_data.decode("utf-8", errors="replace").strip()
            if decoded_data:
                print(decoded_data)
                sys.stdout.flush()
        else:
            # Small sleep avoids pegging a CPU core while polling in_waiting
            time.sleep(0.01)


def main():
    port = find_serial_port()
    baudrate = 115200

    while True:
        print(f"Connecting to {port} at {baudrate} baud...")
        try:
            with serial.Serial(port, baudrate, timeout=1) as ser:
                read_loop(ser)

        except KeyboardInterrupt:
            print("\nStopping data reception.")
            break

        except (serial.SerialException, OSError) as e:
            # This fires when the device is unplugged, the OS drops the port,
            # or the port never opens in the first place.
            print(f"\nSerial connection lost or unavailable ({e}). "
                  f"Retrying on {port} in 2 seconds...")
            time.sleep(2)
            continue

        except Exception as e:
            print(f"\nUnexpected error: {e}")
            time.sleep(2)
            continue


if __name__ == "__main__":
    main()