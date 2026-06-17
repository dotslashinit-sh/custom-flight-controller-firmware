import serial

port = 'COM3' 
baudrate = 115200

def get_port_choice():
    serial.tools.list_ports.comports()
    
try:
    # Open the serial port
    ser = serial.Serial(port, baudrate, timeout=1)
    
    # Send data (must be encoded to bytes)
    ser.write(b'Hello UART!')
    
    # Read incoming data
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').strip()
        print(f"Received: {data}")

except Exception as e:
    print(f"Error: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
