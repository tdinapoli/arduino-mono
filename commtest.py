import serial
import time

# Adjust this to match your Arduino's port
port = '/dev/ttyUSB1'  # Or 'COM3' on Windows
baudrate = 9600

with serial.Serial(port, baudrate, timeout=2) as ser:
    time.sleep(2)  # Wait for Arduino to reset
    command = '13 5 1 1\n'  # Format: N onTime offTime
    ser.write(command.encode())

    time.sleep(5)
    response = ser.readline().decode().strip()
    print("Arduino response:", response)

    ser.write(b'READ 13\n')
    state = ser.readline().decode().strip()
    print("Pin 12 state:", state)



