import serial
import time

# Replace with the correct port for the ESP32
port = "/dev/ttyUSB0"  # For Linux/Mac
# port = "COM3"        # For Windows

baudrate = 115200  # Standard ESP32 baud rate

with serial.Serial(port, baudrate) as ser:
    # Trigger reset
    ser.dtr = False  # Set DTR low
    ser.rts = True   # Set RTS high
    time.sleep(0.1)  # Short delay for reset
    ser.dtr = True   # Set DTR high again
    ser.rts = False  # Set RTS low again

    # Ensure both DTR and RTS are released (set to high)
    ser.dtr = True
    ser.rts = True

    print("ESP32 reset successfully and returned to normal mode.")

