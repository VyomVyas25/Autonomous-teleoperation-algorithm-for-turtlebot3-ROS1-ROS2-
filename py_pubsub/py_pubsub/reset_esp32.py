import serial
import time

def reset_esp32(serial_port="/dev/ttyUSB0"):
    try:
        ser = serial.Serial(serial_port, 115200)
        ser.rts = False  # Pull RTS low to reset
        time.sleep(0.1)
        ser.rts = True   # Release RTS
        ser.close()
        print("ESP32 reset successfully!")
    except Exception as e:
        print(f"Failed to reset ESP32: {e}")

if __name__ == "__main__":
    reset_esp32()
