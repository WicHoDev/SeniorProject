import mat
import serial

print("Hello world")

ser = serial.Serial('COM3', 921600, timeout=1)
print("connected: ", ser.name)

ser.write(b"scan\r")
raw = ser.read(4000)
print(raw)
