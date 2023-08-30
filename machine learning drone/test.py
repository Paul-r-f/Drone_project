import serial
import time

arduino = serial.Serial(port='/dev/cu.usbmodem145401', baudrate=9600, timeout=.1)

def write_read(x):
    arduino.write(x.encode('utf-8'))
    arduino.flush()
    time.sleep(5.05)
    data = arduino.readline()
    return data

while True:
    num = input("Enter a number: ")
    value = write_read(num)
    print(value.decode('utf-8').strip())

