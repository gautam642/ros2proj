import time
import serial
import random

ser = serial.Serial('/dev/pts/7', 9600)

while True:
    adc_val = random.randint(0, 1023)
    adc_str = f"!E{adc_val:04d}D"
    # ser.write(adc_str.encode())
    ser.write((adc_str + '\n').encode())
    print(f"Sent: {adc_str}")
    time.sleep(1)
