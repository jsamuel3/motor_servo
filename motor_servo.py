import serial
import time

ser = serial.Serial('/dev/ttyS0', baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)

ser.isOpen()

print("Connected to: " + ser.portstr)
while True:
    input = raw_input("Enter angle for servo: ")
    p = int(float(input));
    if p >= 0:
        sgn = 1
    else:
        sgn = 0
    
    md = abs(p) % 256
    dv = abs(p) / 256

    chk = ((sgn + dv + md)%256)
    a =(str(chr(0xFF) + chr(0xFF) + chr(sgn) + chr(md) + chr(dv) + chr(chk)))
    ser.write(a)
