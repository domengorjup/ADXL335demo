import serial
import serial.tools.list_ports
from time import sleep

serialSpeed = 115200

ports = list(serial.tools.list_ports.comports())
for p in ports:
    print(p)
    if "Arduino" in p[1]:
        arduinoPort = p[0]
    elif "Silicon" in p[1]:
        arduinoPort = p[0]

ser = serial.Serial(arduinoPort, timeout=None, baudrate=serialSpeed)

if ser.isOpen():
    ser.close()
ser.open()

print(ser.name, 'serial connection established at', serialSpeed, 'bps.\n')

count = 0
lines = []
started = 0;

while True:

    if not started:
        started = 1
        print('Sampling started.\n\n')
        
    line = ser.readline().decode('utf-8').rstrip()
    count = count + 1

    if line == "end":
         break

    lines.append(line)

print("Sampling ended.\n\nRESULT:\t%d Hz\n\nClosing serial." %count)

ser.close()
