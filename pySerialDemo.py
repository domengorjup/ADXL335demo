import serial
import serial.tools.list_ports
from time import sleep
import matplotlib.pyplot as plt
import numpy as np

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

started = 0
count = 0
lines = []

npoints = 500
x = [0] * npoints
y = [0] * npoints
z = [0] * npoints
t = [0] * npoints

def dataUpdate():

    global started
    global count
    if not started:
        started = 1
        print('Sampling started.\n\n')
        
    line = ser.readline().decode('utf-8').rstrip().split()
    count = count + 1

    if 'end' in line:
         raise ValueError('Sampling ended.')

    t.append(float(line[0]))
    del t[0]
    x.append(float(line[1]))
    del x[0]
    y.append(float(line[2]))
    del y[0]
    z.append(float(line[3]))
    del z[0]


while True:
    try:
        dataUpdate()

    except (KeyboardInterrupt, ValueError):
        break


print("Sampling ended.\n\nRESULT:\t%d Hz\n\nClosing serial." %count)

ser.close()

