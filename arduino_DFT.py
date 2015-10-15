import serial
import serial.tools.list_ports
from time import sleep
import matplotlib.pyplot as plt
import numpy as np
from numpy import fft 

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

x = np.array([], dtype=float)
y = np.array([], dtype=float)
z = np.array([], dtype=float)
t = np.array([], dtype=float)

while True:
    try:
        if not started:
            started = 1
            print('Sampling started.\n\n')
        
        line = ser.readline().decode('utf-8').rstrip().split()
        count = count + 1

        if 'end' in line:
             raise ValueError('Sampling ended.')

        t = np.append(t, float(line[0]))
        x = np.append(x, float(line[1]))
        y = np.append(y, float(line[2]))
        z = np.append(z, float(line[3]))

    except (KeyboardInterrupt, ValueError):
        break


print("Sampling ended.\n\nRESULT:\t%d samples\n\nClosing serial." %count)

ser.close()


# ------------------------------------
# IZRAČUN POSPEŠKOV
# ------------------------------------
zero = 350.5    # povprečna vrednsot za (ADXL335)
g1 = 428.2      # povprečna vrednost z osi pri merovanju (ADXL335)
# g = meritev - zero / (g1-zero)

x = (x - zero) / (g1-zero)
y = (y - zero) / (g1-zero)
z = (z - zero) / (g1-zero) 

# remove DC offet
xn = x - np.mean(x)
yn = y - np.mean(y)
zn = z - np.mean(z)

# ------------------------------------
# DFT
# ------------------------------------

n = t.size

#calculate time step:
steps =  np.array([], dtype=float)
for i in range(1,t.size):
    step = t[i] - t[i-1]
    steps= np.append(steps, step)
timeStep = np.mean(steps)#/1000  # /1000, ker je čas v ms

zf = fft.fft(zn)/n
zf = fft.fftshift(zf)   #shift zero f to center

xf = fft.fft(xn)/n
xf = fft.fftshift(xf)

yf = fft.fft(yn)/n
yf = fft.fftshift(yf)

freq = fft.fftfreq(n,d=timeStep)
freq = fft.fftshift(freq)


# ------------------------------------
# FILTER ?
# ------------------------------------


plt.figure()
plt.hold(True)
plt.plot(freq, np.abs((zf)**2), 'g')
plt.plot(freq, np.abs((xf)**2), 'b')
plt.plot(freq, np.abs((yf)**2), 'r')


plt.figure()
plt.hold(True)
plt.plot(t,x,'b')
plt.plot(t,y,'r')
plt.plot(t,z,'g')

plt.show()
