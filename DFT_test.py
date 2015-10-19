import matplotlib.pyplot as plt
import numpy as np
from numpy import fft 


# ------------------------------------
# DFT
# ------------------------------------

#signal

wx = 50
wy = [100,150]
wz = 250

n = 1000
timeStep = 0.001

t = np.linspace(0.0, n*timeStep, n)

z = np.sin(2*np.pi*t * wz)
x = np.sin(2*np.pi*t * wx) 
y = np.sin(2*np.pi*t * wy[0]) + 0.5*np.sin(2*np.pi*t * wy[1])



zf = fft.fft(z)/n
zf = fft.fftshift(zf)   #shift zero f to center

xf = fft.fft(x)/n
xf = fft.fftshift(xf)

yf = fft.fft(y)/n
yf = fft.fftshift(yf)

freq = fft.fftfreq(n,d=timeStep)
freq = fft.fftshift(freq)



plt.figure()
plt.hold(True)
plt.plot(freq, np.abs(zf)**2, 'g')
plt.plot(freq, np.abs(xf)**2, 'b')
plt.plot(freq, np.abs(yf)**2, 'r')

plt.grid()
=======
import serial
import serial.tools.list_ports
from time import sleep
import matplotlib.pyplot as plt
import numpy as np
from numpy.fft import fft as fft

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
# DFT
# ------------------------------------
z = np.random.randint(100,400,size=1000)
x = np.random.randint(100,400,size=1000)
y = np.random.randint(100,400,size=1000)
zf = fft(z)
xf = fft(x)
yf = fft(y)

#calculate time step:
steps =  np.array([], dtype=float)
for i in range(1,t.size):
    step = t[i] - t[i-1]
    steps= np.append(steps, step)
timeStep = np.mean(steps)/1000
timeStep = 2/1000

n = t.size
freq = np.fft.fftfreq(n,d=timeStep)

plt.figure()
plt.hold(True)
plt.plot(freq, zf, 'g')
plt.plot(freq, xf, 'b')
plt.plot(freq, yf, 'r')

>>>>>>> 56deff5fce1474faa0228a1014afb95fb7ab6bd7
plt.show()
