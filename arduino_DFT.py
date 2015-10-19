import serial
import serial.tools.list_ports
from time import sleep
import matplotlib.pyplot as plt
import numpy as np
from numpy import fft
import multiprocessing as mp
from playTone import playTone


outputFrequency = 200    # Hz
serialSpeed = 115200

# -----------------------------------------------------------------------------

def startSample(speed):

    serialSpeed = speed

    x = np.array([], dtype=float)
    y = np.array([], dtype=float)
    z = np.array([], dtype=float)
    t = np.array([], dtype=float)

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
    started = 0
    
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
    
    return (t,x,y,z)


if __name__ == '__main__':

    #Start playing sound and sampling simultaneously:
    pool = mp.Pool(processes=2)
    results = pool.apply_async(startSample, args=(serialSpeed,))
    closecode = pool.apply_async(playTone, args=(outputFrequency, 20,))
    
    results = results.get()
    
    t = results[0]
    x = results[1]
    y = results[2]
    z = results[3]
    
    # ------------------------------------
    # IZRAČUN POSPEŠKOV
    # ------------------------------------
    zero = 350.5    # povprečna vrednsot za (ADXL335)
    g1 = 428.2      # povprečna vrednost z osi pri merovanju (ADXL335)
    # g = meritev - zero / (g1-zero)

    x = (x - zero) / (g1-zero) * 9.81
    y = (y - zero) / (g1-zero) * 9.81
    z = (z - zero) / (g1-zero) * 9.81

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
    timeStep = np.mean(steps)/1000  # /1000, ker je čas v ms

    zf = fft.fft(zn)/n
    xf = fft.fft(xn)/n
    yf = fft.fft(yn)/n
    freq = fft.fftfreq(n,d=timeStep)

    # ------------------------------------
    # FILTER ?
    # ------------------------------------

    izbor = freq >= 0

    graf, ax = plt.subplots(2,1)
    ax[0].hold(True)
    ax[0].plot(freq[izbor], np.abs((zf)**2)[izbor], 'g')
    ax[0].plot(freq[izbor], np.abs((xf)**2)[izbor], 'b')
    ax[0].plot(freq[izbor], np.abs((yf)**2)[izbor], 'r')
    ax[0].set_xlabel(r'$f$ [Hz]')

    ax[1].hold(True)
    ax[1].plot(t,x,'b')
    ax[1].plot(t,y,'r')
    ax[1].plot(t,z,'g')
    ax[1].set_xlabel(r'$t$ [s]')
    ax[1].set_ylabel(r'$\ddot{x}$ [m/s$^2$]')

    plt.show()
