# -----------------------
# FUNCTION DEFINITIONS  |
# -----------------------

def dataUpdate():

    global count
    global started
    global x
    global y
    global z
    global t
    
    if not started:
        started = 1
        print('Sampling started.\n\n')
        
    line = ser.readline().decode('utf-8').rstrip().split()
    count = count + 1

    if 'end' in line:
         raise ValueError('Sampling ended.')

    #t = np.append(t, float(line[0]))
    #t = t[1:]
    x = np.append(x, float(line[1]))
    x = x[1:]
    y = np.append(y, float(line[2]))
    y = y[1:]
    z = np.append(z, float(line[3]))
    z = z[1:]
  


# --------------
# EXECUTITION  |
# --------------

if __name__ == '__main__':

    import serial
    import serial.tools.list_ports
    import time
    import matplotlib.pyplot as plt
    from matplotlib import animation
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

    npoints = 1000
    x = np.zeros(npoints, dtype=float)
    y = np.zeros(npoints, dtype=float)
    z = np.zeros(npoints, dtype=float)
    t = np.zeros(npoints, dtype=float)


    #plot init
    fig, ax = plt.subplots(1,1)
    ax.set_xlim(0, 1000)
    ax.set_ylim(300, 500)
    ax.hold(True)
    plt.show(block=False)
    plt.draw()

    background = fig.canvas.copy_from_bbox(ax.bbox)
    
    xaxis = np.arange(0,npoints,1)
    liZ, = ax.plot(xaxis,z)
    liX, = ax.plot(xaxis,x)
    liY, = ax.plot(xaxis,y)


    while True:
        try:
            dataUpdate()
            
            liZ.set_ydata(z)
            liX.set_ydata(x)
            liY.set_ydata(y)
            
            fig.canvas.restore_region(background)
            ax.draw_artist(liX)
            ax.draw_artist(liY)
            ax.draw_artist(liZ)
            
            fig.canvas.blit(ax.bbox)
            

        except (KeyboardInterrupt, ValueError):
            break

    print("\nSampling ended.\n\nRESULT:\t%d Hz\n\nClosing serial." %count)

    ser.close()

