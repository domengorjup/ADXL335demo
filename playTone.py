def playTone(f,duration):

    import pyaudio
    import numpy as np
    
    p = pyaudio.PyAudio()

    volume = 1.0
    fs = 44100      # sampling rate [Hz]
    

    # Generate samples, convert to float32
    samples = (np.sin(2*np.pi*np.arange(fs*duration)*f/fs)).astype(np.float32)


    # Sample values must be in range [-1.0, 1.0] for paFloat32
    stream = p.open(format=pyaudio.paFloat32,
                    channels=1,
                    rate=fs,
                    output=True)

    # Play
    stream.write(volume*samples)

    stream.stop_stream()
    stream.close()

    p.terminate()

    return 1


if __name__ == '__main__':
    
    playTone(300,2)
