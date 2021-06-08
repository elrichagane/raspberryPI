# Record from night vision camera and send out trigger and frame pulses
# arg 1: animal name
# arg 2: session number
# arg 3: com port (default COM1)
# arg 4: camera number

from threading import Thread
#from Queue import Queue
import cv2
import sys
#import serial
import datetime
import math
import pyaudio
import time
from multiprocessing import Process
import RPi.GPIO as GPIO

GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
GPIO.setup(8, GPIO.OUT, initial=GPIO.LOW) # Set pin 8 to be an output pin and set initial value to low (off)
GPIO.setup(10, GPIO.OUT, initial=GPIO.LOW) # Set pin 10 to be an output pin for audio tone and set initial value to low (off)
GPIO.setup(12, GPIO.OUT, initial=GPIO.LOW) # Set pin 12 to be an output pin for stim and set initial value to low (off)


class thread_stream: #create separate thread for reading frames from camera to increase FPS
    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src)
        (self.rval, self.frame) = self.stream.read()
        self.running = True

    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while self.running:
            (self.rval, self.frame) = self.stream.read()
        return

    def read(self):
        Thread(target=pulse, args=()).start()
        return self.frame

    def isRunning(self):
        return self.running

    def stop(self):
        self.running = False
        self.stream.release()

### Send a pulse
def pulse(pulse_len=0.01, pin=8):
    GPIO.output(pin, GPIO.HIGH) # Turn on
    time.sleep(pulse_len)
    GPIO.output(pin, GPIO.LOW) # Turn off

### generate mother fucking stupid sound
def beep(FREQUENCY=440, LENGTH=1):
    PyAudio = pyaudio.PyAudio     #initialize pyaudio
    
    BITRATE = 16000     #number of frames per second/frameset.      
    
    if FREQUENCY > BITRATE:
        BITRATE = FREQUENCY+100
    
    NUMBEROFFRAMES = int(BITRATE * LENGTH)
    RESTFRAMES = NUMBEROFFRAMES % BITRATE
    WAVEDATA = ''    
    
    #generating wawes
    for x in range(NUMBEROFFRAMES):
     WAVEDATA = WAVEDATA+chr(int(math.sin(x/((BITRATE/FREQUENCY)/math.pi))*127+128))    
    
    for x in range(RESTFRAMES): 
     WAVEDATA = WAVEDATA+chr(128)
    
    p = PyAudio()
    stream = p.open(format = p.get_format_from_width(1), 
                    channels = 1, 
                    rate = BITRATE, 
                    output = True)
    
    Thread(target=pulse, args=(LENGTH, 10)).start()
    stream.write(WAVEDATA)
    stream.stop_stream()
    stream.close()
    Thread(target=pulse, args=(0.05, 12)).start()
    p.terminate()


### main programme
if len(sys.argv) < 3:
    print('Error: You need to provide animal and session IDs')
    sys.exit(0)

cv2.namedWindow("Live Feed")
cam = thread_stream().start()

fourcc = cv2.VideoWriter_fourcc(*'X264')
out = cv2.VideoWriter(datetime.datetime.now().strftime("%Y%m%d_%H%M%S_") + sys.argv[1] + "_" + sys.argv[2] + ".mp4",fourcc, int(cam.stream.get(5)), (int(cam.stream.get(3)),int(cam.stream.get(4))))
print('starting acquisition from camera ...')

while cam.isRunning():
    frame = cam.read()
    out.write(frame)
    cv2.imshow("Live Feed", frame)
    key = cv2.waitKey(1)
    if key == ord('t'):
        #Thread(target=beep, args=()).start()
        Process(target=beep).start()
    elif key == ord('q'):
        break

cam.stop()
out.release()
cv2.destroyWindow("Live Feed")

print('done acquisition and exited without error :)')
