import picamera
import io
import time
import datetime
import keyboard
import math
import pyaudio
from threading import Thread
import sys
import RPi.GPIO as GPIO

GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
GPIO.setup(8, GPIO.OUT, initial=GPIO.LOW) # Set pin 8 to be an output pin for video timestamp and set initial value to low (off)
GPIO.setup(10, GPIO.OUT, initial=GPIO.LOW) # Set pin 8 to be an output pin for video timestamp and set initial value to low (off)
GPIO.setup(12, GPIO.OUT, initial=GPIO.LOW) # Set pin 12 to be an output pin for stim and set initial value to low (off)
GPIO.setup(16, GPIO.OUT, initial=GPIO.LOW) # Set pin 16 to be an output pin for led and set initial value to low (off)
GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW) # Set pin 18  to be an output pin for led timestamp and set initial value to low (off)

# audio garbage stuff....
LENGTH=1
FREQUENCY=16000
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
# end of audio garbage

class motherfucker(object):
    def __init__(self, camera, fn):
        self.camera = camera
        self.video_output = io.open(fn, 'wb')
    def write(self, buf):
        self.video_output.write(buf)
        if self.camera.frame.complete:
            Thread(target=pulse, args=()).start()

def pulse(pulse_len=0.005, pin=8):
    GPIO.output(pin, GPIO.HIGH) # Turn on
    time.sleep(pulse_len)
    GPIO.output(pin, GPIO.LOW) # Turn off

### generate mother fucking stupid sound
def beep(stream, WAVEDATA):
    Thread(target=pulse, args=(LENGTH, 10)).start()
    stream.write(WAVEDATA)
    Thread(target=pulse, args=(LENGTH, 16)).start()
    Thread(target=pulse, args=(LENGTH, 18)).start()
    time.sleep(3)
    Thread(target=pulse, args=(0.05, 12)).start()

def killcam(cam):
    print('fuck off bitch!')
    cam.stop_recording()

if len(sys.argv) < 3:
    print('Error: You need to provide animal and session IDs')
    sys.exit(0)

fn = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_") + sys.argv[1] + "_" + sys.argv[2] + ".h264"
print('starting acquisition from camera ...')

cam = picamera.PiCamera()
cam.resolution = (640, 480)
cam.framerate = 30

keyboard.add_hotkey('t', beep, args=(stream, WAVEDATA))
keyboard.add_hotkey('q', killcam, args=[cam])

cam.start_preview(fullscreen=False, window = (0, 0, 640, 480))
cam.start_recording(motherfucker(cam, fn), format='h264')
cam.wait_recording(24 * 60 * 60)
cam.stop_preview()
cam.stop_recording()
cam.close()
