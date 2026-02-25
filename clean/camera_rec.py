#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Python 2.7 (Raspberry Pi):
- Record H.264 video with PiCamera
- Emit TTL pulse on ONE GPIO pin (BOARD 8) once per completed camera frame
- Print to terminal:
    * when GPIO goes HIGH
    * when GPIO goes LOW
    * when a USB serial line is received from Arduino

Usage:
    python record_cam_terminal.py <animal_id> <session_id>

Output video:
    YYYYMMDD_HHMMSS_<animal_id>_<session_id>.h264
"""

import datetime
import io
import sys
import time
import threading

import picamera
import RPi.GPIO as GPIO

# requires pyserial on the Pi: sudo pip install pyserial
import serial


# ----------------------------
# Configuration
# ----------------------------
GPIO_MODE = GPIO.BOARD
PULSE_PIN = 8
PULSE_LEN_S = 0.005

RESOLUTION = (640, 480)
FRAMERATE = 30

SERIAL_PORT = "/dev/ttyACM0"   # adjust if needed: /dev/ttyACM1 or /dev/ttyUSB0
SERIAL_BAUD = 115200

RECORD_MAX_S = 24 * 60 * 60


# ----------------------------
# Utilities
# ----------------------------
def ts():
    """
    Timestamp string for terminal logs.
    Python 2.7 doesn't have time_ns/monotonic_ns, so we use:
      - wall clock seconds with micro-ish precision as float
      - plus a human-readable datetime
    """
    t = time.time()
    # Keep both: ISO-like and epoch seconds
    return "%s | %.6f" % (datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"), t)


_print_lock = threading.Lock()

def log(msg):
    with _print_lock:
        sys.stdout.write("[%s] %s\n" % (ts(), msg))
        sys.stdout.flush()


# ----------------------------
# GPIO
# ----------------------------
def setup_gpio(pin):
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO_MODE)
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)


def pulse(pulse_len_s=PULSE_LEN_S, pin=PULSE_PIN):
    """
    Emit TTL pulse and print exact transition moments to terminal.
    """
    GPIO.output(pin, GPIO.HIGH)
    log("GPIO_HIGH pin=%s" % str(pin))

    time.sleep(pulse_len_s)

    GPIO.output(pin, GPIO.LOW)
    log("GPIO_LOW  pin=%s" % str(pin))


# ----------------------------
# USB Serial reader (Arduino -> Pi)
# ----------------------------
class SerialReader(object):
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.stop_evt = threading.Event()
        self.thread = threading.Thread(target=self._run)
        self.thread.daemon = True
        self.ser = None

    def start(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        log("USB serial opened port=%s baud=%s" % (self.port, str(self.baud)))

        # Optional: read a bit at startup so you see READY lines
        t0 = time.time()
        while time.time() - t0 < 2.0:
            line = self._readline()
            if line:
                log("USB_RX %s" % line)
                if line.startswith("READY"):
                    break

        self.thread.start()

    def _readline(self):
        try:
            raw = self.ser.readline()
            if not raw:
                return ""
            # strip \r\n and make safe printable
            return raw.decode("utf-8", "ignore").strip()
        except Exception:
            return ""

    def _run(self):
        while not self.stop_evt.is_set():
            line = self._readline()
            if line:
                log("USB_RX %s" % line)

    def close(self):
        self.stop_evt.set()
        try:
            self.thread.join(1.0)
        except Exception:
            pass
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        log("USB serial closed")


# ----------------------------
# PiCamera output wrapper: pulse per completed frame
# ----------------------------
class PulseOnFrameComplete(io.RawIOBase):
    def __init__(self, camera, filename):
        self.camera = camera
        self.video_output = io.open(filename, "wb")

    def write(self, buf):
        n = self.video_output.write(buf)

        # When frame completes, emit pulse in a thread so write() isn't blocked
        if self.camera.frame.complete:
            t = threading.Thread(target=pulse)
            t.daemon = True
            t.start()

        return n

    def close(self):
        try:
            if not self.video_output.closed:
                self.video_output.close()
        finally:
            io.RawIOBase.close(self)


# ----------------------------
# Main
# ----------------------------
def main():
    if len(sys.argv) < 3:
        sys.stdout.write("Error: You need to provide animal and session IDs\n")
        sys.stdout.write("Usage: python record_cam_terminal.py <animal_id> <session_id>\n")
        return 1

    animal_id = sys.argv[1]
    session_id = sys.argv[2]

    base = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_") + animal_id + "_" + session_id
    video_fn = base + ".h264"

    setup_gpio(PULSE_PIN)

    serial_reader = None
    cam = None
    writer = None

    try:
        log("Saving video: %s" % video_fn)

        # Start Arduino USB RX logging (non-fatal if missing)
        try:
            serial_reader = SerialReader(SERIAL_PORT, SERIAL_BAUD)
            serial_reader.start()
        except Exception as e:
            log("WARNING: could not open serial %s (%s). Continuing without USB RX logging." %
                (SERIAL_PORT, str(e)))
            serial_reader = None

        cam = picamera.PiCamera()
        cam.resolution = RESOLUTION
        cam.framerate = FRAMERATE

        # Preview is optional; disable if headless
        cam.start_preview(fullscreen=False, window=(0, 0, RESOLUTION[0], RESOLUTION[1]))

        writer = PulseOnFrameComplete(cam, video_fn)
        cam.start_recording(writer, format="h264")

        log("Recording started. Ctrl+C to stop.")
        cam.wait_recording(RECORD_MAX_S)

        return 0

    except KeyboardInterrupt:
        log("Stopping (KeyboardInterrupt).")
        return 0

    finally:
        # Stop camera cleanly
        try:
            if cam:
                try:
                    cam.stop_recording()
                except Exception:
                    pass
                try:
                    cam.stop_preview()
                except Exception:
                    pass
                try:
                    cam.close()
                except Exception:
                    pass
        finally:
            try:
                if writer:
                    writer.close()
            except Exception:
                pass

            if serial_reader:
                serial_reader.close()

            # Ensure GPIO low and cleanup
            try:
                GPIO.output(PULSE_PIN, GPIO.LOW)
            except Exception:
                pass
            try:
                GPIO.cleanup()
            except Exception:
                pass

            log("Clean exit.")

if __name__ == "__main__":
    sys.exit(main())