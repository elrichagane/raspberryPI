#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Raspberry Pi:
- Record H.264 video with PiCamera
- Save per-frame timestamps to CSV
- Send 3 GPIO pulses at the start of recording
- Send 3 GPIO pulses at the end of recording

Usage:
    python record_cam_terminal.py <animal_id> <session_id>

Outputs:
    YYYYMMDD_HHMMSS_<animal_id>_<session_id>.h264
    YYYYMMDD_HHMMSS_<animal_id>_<session_id>_frame_timestamps.csv
"""

import csv
import datetime
import io
import sys
import time

import picamera
import RPi.GPIO as GPIO


# ----------------------------
# Configuration
# ----------------------------
RESOLUTION = (640, 480)
FRAMERATE = 30
RECORD_MAX_S = 24 * 60 * 60
ENABLE_PREVIEW = True

GPIO_MODE = GPIO.BOARD
PULSE_PIN = 8
PULSE_LEN_S = 0.005
PULSE_COUNT = 3
PULSE_GAP_S = 0.001


# ----------------------------
# Logging
# ----------------------------
def ts():
    return "%s | %.6f" % (
        datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"),
        time.time()
    )


def log(msg):
    sys.stdout.write("[%s] %s\n" % (ts(), msg))
    sys.stdout.flush()


# ----------------------------
# GPIO
# ----------------------------
def setup_gpio():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO_MODE)
    GPIO.setup(PULSE_PIN, GPIO.OUT, initial=GPIO.LOW)


def pulse_train(count=PULSE_COUNT):
    for _ in range(count):
        GPIO.output(PULSE_PIN, GPIO.HIGH)
        time.sleep(PULSE_LEN_S)
        GPIO.output(PULSE_PIN, GPIO.LOW)
        time.sleep(PULSE_GAP_S)


# ----------------------------
# PiCamera output wrapper
# ----------------------------
class FrameTimestampWriter(io.RawIOBase):
    def __init__(self, camera, video_filename, csv_filename):
        self.camera = camera
        self.video_output = open(video_filename, "wb")
        self.csv_file = open(csv_filename, "a", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.last_frame_index = None

        self.csv_writer.writerow([
            "frame_index",
            "frame_type",
            "frame_size",
            "video_timestamp_us",
            "wall_time_epoch_s",
            "wall_time_iso"
        ])
        self.csv_file.flush()

    def write(self, buf):
        n = self.video_output.write(buf)

        frame = self.camera.frame
        if frame.complete and frame.index != self.last_frame_index:
            self.last_frame_index = frame.index
            self.csv_writer.writerow([
                frame.index,
                str(frame.frame_type),
                frame.frame_size,
                frame.timestamp,
                "%.6f" % time.time(),
                datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
            ])
            self.csv_file.flush()

        return n

    def close(self):
        try:
            if not self.video_output.closed:
                self.video_output.close()
        except Exception:
            pass

        try:
            if not self.csv_file.closed:
                self.csv_file.close()
        except Exception:
            pass

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
    csv_fn = base + "_frame_timestamps.csv"

    cam = None
    writer = None
    recording_started = False

    setup_gpio()

    try:
        log("Saving video: %s" % video_fn)
        log("Saving frame timestamps: %s" % csv_fn)

        cam = picamera.PiCamera(clock_mode='raw')
        cam.resolution = RESOLUTION
        cam.framerate = FRAMERATE

        if ENABLE_PREVIEW:
            cam.start_preview(fullscreen=False, window=(0, 0, RESOLUTION[0], RESOLUTION[1]))

        writer = FrameTimestampWriter(cam, video_fn, csv_fn)

        log("Sending start pulse train")
        pulse_train()

        cam.start_recording(writer, format="h264")
        recording_started = True

        log("Recording started. Ctrl+C to stop.")
        cam.wait_recording(RECORD_MAX_S)
        return 0

    except KeyboardInterrupt:
        log("Stopping (KeyboardInterrupt).")
        return 0

    finally:
        try:
            if cam and recording_started:
                try:
                    cam.stop_recording()
                except Exception:
                    pass

                try:
                    log("Sending end pulse train")
                    pulse_train()
                except Exception:
                    pass

            if cam:
                try:
                    if ENABLE_PREVIEW:
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
                pass
            except Exception:
                pass

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