import struct
import termios, tty
import sys
import Jetson.GPIO as GPIO
from picovoice import Picovoice
import pyaudio
import os, time
import threading
import signal

access_key = "" # AccessKey obtained from Picovoice Console

keyword_path = '/home/jetbot/Downloads/computer_en_jetson_v2_0_0.ppn'
context_path = '/home/jetbot/Downloads/switch_en_jetson_v2_0_0.rhn'
GPIO.setwarnings(False)

auto=False

left_en = 21
right_en = 20

motorLeft = 16
motorRight = 19

pin_1 = 25
pin_2 = 8

def init_gpio():
    print("in init func now")
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(motorLeft,GPIO.OUT)
    GPIO.setup(motorRight,GPIO.OUT)
    GPIO.setup(left_en,GPIO.OUT)
    GPIO.setup(right_en,GPIO.OUT)
    GPIO.setup(pin_1, GPIO.OUT)
    GPIO.setup(pin_2, GPIO.OUT)

    GPIO.output(pin_1, False)
    GPIO.output(pin_2, False)
    GPIO.output(motorLeft, False)
    GPIO.output(motorRight, False)
    GPIO.output(left_en, False)
    GPIO.output(right_en, False)

def back():
    print("moving back!!!!!!")
    GPIO.output(motorLeft, False)
    GPIO.output(left_en, True)

    GPIO.output(motorRight, False)
    GPIO.output(right_en, True)

def right():
    GPIO.output(motorLeft, False)
    GPIO.output(left_en, True)

def left():
    GPIO.output(motorLeft, True)
    GPIO.output(left_en, True)

def forward():
    GPIO.output(motorLeft, True)
    GPIO.output(left_en, True)

    GPIO.output(right_en, True)
    GPIO.output(motorRight, True)

def zoom():
    GPIO.output(pin_2, True)
    time.sleep(2.5)
    GPIO.output(pin_2, False)

def zoom_out():
    GPIO.output(pin_1, True)
    time.sleep(2.5)
    GPIO.output(pin_1, False)

def stop():
    GPIO.output(left_en, False)
    GPIO.output(right_en, False)

#Define a function to get the pressed key
def getch():
    #Get standard input file descriptor
    fd = sys.stdin.fileno()
    #Get terminal attributes of file descriptor
    old_settings = termios.tcgetattr(fd)
    #Make it unnecessary to press the enter key
    tty.setcbreak(fd)
    #Receive characters typed on the keyboard
    ch = sys.stdin.read(1)
    #Restore the terminal attributes of the file descriptor
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    #ch(character =In this case the pressed key)return it
    return ch

def wake_word_callback():
    print("[wake word]")

def inference_callback(inference):
    global auto

    if inference.is_understood:
        if inference.intent == 'start':
            auto = True
        elif inference.intent == 'stop':
            auto = False

def controls(direction):
    while auto:
        if auto:
            direction=getch()
            print(direction)
        else:
            stop()
            return
        if direction == 'l':
            left()
        if direction == 'r':
            right()
        if direction == 'f':
            forward()
        if direction == 'b':
            back()
        direction=''
        time.sleep(2)
        stop()
    stop()

def listen_for_voice():
    print("inside listen for audio")
    while True:
        pcm = audio_stream.read(pv.frame_length)
        audio_frame = struct.unpack_from("h" * pv.frame_length, pcm)
        pv.process(audio_frame)

def use_controls(opt=None):
    while True:
        try:
            if auto:
                print("testsing")
                direction=getch()
                controls(direction)
        # ctrl+c catch here
        except KeyboardInterrupt:
            break

def handler(signum, frame):
    print("interupt detected")
    thread.join()
    thread_eile.join()
    stop()
    GPIO.cleanup()
    exit(1)
