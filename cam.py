from multiprocessing import Process, Value
import struct
import termios, tty
from picovoice import Picovoice
import pyaudio
import os
import threading

import signal
import pyaudio
import jetson.inference
import jetson.utils
import Jetson.GPIO as GPIO
import argparse
import common as cm
import cv2
import numpy as np
from PIL import Image
import time
import sys
import util as ut
from picovoice import Picovoice
import pyttsx3

engine=pyttsx3.init()
engine.setProperty('rate', 150)

access_key = "" # AccessKey obtained from Picovoice Console

auto=Value('b', False)
frame_width=Value('i', 1280)
frame_height= Value('i', 720)
tolerance_=Value('i', 200)

keyword_path = '/home/jetbot/hey-cam_en_jetson_v2_1_0.ppn'
context_path = '/home/jetbot/Downloads/switch_en_jetson_v2_1_0.rhn'

parser = argparse.ArgumentParser(description="Locate objects in a live camera stream using an object detection DNN.",
                                 formatter_class=argparse.RawTextHelpFormatter, epilog=jetson.inference.detectNet.Usage() +
                                 jetson.utils.videoSource.Usage() + jetson.utils.videoOutput.Usage() + jetson.utils.logUsage())

parser.add_argument("input_URI", type=str, default="", nargs='?', help="URI of the input stream")
parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")
parser.add_argument("--network", type=str, default="ssd-mobilenet-v2", help="pre-trained model to load (see below for options)")
parser.add_argument("--overlay", type=str, default="box,labels,conf", help="detection overlay flags (e.g. --overlay=box,labels,conf)\nvalid combinations are:  'box', 'labels', 'conf', 'none'")
parser.add_argument("--threshold", type=float, default=0.5, help="minimum detection threshold to use")

threshold=0.7

model_dir = '/home/jetbot/all_models/'
lbl = 'coco_labels.txt'

x_deviation=0
y_max=0

object_to_track='person'

print("init doing now")
ut.init_gpio()
print("init done")

def track_object(obj,labels, area_):

    global delay
    global x_deviation, y_max, tolerance, y_min
    global area
    area = area_
    flag=0
    x_min, y_min, x_max, y_max = list(obj.bbox)

    print('Left: ', x_min, 'Top: ', y_min, 'Right: ', x_max, 'Bottom: ', y_max )

    x_diff=x_max-x_min
    y_diff=y_max-y_min

    obj_x_center=x_min+(x_diff/2)
    obj_x_center=round(obj_x_center,3)

    obj_y_center=y_min+(y_diff/2)
    obj_y_center=round(obj_y_center,3)

    x_deviation=round((frame_width.value/2)-obj_x_center,3)
    y_max=round(y_max,3)

    print("{",x_deviation,y_max,"}")

    thread = threading.Thread(target = move_robot)
    thread.start()

def move_robot():
    global x_deviation, y_max, area
    tolerance=tolerance_.value
    y=frame_height.value-y_max

    if(abs(x_deviation)<tolerance and area < 350000):
        if(y_min < 150 ):
            ut.stop()
            print("reached person...........")
        else:
            ut.forward()
            print("moving robot ...FORWARD....!!!!!!!!!!!!!!")
    else:
        if(x_deviation>=tolerance):
            delay1=get_delay(x_deviation)
            ut.left()
            time.sleep(delay1)
            ut.stop()
            print("moving robot ...Left....<<<<<<<<<<")
        if(x_deviation<=-1*tolerance):
            delay1=get_delay(x_deviation)
            ut.right()
            time.sleep(delay1)
            ut.stop()
            print("moving robot ...Right....>>>>>>>>")
        if(area > 350000):
            ut.back()
            print("moving robot ...Back.......^^^^^^^^^")

def get_delay(deviation):

    deviation=abs(deviation)

    if(deviation>=0.4):
        d=0.080
    elif(deviation>=0.35 and deviation<0.40):
        d=0.060
    elif(deviation>=0.20 and deviation<0.35):
        d=0.050
    else:
        d=0.040
    return d

def following_loop(net, input_video, argv, opt=None):
    output = jetson.utils.videoOutput(opt.output_URI, argv=sys.argv)
    global auto

    text="Ok you can ask me to do things now Pat"
    engine.say(text)
    engine.runAndWait()

    while True:
        #----------------Capture Camera Frame-----------------
        # capture the next image
        img = input_video.Capture()

        # detect objects in the image (with overlay)
        detections = net.Detect(img, overlay=opt.overlay)

        for detection in detections:
            if net.GetClassDesc(detection.ClassID) == 'person':

                # print out performance info
                net.PrintProfilerTimes()
                objs = cm.get_output(detection)

                labels=cm.load_labels(model_dir+lbl)

                if auto.value:
                    track_object(objs,labels, detection.Area)#tracking  <<<<<<<

        # exit on input/output EOS
        if not input_video.IsStreaming() or not output.IsStreaming():
                break

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
    if inference.is_understood:
        if inference.intent == 'start':
            auto.value = True
        elif inference.intent == 'stop':
            auto.value = False
        elif inference.intent == 'zoom':
            ut.zoom()
        elif inference.intent == 'zoomout':
            ut.zoom_out()
        elif inference.intent == 'intro':
            text="Hi my name is cam and I am a robot that follows Patrick"
            engine.say(text)
            engine.runAndWait()
        elif inference.intent == 'fartnoises':
            file_ = '/home/jetbot/fart_sounds/thewholething.mp3'
            os.system("mpg123 " + file_)
        elif inference.intent == 'alongone':
            file_ = '/home/jetbot/fart_sounds/alongone.mp3'
            os.system("mpg123 " + file_)
        elif inference.intent == 'awetone':
            file_ = '/home/jetbot/fart_sounds/awetone.mp3'
            os.system("mpg123 " + file_)
        elif inference.intent == 'playusout':
            text="Make sure to hit that subscribe button and share this video with your friends so that Patrick doesn't kill me"
            engine.say(text)
            engine.runAndWait()
        elif inference.intent == 'tiktok':
            if frame_width.value == 720:
                frame_width.value = 1280
                tolerance_.value=120
            else:
                frame_width.value = 720
                tolerance_.value = 200

def controls(direction):

    while True:
        if auto.value:
            direction=getch()

            if direction == 'l':
                while direction == 'l':
                    direction=getch()
                    ut.left()
                    direction=''
            if direction == 'r':
                while direction == 'r':
                    direction=getch()
                    ut.right()
                    direction=''
            if direction == 'f':
                while direction == 'f':
                    direction=getch()
                    ut.forward()
                    direction=''
            if direction == 'b':
                while direction == 'b':
                    direction=getch()
                    ut.back()
                    direction=''
            stop()

def listen_for_voice():
    pv = Picovoice(
        access_key='',
        keyword_path=keyword_path,
        wake_word_callback=wake_word_callback,
        context_path=context_path,
        inference_callback=inference_callback
    )
    py_audio = pyaudio.PyAudio()

    audio_stream = py_audio.open(
        rate=pv.sample_rate,
        channels=1,
        format=pyaudio.paInt16,
        input=True,
        frames_per_buffer=pv.frame_length)

    while True:
        pcm = audio_stream.read(pv.frame_length)
        audio_frame = struct.unpack_from("h" * pv.frame_length, pcm)
        pv.process(audio_frame)

def use_controls():
    if not auto.value:
        direction=getch()
        controls(direction)

def handler(signum, frame):
    print("interupt detected")
    p.join()
    thread_eile.join()
    stop()
    GPIO.cleanup()
    exit(1)

def main():
    text="I'm just booting up give me a minute it takes me some time to get myself together"
    engine.say(text)
    engine.runAndWait()
    try:
        opt = parser.parse_known_args()[0]
    except:
        print("")
        parser.print_help()
        sys.exit(0)

    net = jetson.inference.detectNet('ssd-mobilenet-v2', sys.argv, 0.8)

    # create video sources
    input_video = jetson.utils.videoSource(opt.input_URI, argv=sys.argv)

    p = Process(target=listen_for_voice)
    thread_eile = threading.Thread(target=following_loop, args=(net, input_video, sys.argv, opt, ))
    signal.signal(signal.SIGINT, handler)
    p.start()
    thread_eile.start()
 
    thread_eile.join()
    p.join()

    ut.stop()
    GPIO.cleanup()

if __name__ == '__main__':
    main()
