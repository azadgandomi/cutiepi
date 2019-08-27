import socket
import threading
import os
import sys
import signal
import picamera
from time import sleep
from gpiozero import RGBLED
from gpiozero import DistanceSensor
from gpiozero import Motor

signal.signal(signal.SIGINT, signal.default_int_handler)

os.chdir(os.path.dirname(__file__))

PORT_CAMERA = 45678
PORT_CONTROL = 56789
SPEED = 0.5

blue = (0, 0, 1)
green = (0, 1, 0)
red = (1, 0, 0)

class CameraThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.isAlive = True
        
    def run(self):
        with picamera.PiCamera(resolution=(480,360), framerate=24) as camera:
            while self.isAlive:
                try:
                    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                        s.bind(('', PORT_CAMERA))
                        print('Listening for Camera...')
                        s.listen()
                        connection, addr = s.accept()
                        with connection.makefile('wb') as output:
                            print('Connected for camera by', addr)
                            camera.start_recording(output, format='h264', profile='main')
                            while self.isAlive:
                                camera.wait_recording(0.1)
                            
                except:
                    print("Unexpected camera error type: ", sys.exc_info()[0])
                    print("Unexpected camera error value: ", sys.exc_info()[1])
                    
                try:
                    camera.stop_recording()
                except:
                    print("Exception when stopping recording: ", sys.exc_info()[1])
                sleep(1)
        
    def stop(self):
        self.isAlive = False
            


def stop():
    motorL.stop()
    motorR.stop()
    
def goForward():
    motorL.forward(SPEED)
    motorR.forward(SPEED)
        
def goBackward():
    motorL.backward(SPEED)
    motorR.backward(SPEED)
    
def turnRight():
    motorR.stop()
    motorL.forward(SPEED)
    
def turnLeft():
    motorL.stop()
    motorR.forward(SPEED)
    
def rotateClockwise():
    motorL.forward(SPEED)
    motorR.backward(SPEED)

def rotateAntiClockwise():
    motorL.backward(SPEED)
    motorR.forward(SPEED)

class DistanceThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.isObstacleClose = False
        self.isAlive = True
        self.color = (0, 0, 0)
        
    def run(self):
        with DistanceSensor(26,19, max_distance=4.5) as sensor:
            while self.isAlive:
                if(sensor.distance < 0.25):
                    if not self.isObstacleClose:
                        stop()
                        self.isObstacleClose = True
                        led.color = red
                else:
                    self.isObstacleClose = False
                    led.color = self.color
                    
                sleep(0.05)
                
    def stop(self):
        self.isAlive = False
        
            
isOn = True
with RGBLED(22, 27, 10, False) as led,\
     Motor(forward=2, backward=3) as motorL,\
     Motor(forward=4, backward=17) as motorR:
    distanceSystem = DistanceThread()
    distanceSystem.start()
    cameraSystem = CameraThread()
    cameraSystem.start()
    
    commands = {b'S' : stop,
                b'GF' : goForward,
                b'GB' : goBackward,
                b'TR' : turnRight,
                b'TL' : turnLeft,
                b'RC' : rotateClockwise,
                b'RA' : rotateAntiClockwise}
         
    while isOn:
        try:    
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.bind(('', PORT_CONTROL))
                print('Listening for control...')
                distanceSystem.color = blue
                s.listen()
                conn, addr = s.accept()
                distanceSystem.color = green
                with conn:
                    print('Connected for control by', addr)
                    while True:
                        data = conn.recv(1024)
                        print(data)
                        if (not data):
                            print("socket closed!")
                            break
                        elif data == b'shutdown':
                            isOn = False
                            break
                        elif distanceSystem.isObstacleClose:
                            if data == b'GB':
                                goBackward()
                            elif data == b'S':
                                stop()
                            continue
                        else:
                            commands[data]()
        except:
            print("Unexpected unkown error: ", sys.exc_info()[0])
            isOn = False
        sleep(1)
        
    cameraSystem.stop()
    cameraSystem.join()
    distanceSystem.stop()
    distanceSystem.join()
    stop()
    led.off()
print('Done')