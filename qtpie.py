import socket
import threading
import os
import sys
import signal
import camera_server
from time import sleep
from gpiozero import RGBLED
from gpiozero import DistanceSensor
from gpiozero import Motor

signal.signal(signal.SIGINT, signal.default_int_handler)

os.chdir(os.path.dirname(__file__))

PORT_CAMERA = 8082
PORT_CONTROL = 56789
SPEED = 0.5

blue = (0, 0, 1)
green = (0, 1, 0)
red = (1, 0, 0)

class CameraThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.event = threading.Event()
        
    def run(self):
        camera_server.start(self.event)
            


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
        self.stopped = False
        self.color = (0, 0, 0)
        
    def run(self):
        with DistanceSensor(26,19, max_distance=4.5) as sensor:
            while(not self.stopped):
                if(sensor.distance < 0.25):
                    if not self.isObstacleClose:
                        stop()
                        self.isObstacleClose = True
                        led.color = red
                else:
                    self.isObstacleClose = False
                    led.color = self.color
                    
                sleep(0.05)
            
            
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
        except OSError as e:
            print("Unexpected error: ", e)
        except:
            print("Unexpected error: ", sys.exc_info()[0])
            break;
        sleep(1)
    distanceSystem.stopped = True
    cameraSystem.event.set()
    distanceSystem.join()
    cameraSystem.join()
    stop()
    led.off()
print('Done')