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
signal.signal(signal.SIGTERM, signal.default_int_handler)

os.chdir(os.path.dirname(__file__))

PORT_CAMERA = 45678
PORT_CONTROL = 56789

DEFAULT_POWER = 0.5

LED_BLUE = (0, 0, 1)
LED_GREEN = (0, 1, 0)
LED_RED = (1, 0, 0)



class CameraThread(threading.Thread):
    def __init__(self, ready):
        super().__init__()
        self.ready = ready
        self.isAlive = True
        
        
    def run(self):
        with picamera.PiCamera(resolution=(900,400), framerate=24) as camera:
            while self.isAlive:
                try:
                    self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    with self.socket as s:
                        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                        s.bind(('', PORT_CAMERA))
                        print('Listening for Camera...')
                        s.listen()
                        self.ready.set()
                        connection, addr = s.accept()
                        with connection.makefile('wb') as output:
                            print('Connected for camera by', addr)
                            camera.start_recording(output, format='mjpeg')
                            while self.isAlive:
                                camera.wait_recording(0.1)                            
                    
                except:
                    print("camera error type: ", sys.exc_info()[0])
                    print("camera error value: ", sys.exc_info()[1])
                    
                try:
                    camera.stop_recording()
                except:
                    pass
                
        
    def stop(self):
        self.isAlive = False
        try:
            self.socket.shutdown(socket.SHUT_RDWR)
        except:
            pass


def stop():
    motorL.stop()
    motorR.stop()
    
def goForward():
    motorL.forward(power)
    motorR.forward(power)
        
def goBackward():
    motorL.backward(power)
    motorR.backward(power)
    
def turnRight():
    motorL.forward(power)
    motorR.stop()
    
def turnLeft():
    motorL.stop()
    motorR.forward(power)
    
def rotateClockwise():
    motorL.forward(power)
    motorR.backward(power)

def rotateAntiClockwise():
    motorL.backward(power)
    motorR.forward(power)

class DistanceThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.isObstacleClose = False
        self.isAlive = True
        
    def run(self):
        with DistanceSensor(26,19, max_distance=4.5) as sensor:
            while self.isAlive:
                if(sensor.distance < 0.25):
                    if not self.isObstacleClose:
                        stop()
                        self.isObstacleClose = True
                        led.color = LED_RED
                else:
                    self.isObstacleClose = False
                    led.color = LED_GREEN
                
    def stop(self):
        self.isAlive = False
        
            
isOn = True
with RGBLED(22, 27, 10, False) as led,\
     Motor(forward=2, backward=3) as motorL,\
     Motor(forward=4, backward=17) as motorR:
    
    restrictedCommands = {b'ST' : stop,
                          b'GB' : goBackward,
                          b'RC' : rotateClockwise,
                          b'RA' : rotateAntiClockwise}
    
    commands = {**restrictedCommands,
                b'GF' : goForward,
                b'TR' : turnRight,
                b'TL' : turnLeft}
         
    while isOn:
        try:    
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.bind(('', PORT_CONTROL))
                print('Listening for controller...')
                led.color = LED_BLUE
                s.listen()
                conn, addr = s.accept()
                with conn:
                    print('Connected for control by', addr)
                    distanceSystem = DistanceThread()
                    distanceSystem.start()
                    cameraThreadReady = threading.Event()
                    cameraSystem = CameraThread(cameraThreadReady)
                    cameraSystem.start()
                    cameraThreadReady.wait()
                    power = DEFAULT_POWER
                    conn.send(bytes([int(power*100)]))
                    while True:
                        data = conn.recv(2)
                        print(data)
                        if (not data):
                            print("socket closed!")
                            break
                        elif data == b'SD':
                            isOn = False
                            break
                        elif data[0:1] == b'P':
                            newPower = data[1]/100
                            if 0 <= newPower <= 1:
                                print("Setting power to {}".format(newPower))
                                power = newPower
                            else:
                                print("Invalid power value, resetting power to default")
                                power = DEFAULT_POWER
                        else:
                            if data in (restrictedCommands if distanceSystem.isObstacleClose else commands):
                                commands[data]()
                            else:
                                print("Illegal Command Received!, Stopping motors!")
                                stop()
        except:
            if sys.exc_info()[0] is not KeyboardInterrupt:
                print("Unexpected error: ", sys.exc_info()[0])
            isOn = False
        
        led.off()
        stop()
        try:
            cameraSystem.stop()
            cameraSystem.join()
            print("Camera system stopped!")
            distanceSystem.stop()
            distanceSystem.join()
            print("Distance system stopped!")
        except:
            pass
        sleep(1)
        
print('Shutting down!')