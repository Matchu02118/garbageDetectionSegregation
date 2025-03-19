#! /usr/bin/env python3

import warnings
import cv2
import torch
import numpy as np
import RPi.GPIO as GPIO
import time

# Pin Connections
GPIO.setmode(GPIO.BCM)
segregatorPin = 17 # Pin 11
armL = 2 # Pin 3
armR = 3 # Pin 5
collectorL = 14 # Pin 8
collectorR = 15 # Pin 10

obstaTRIG = 22 # Pin 15
obstaECHO = 27 # Pin 13

redTRIG = 5 #  Pin 29
redECHO = 6 #  Pin 31

bluTRIG = 9  # Pin 21
bluECHO = 10 # Pin 19

grnTRIG = 0 # Pin 27 
grnECHO = 1 # Pin 28

redLED = 7 # Pin 26
grnLED = 8 # Pin 24
bluLED = 21 # Pin 40

# DC Motors
Mla = 23 # Pin 16
Mlb = 24 # Pin 18
MRa = 25 # Pin 22
MRb = 26 # Pin 37

# GPIO Setup
GPIO.setup(segregatorPin, GPIO.OUT)
GPIO.setup(armL, GPIO.OUT)
GPIO.setup(armR, GPIO.OUT)
GPIO.setup(collectorL, GPIO.OUT)
GPIO.setup(collectorR, GPIO.OUT)

GPIO.setup(obstaTRIG, GPIO.OUT)
GPIO.setup(obstaECHO, GPIO.IN)
GPIO.setup(bluTRIG, GPIO.OUT)
GPIO.setup(bluECHO, GPIO.IN)
GPIO.setup(grnTRIG, GPIO.OUT)
GPIO.setup(grnECHO, GPIO.IN)
GPIO.setup(redTRIG, GPIO.OUT)
GPIO.setup(redECHO, GPIO.IN)

GPIO.setup(redLED, GPIO.OUT)
GPIO.setup(bluLED, GPIO.OUT)
GPIO.setup(grnLED, GPIO.OUT)

GPIO.setup(Mla, GPIO.OUT)
GPIO.setup(Mlb, GPIO.OUT)
GPIO.setup(MRa, GPIO.OUT)
GPIO.setup(MRb, GPIO.OUT)

# Setup servo motors
servo = GPIO.PWM(segregatorPin, 50)
servo1 = GPIO.PWM(armL, 50)
servo2 = GPIO.PWM(armR, 50)
servo3 = GPIO.PWM (collectorL, 50)
servo4 = GPIO.PWM (collectorR, 50)

servo.start(0)
servo1.start(0)
servo2.start(0)
servo3.start(0)
servo4.start(0)

warnings.filterwarnings("ignore", category=FutureWarning, module="torch")
warnings.filterwarnings("ignore", module=".*common.py.*")
warnings.filterwarnings("ignore", message=".*autocast.*", category=FutureWarning)

# Segregation Servo Motor
def segregation_servo(angle):
    duty_cycle = 2 + (angle / 18)
    GPIO.output(segregatorPin, GPIO.HIGH)
    servo.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)
    GPIO.output(segregatorPin, GPIO.LOW)
    servo.ChangeDutyCycle(0)

# Arm Control Function
def arm_servo(servo_pwm, angle):
    duty_cycle = 2 + (angle / 18)
    servo_pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)
    servo_pwm.ChangeDutyCycle(0)
    
# Collector Control Function
def collector_servo(collect_pwm, angle):
    duty_cycle = 2 + (angle / 18)
    collect_pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)
    collect_pwm.ChangeDutyCycle(0)
    
# HC-SR04 for obstacle avoidance / garbage distance measuring 
def measuringUltra():
    GPIO.output(obstaTRIG, GPIO.LOW)
    time.sleep(0.000002)
    GPIO.output(obstaTRIG, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(obstaTRIG, GPIO.LOW)
    
    timeout = time.time() + 1
    while GPIO.input(obstaECHO) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            return None  # No response from sensor

    while GPIO.input(obstaECHO) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            return None  # Echo signal lasted too long

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150 # convert to cm
    return round(distance, 2)

def redUltra():
    GPIO.output(redTRIG, GPIO.LOW)
    time.sleep(0.000002)
    GPIO.output(redTRIG, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(redTRIG, GPIO.LOW)
    
    """ timeout = time.time() + 1
    while GPIO.input(redECHO) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            return None  # No response from sensor

    while GPIO.input(redECHO) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            return None  # Echo signal lasted too long """
            
    while GPIO.input(redECHO) == 0:
        pulse_start = time.time()
    
    while GPIO.input(redECHO) == 1:
        pulse_end = time.time()
    
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # convert to cm
    return round(distance, 2)

def grnUltra():
    GPIO.output(grnTRIG, GPIO.LOW)
    time.sleep(0.000002)
    GPIO.output(grnTRIG, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(grnTRIG, GPIO.LOW)
    
    """ timeout = time.time() + 1
    while GPIO.input(grnECHO) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            return None  # No response from sensor

    while GPIO.input(grnECHO) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            return None  # Echo signal lasted too long """
            
    while GPIO.input(grnECHO) == 0:
        pulse_start = time.time()
    
    while GPIO.input(grnECHO) == 1:
        pulse_end = time.time()
 
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # convert to cm
    return round(distance, 2)

def bluUltra():
    GPIO.output(bluTRIG, GPIO.LOW)
    time.sleep(0.000002)
    GPIO.output(bluTRIG, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(bluTRIG, GPIO.LOW)
    
    """ timeout = time.time() + 1
    while GPIO.input(bluECHO) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            return None  # No response from sensor

    while GPIO.input(bluECHO) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            return None  # Echo signal lasted too long """
            
    while GPIO.input(bluECHO) == 0:
        pulse_start = time.time()
    
    while GPIO.input(bluECHO) == 1:
        pulse_end = time.time()
    
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # convert to cm
    return round(distance, 2)

# Motor codes
def stop_motors():
    GPIO.output(Mla, GPIO.LOW)
    GPIO.output(Mlb, GPIO.LOW)
    GPIO.output(MRa, GPIO.LOW)
    GPIO.output(MRb, GPIO.LOW)

def move_forward():
    GPIO.output(Mla, GPIO.HIGH)
    GPIO.output(Mlb, GPIO.LOW)
    GPIO.output(MRa, GPIO.HIGH)
    GPIO.output(MRb, GPIO.LOW)

def move_backward():
    GPIO.output(Mla, GPIO.LOW)
    GPIO.output(Mlb, GPIO.HIGH)
    GPIO.output(MRa, GPIO.LOW)
    GPIO.output(MRb, GPIO.HIGH)

def move_left():
    GPIO.output(Mla, GPIO.LOW)
    GPIO.output(Mlb, GPIO.LOW)
    GPIO.output(MRa, GPIO.HIGH)
    GPIO.output(MRb, GPIO.LOW)

def move_right():
    GPIO.output(Mla, GPIO.HIGH)
    GPIO.output(Mlb, GPIO.LOW)
    GPIO.output(MRa, GPIO.LOW)
    GPIO.output(MRb, GPIO.LOW)

# Obstacle Avoidance (Shoutout to CpE 411.1)
def obstacleAvoidance(distance):
    global collecting
    if collecting:
        return
    print(f"Obstacle Distance: {distance} cm")
    if distance > 25:  # Clear path
        move_forward()

    elif 0 < distance <= 25:  # Obstacle detected
        stop_motors()
        time.sleep(1)
        move_backward()
        time.sleep(2)
        move_left()
        time.sleep(2)
        stop_motors()

# Garbage Collection & Segregation Functions
def collectGarbage():
    arm_servo(servo1, 150)
    arm_servo(servo2, 90)
    time.sleep(3)                       
    collector_servo(servo3, 125)
    collector_servo(servo4, 45)
    time.sleep(2)                      
    arm_servo(servo1, 0)
    arm_servo(servo2, 180)
    time.sleep(2)  
    collector_servo(servo3, 180)
    collector_servo(servo4, 0)
    time.sleep(2)

def resetAngle():
    arm_servo(servo1, 0)
    arm_servo(servo2, 170)
    collector_servo(servo3, 180)
    collector_servo(servo4, 0)
    time.sleep(2)
    servo1.stop()
    servo2.stop()
    servo3.stop()
    servo4.stop()

# YOLOv5 Model Paths
modelPathA = "/home/george123/Dataset/garbage/A.pt"
modelPathB = "/home/george123/Dataset/garbage/B.pt"

try:
    model_a = torch.hub.load('ultralytics/yolov5', 'custom', path=modelPathA)
    model_b = torch.hub.load('ultralytics/yolov5', 'custom', path=modelPathB)
except Exception as e:
    print(f"Error loading YOLOv5 models: {e}")
    exit(-1)

# Segregation of Items
class_to_message_a = {
    'can': "Segregate to Recyclable",
    'food-wrapper': "Segregate to Non-Biodegradable",
    'paper': "Segregate to Biodegradable",
    'plastic bottle': "Segregate to Recyclable"
}

class_to_message_b = {
    'BIODEGRADABLE': "Segregate to Biodegradable",
    'CARDBOARD': "Segregate to Recyclable",
    'GLASS': "Segregate to Non-Biodegradable",
    'METAL': "Segregate to Recyclable",
    'PAPER': "Segregate to Biodegradable",
    'PLASTIC': "Segregate to Non-Biodegradable",
}

# Confidence threshold
threshold = 0.75

# Start Camera
cam = cv2.VideoCapture(0)
width = 480
height = 320 
cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

if not cam.isOpened():
    print("Error: Could not open webcam.")
    GPIO.cleanup()
    exit(-1)
    
time.sleep(1)

# OpenCV Window
cv2.namedWindow("Preview Window", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Preview Window", 480, 320)

# Collecting Status
collecting = False

# Main Loop
try:
    while True:
        # Ultrasonic Sensor readings
        distance_to_object = measuringUltra()
        redFull = redUltra()
        grnFull = grnUltra()
        bluFull = bluUltra()
        
        # Storage Checking
        if bluFull <= 8:
            GPIO.output(bluLED, GPIO.HIGH)
        else:
            GPIO.output(bluLED, GPIO.LOW)
        if grnFull <= 8:
            GPIO.output(grnLED, GPIO.HIGH)
        else:
            GPIO.output(grnLED, GPIO.LOW)
        if redFull <= 8:
            GPIO.output(redLED, GPIO.HIGH)
        else:
            GPIO.output(redLED, GPIO.LOW)
        
        # Start Navigation
        obstacleAvoidance(distance_to_object)
        
        # Capture frame 
        ret, frame = cam.read()

        if not ret or frame is None:
            print("Error: Failed to capture from webcam")
            continue
        
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Detect objects with both models
        results_a = model_a(frame_rgb)
        results_b = model_b(frame_rgb)

        # Process detections from Model A
        detections_a = results_a.xyxy[0].cpu().numpy() if results_a.xyxy[0].shape[0] > 0 else []
        for *box, confidence, classId in detections_a:
            if confidence < threshold:
                continue
            x1, y1, x2, y2 = map(int, box)
            class_name = model_a.names[int(classId)]
            label = f"{class_name} {confidence:.2f}"
            cv2.rectangle(frame_rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green for Model A
            cv2.putText(frame_rgb, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Print segregation message for Model A
            if class_name in class_to_message_a:
                collecting = True
                print(f"Garbage Distance: {distance_to_object} cm")
                stop_motors()
                category = class_to_message_a[class_name]
                print(category)
                attempts = 0
                max_attempts = 2
                while collecting and attempts < max_attempts:
                    if 35 < distance_to_object <= 37.5:
                        if "Non-Biodegradable" in category:
                            segregation_servo(88)
                            time.sleep(3)
                            collectGarbage()                        
                        elif "Biodegradable" in category:
                            segregation_servo(180)
                            time.sleep(3)
                            collectGarbage()
                        elif "Recyclable" in category:
                            segregation_servo(0)
                            time.sleep(3)
                            collectGarbage()
                        collecting = False         
                    elif distance_to_object < 30:
                        move_backward()
                    elif distance_to_object > 40:
                        move_forward()
                        
                    attempts+=1
                    
                    if attempts >= max_attempts:
                        collecting=False
                        obstacleAvoidance(distance_to_object)                    
                        break
                            
        # Process detections from Model B
        detections_b = results_b.xyxy[0].cpu().numpy() if results_b.xyxy[0].shape[0] > 0 else []
        for *box, confidence, classId in detections_b:
            if confidence < threshold:
                continue
            x1, y1, x2, y2 = map(int, box)
            class_name = model_b.names[int(classId)]

            # Skip "PAPER" and "PLASTIC" detections from Model B
            if class_name == "PAPER" or class_name == "PLASTIC":
               print(f"Skipping {class_name} detection.")
               continue
           
            label = f"{class_name} {confidence:.2f}"
            cv2.rectangle(frame_rgb, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Blue for Model B
            cv2.putText(frame_rgb, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            # Print segregation message for Model B
            if class_name in class_to_message_b:
                collecting = True
                print(f"Garbage Distance: {distance_to_object} cm")
                stop_motors()
                category = class_to_message_b[class_name]
                print(category)
                attempts = 0
                max_attempts = 2
                while collecting and attempts < max_attempts:
                    if 35 < distance_to_object <= 37.5:
                        if "Non-Biodegradable" in category:
                            segregation_servo(88)                    
                            time.sleep(3)
                            collectGarbage()
                        elif "Biodegradable" in category:
                            segregation_servo(180)
                            time.sleep(3)
                            collectGarbage()
                        elif "Recyclable" in category:
                            segregation_servo(0)
                            time.sleep(3)
                            collectGarbage()
                        collecting = False
                    elif distance_to_object < 30:
                        move_backward()
                    elif distance_to_object > 40:
                        move_forward()
                        
                    attempts+=1
                    
                    if attempts >= max_attempts:
                        collecting=False
                        obstacleAvoidance(distance_to_object)                    
                        break
            
        cv2.imshow("Preview Window", frame_rgb)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
               
# Emergency Shutdown (Ctrl+C) 
except KeyboardInterrupt:
    print("Program cancelled....")
finally:
    print("Exiting and cleaning up resources.")
    cam.release()
    cv2.destroyAllWindows()
    resetAngle()
    segregation_servo(88)
    GPIO.cleanup()
    exit(0)
    