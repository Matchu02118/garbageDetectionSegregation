""" 
------------------------------------------- MULTITHREAD -------------------------------------------
Title: AI Camera-Integrated Waste Collection and Segregation System Using Raspberry Pi 

Researchers:
 - Karl Russell Dino
 - Carl Mathew Estorga
  
Features: 
    - Collection & Segregation mechanism (Servo Control)
    - Auto Navigation & Garbage Distance calculation (DC Motor control & Ultrasonic Reading)
    - Storage checker (Ultrasonic Reading)
---------------------------------------------------------------------------------------------------     
"""
import warnings
import threading
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
    
    timeout = time.time() + 1
    while GPIO.input(redECHO) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            return None  # No response from sensor

    while GPIO.input(redECHO) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            return None  # Echo signal lasted too long
            
    """ while GPIO.input(redECHO) == 0:
        pulse_start = time.time()
    
    while GPIO.input(redECHO) == 1:
        pulse_end = time.time() """
    
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # convert to cm
    return round(distance, 2)

def grnUltra():
    GPIO.output(grnTRIG, GPIO.LOW)
    time.sleep(0.000002)
    GPIO.output(grnTRIG, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(grnTRIG, GPIO.LOW)
    
    timeout = time.time() + 1
    while GPIO.input(grnECHO) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            return None  # No response from sensor

    while GPIO.input(grnECHO) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            return None  # Echo signal lasted too long
            
    """ while GPIO.input(grnECHO) == 0:
        pulse_start = time.time()
    
    while GPIO.input(grnECHO) == 1:
        pulse_end = time.time() """
 
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # convert to cm
    return round(distance, 2)

def bluUltra():
    GPIO.output(bluTRIG, GPIO.LOW)
    time.sleep(0.000002)
    GPIO.output(bluTRIG, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(bluTRIG, GPIO.LOW)
    
    timeout = time.time() + 1
    while GPIO.input(bluECHO) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            return None  # No response from sensor

    while GPIO.input(bluECHO) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            return None  # Echo signal lasted too long
            
    """ while GPIO.input(bluECHO) == 0:
        pulse_start = time.time()
    
    while GPIO.input(bluECHO) == 1:
        pulse_end = time.time() """
    
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
    if distance > 25:  # No obstacle
        move_forward()
    elif 0 < distance <= 25:  # Obstacle detected
        stop_motors()
        time.sleep(0.5)
        move_backward()
        time.sleep(1.5)
        stop_motors()
        time.sleep(0.5)
        move_left()
        time.sleep(3)
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

# Function to handle ultrasonic sensor readings in a thread
def ultrasonic_thread():
    global distance_to_object, redFull, grnFull, bluFull
    while True:
        distance_to_object = measuringUltra()
        redFull = redUltra()
        grnFull = grnUltra()
        bluFull = bluUltra()       
        time.sleep(0.1)

# Function to handle camera processing in a thread
def camera_thread():
    global collecting
    cam = cv2.VideoCapture(0)
    if not cam.isOpened():
        print("Error: Could not open webcam.")
        GPIO.cleanup()
        return
    
    while True:
        ret, frame = cam.read()
        if not ret or frame is None:
            continue
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        results_a = model_a(frame_rgb)
        results_b = model_b(frame_rgb)
        
        process_detections(results_a, model_a, class_to_message_a)
        process_detections(results_b, model_b, class_to_message_b)
        
        cv2.imshow("Preview Window", frame_rgb)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cam.release()
    cv2.destroyAllWindows()

# Function to process YOLO detections
def process_detections(results, model, class_to_message):
    global collecting, distance_to_object
    detections = results.xyxy[0].cpu().numpy() if results.xyxy[0].shape[0] > 0 else []
    for *box, confidence, classId in detections:
        if confidence < 0.80:
            continue
        class_name = model.names[int(classId)]
        
        if class_name in class_to_message:
            collecting = True
            attempts = 0
            max_attempts = 3
            stop_motors()
            category = class_to_message[class_name]
            print(category)

            while collecting and attempts < max_attempts:
                if distance_to_object is not None and 35 < distance_to_object <= 37.5:
                    if "Non-Biodegradable" in category:
                        segregate_waste(class_to_message[class_name])
                    elif "Biodegradable" in category:
                        segregate_waste(class_to_message[class_name])
                    elif "Recyclable" in category:
                        segregate_waste(class_to_message[class_name])

                    time.sleep(3)
                    collectGarbage()
                    collecting = False  # Collection is complete
                    obstacleAvoidance(distance_to_object)
                      
                elif distance_to_object is not None and distance_to_object < 30:
                    move_backward()
                elif distance_to_object is not None and distance_to_object > 40:
                    move_forward()
                
                attempts += 1

# Function to handle waste segregation
def segregate_waste(category):
    if "Non-Biodegradable" in category:
        segregation_servo(88)
    elif "Biodegradable" in category:
        segregation_servo(180)
    elif "Recyclable" in category:
        segregation_servo(0)
        
    collectGarbage()
    global collecting
    collecting = False

# Load YOLO models
modelPathA = "/home/george123/Dataset/garbage/A.pt"
modelPathB = "/home/george123/Dataset/garbage/B.pt"
model_a = torch.hub.load('ultralytics/yolov5', 'custom', path=modelPathA)
model_b = torch.hub.load('ultralytics/yolov5', 'custom', path=modelPathB)

distance_to_object = None
redFull = None
grnFull = None
bluFull = None
collecting = False

# Start threads
ultrasonic_thread = threading.Thread(target=ultrasonic_thread, daemon=True)
camera_thread = threading.Thread(target=camera_thread, daemon=True)

ultrasonic_thread.start()
camera_thread.start()

# Main loop to handle robot movement
try:
    while True:
        if distance_to_object is not None and not collecting:
            obstacleAvoidance(distance_to_object)     
        time.sleep(0.1)
                
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
except KeyboardInterrupt:
    print("Program cancelled....")
finally:
    print("Exiting and cleaning up resources.")
    GPIO.cleanup()
    exit(0)
