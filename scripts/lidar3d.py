#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

#this code was taken from
#https://docs.sunfounder.com/projects/davinci-kit/en/latest/1.3.2_servo.html


SERVO_MIN_PULSE = 500
SERVO_MAX_PULSE = 2500

ServoPin = 24

def map(value, inMin, inMax, outMin, outMax):
    return (outMax - outMin) * (value - inMin) / (inMax - inMin) + outMin

def setup():
    global p
    GPIO.setmode(GPIO.BCM)       # Numbers GPIOs by BCM
    GPIO.setup(ServoPin, GPIO.OUT)   # Set ServoPin's mode is output
    GPIO.output(ServoPin, GPIO.LOW)  # Set ServoPin to low
    p = GPIO.PWM(ServoPin, 50)     # set Frequecy to 50Hz
    p.start(0)                     # Duty Cycle = 0

def setAngle(angle):      # make the servo rotate to specific angle (0-180 degrees)
    angle = max(0, min(270, angle))
    pulse_width = map(angle, 0, 270, SERVO_MIN_PULSE, SERVO_MAX_PULSE)
    pwm = map(pulse_width, 0, 20000, 0, 100)
    print("angle: "+ str(angle)+ "pwm: "+str(pwm))
    p.ChangeDutyCycle(pwm)#map the angle to duty cycle and output it

def loop():
    
    while True:
        #setAngle(10)
        #time.sleep(1)
        for i in range(200, 241, 1):   #make servo rotate from 0 to 180 deg
            setAngle(i)     # Write to servo
            time.sleep(0.001)
        time.sleep(1)
        for i in range(240, 199, -1): #make servo rotate from 180 to 0 deg
            setAngle(i)
            time.sleep(0.001)
        time.sleep(1)

def destroy():
    p.stop()
    GPIO.cleanup()

if __name__ == '__main__':     #Program start from here
    setup()
    try:
        loop()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the program destroy() will be executed.
        destroy()
