#!/usr/bin/env python

import RPi.GPIO as GPIO
import time

def fire_torpedo():
    # for 1st Motor on ENA
    ENA = 32
    IN1 = 37
    IN2 = 38

    # set pin numbers to the board's
    GPIO.setmode(GPIO.BOARD)
    # initialize EnA, In1 and In2
    GPIO.setup(ENA, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)

    GPIO.output(ENA, GPIO.HIGH)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    time.sleep(5)
    GPIO.output(ENA, GPIO.HIGH)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    time.sleep(5)
    GPIO.cleanup()
if __name__ == '__main__':

    fire_torpedo()
