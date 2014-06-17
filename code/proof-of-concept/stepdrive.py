#!/usr/bin/python
import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
 
#GPIO.setmode(GPIO.BCM)
#coil_A_1_pin, coil_A_2_pin, coil_B_1_pin, coil_B_2_pin = 17, 18, 22, 23
GPIO.setmode(GPIO.BOARD)
coil_A_1_pin, coil_A_2_pin, coil_B_1_pin, coil_B_2_pin = pins = 11, 12, 15, 16

for p in pins:
	GPIO.setup(p, GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)
#GPIO.setup(coil_A_1_pin, GPIO.OUT)
#GPIO.setup(coil_A_2_pin, GPIO.OUT)
#GPIO.setup(coil_B_1_pin, GPIO.OUT)
#GPIO.setup(coil_B_2_pin, GPIO.OUT)

def forward(delay, steps):
    for i in range(0, steps):
        setStep(1, 0, 1, 0)
        time.sleep(delay)
        setStep(0, 1, 1, 0)
        time.sleep(delay)
        setStep(0, 1, 0, 1)
        time.sleep(delay)
        setStep(1, 0, 0, 1)
        time.sleep(delay)

def backwards(delay, steps):
    for i in range(0, steps):
        setStep(1, 0, 0, 1)
        time.sleep(delay)
        setStep(0, 1, 0, 1)
        time.sleep(delay)
        setStep(0, 1, 1, 0)
        time.sleep(delay)
        setStep(1, 0, 1, 0)
        time.sleep(delay)

def setStep(w1, w2, w3, w4):
	for p, v in ((coil_A_1_pin, w1), (coil_A_2_pin, w2), (coil_B_1_pin, w3), (coil_B_2_pin, w4)):
		#GPIO.setup(p, GPIO.OUT, initial=GPIO.HIGH) if v else GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
		GPIO.output(p, v)
    #GPIO.output(coil_A_1_pin, w1)
    #GPIO.output(coil_A_2_pin, w2)
    #GPIO.output(coil_B_1_pin, w3)
    #GPIO.output(coil_B_2_pin, w4)

while True:
    delay = raw_input("Delay between steps (milliseconds)?")
    steps = raw_input("How many steps forward? ")
    forward(int(delay) / 1000.0, int(steps))
    steps = raw_input("How many steps backwards? ")
    backwards(int(delay) / 1000.0, int(steps))
