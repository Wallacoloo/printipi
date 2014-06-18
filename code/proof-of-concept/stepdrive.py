#!/usr/bin/python
#GPIO 1 (phys 12) has been damaged! Can only output "low".
import RPi.GPIO as GPIO
from RPIO import PWM
import time

#What is the source of the vibrations?
#Doesn't happen when all pins are OUT and LOW. (0 0 0 0)
#Happens when (0 0 0 1)
#Doesn't happen when (0 0 1 1)
#Happens when (1 0 1 1)
#Doesn't happen when (1 1 1 1)
#Happens when (1 0 1 0)
#Basically happens when the inputs are out-of-phase
#Putting a cap over the 12V power supply seems to help this (10uF)
#  RAMPS uses 100 uF cap for each driver.
#  1000 uF / $0.09 http://www.taydaelectronics.com/capacitors/electrolytic-capacitors/1000uf-16v-105c-radial-electrolytic-capacitor-10x16mm.html
#http://forum.arduino.cc/index.php/topic,226350.0.html recommends a different combo.
#NO JITTERING OCCURS WHEN 9v BATTERY IS POWERING CIRCUITRY INSTEAD OF AC ADAPTER, AND STEPPER TURNS CORRECTLY.
#then use more capacitors, or - better yet - a voltage regulator.
#  Radioshack sells good-sized caps (1000+uF) http://www.radioshack.com/family/index.jsp?categoryId=12648753&pg=2
#  Radioshack sells 12V regulator http://www.radioshack.com/product/index.jsp?productId=2062600
#    LM7812 requires 14.5V input voltage.
#  I have a 5V, 1.5A regulator on-hand: https://www.sparkfun.com/products/107
#    NO JITTERING OCCURS WHEN USING 5V REGULATOR AND LAPTOP POWER SUPPLY (though regulator heats up QUICKLY)
#Page for smoothing capacitor selection: http://electronicsclub.info/powersupplies.htm#smoothing
#Jittering still occurs with a wall-wart 12V 1.5A power-supply though less-so during the actual driving.
#Explanation for jitter in wall warts: http://www.robotshop.com/media/files/pdf/unregulated-power-supply-tutorial-prt-08619.pdf
#Visible voltage jitter under load using multimeter
#Even though my LAPTOP power supply shows no jitter with multimeter, still some jitter wit stepper.
#Radioshack Switching regulator: http://www.radioshack.com/product/index.jsp?productId=12753563
#1000 uF cap from Radioshack makes no difference, nor did the 4700 uF
#@W5V0 says I'm shorting my power supply: http://chat.stackexchange.com/rooms/15/electrical-engineering
#@W5V0 recommends I use a variable voltage switching regulator on each stepper.
#  4.2V, 1.5A are absolute maximum ratings.

GPIO.setwarnings(False)
 
#GPIO.setmode(GPIO.BCM)
#coil_A_1_pin, coil_A_2_pin, coil_B_1_pin, coil_B_2_pin = 17, 18, 22, 23
GPIO.setmode(GPIO.BOARD)
coil_A_1_pin, coil_A_2_pin, coil_B_1_pin, coil_B_2_pin = pins = 11, 13, 15, 16 #board numbering
pwm_pin = 24 #BCM numbering
step_configs = (1, 0, 1, 0), (0, 1, 1, 0), (0, 1, 0, 1), (1, 0, 0, 1)
#step_configs = (1, 0, 0, 1), (0, 0, 0, 1), (0, 1, 0, 1), (0, 1, 0, 0), (0, 1, 1, 0), (0, 0, 1, 0), (1, 0, 1, 0), (1, 0, 0, 0)

for p in pins:
	#GPIO.setup(p, GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)
	GPIO.setup(p, GPIO.OUT)

servo = PWM.Servo(subcycle_time_us=3000) #min 3 ms subcycle.
servo.set_servo(pwm_pin, 750) #1/4 duty cycle.

def forward(delay, steps):
	for i in range(steps):
		for c in step_configs:
			setStep(*c)
			time.sleep(delay)
			
def backwards(delay, steps):
	for i in range(steps):
		for c in step_configs[::-1]:
			setStep(*c)
			time.sleep(delay)

"""def forward(delay, steps):
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
        time.sleep(delay)"""

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
