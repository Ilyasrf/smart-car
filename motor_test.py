import RPi.GPIO as GPIO
from time import sleep

# Motor A (left)
IN1 = 17
IN2 = 18
ENA = 23

# Motor B (right)
IN3 = 27
IN4 = 22
ENB = 24

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup([IN1, IN2, ENA, IN3, IN4, ENB], GPIO.OUT)

# Forward
GPIO.output(IN1, GPIO.HIGH)
GPIO.output(IN2, GPIO.LOW)
GPIO.output(ENA, GPIO.HIGH)

GPIO.output(IN3, GPIO.HIGH)
GPIO.output(IN4, GPIO.LOW)
GPIO.output(ENB, GPIO.HIGH)

sleep(5)

GPIO.cleanup()
