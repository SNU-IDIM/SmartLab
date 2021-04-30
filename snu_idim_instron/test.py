import Jetson.GPIO as GPIO
import time

PIN = 15
GPIO.setmode(GPIO.BOARD)
GPIO.setup(PIN, GPIO.OUT, initial=GPIO.LOW)
time.sleep(5)
GPIO.output(PIN, GPIO.HIGH)
time.sleep(5)
GPIO.output(PIN, GPIO.LOW)