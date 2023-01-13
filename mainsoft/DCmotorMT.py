import RPi.GPIO as GPIO
import time

moterA = 17
moterB = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(moterA, GPIO.OUT)

GPIO.output(moterA, True)

                                                                                               
time.sleep(3)
GPIO.output(moterA, False)

GPIO.cleanup() 