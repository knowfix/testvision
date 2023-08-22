import RPi.GPIO as GPIO
import time

def rotate(pin, angle):
    # Set GPIO numbering mode
    GPIO.setmode(GPIO.BOARD)

    # Set pin as an output
    GPIO.setup(pin,GPIO.OUT)
    servo1 = GPIO.PWM(pin,50) # Note 50 = 50Hz pulse

    #start PWM running, but with value of 0 (pulse off)
    servo1.start(0)

    #turn back to input angle degrees
    print ("Turning back to 0 degrees")
    servo1.ChangeDutyCycle(2+(angle/18))
    time.sleep(0.5)

    #Clean things up at the end
    servo1.stop()
    GPIO.cleanup()
    print ("Turning back to {angle} degrees")

if __name__ == '__main__' :
    rotate(11,180)