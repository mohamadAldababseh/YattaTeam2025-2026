import RPi.GPIO as GPIO
import time




class DistanceSensor:
 
    def __init__(self,echo=18,trig=16):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self.EchoPin=echo
        self.TrigPin=trig
        GPIO.setup(self.EchoPin,GPIO.IN)
        GPIO.setup(self.TrigPin,GPIO.OUT)


    def getDistance(self):
        GPIO.output(self.TrigPin,GPIO.LOW)
        time.sleep(0.000002)
        GPIO.output(self.TrigPin,GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(self.TrigPin,GPIO.LOW)

        t3 = time.time()

        while not GPIO.input(self.EchoPin):
            t4 = time.time()
            if (t4 - t3) > 0.03 :
                return -1
        t1 = time.time()
        while GPIO.input(self.EchoPin):
            t5 = time.time()
            if(t5 - t1) > 0.03 :
                return -1

        t2 = time.time()
        time.sleep(0.01)
        #print ("distance_1 is %d " % (((t2 - t1)* 340 / 2) * 100))
        return ((t2 - t1)* 340 / 2) * 100
    def getTrig(self):
         return self.TrigPin
    '''    def close(self):
        GPIO.cleanup()
'''