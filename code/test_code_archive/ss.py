from gpiozero import Motor, AngularServo
from gpiozero import RotaryEncoder
import time

servo = AngularServo(12, min_angle=0, max_angle=180,min_pulse_width=0.0005, max_pulse_width=0.0025)

rotor = RotaryEncoder(17, 27, wrap=True, max_steps=18000)

motor = Motor(19,13)
mid = 110
maxa = 150
mina = 70
ds=43
speed=1

def stop():
    
    motor.stop()
    
def turnleft():
    servo.angle=mina
    motor.forward(speed)

    while rotor.steps <1250:
       
        continue
    motor.stop()
    
def moveforward2():
    motor.forward(speed)
    while  (f>160):
        print(r)
        print(f)

        error = (r-10)*5
        angle=(mid)+error
        if angle>150:
            angle=150
        elif angle <70:
            angle=70
        servo.angle=angle
        
        continue
    while (rotor.steps/ds <100):


        angle=(mid)+error
        if angle>150:
            angle=150
        elif angle <70:
            angle=70
        servo.angle=angle
        
        continue
    motor.stop()
moveforward2()
rotor.steps=0
turnleft()
rotor.steps=0
moveforward2()





