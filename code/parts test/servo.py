from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory

servoPin=13
factory = PiGPIOFactory()
servo = AngularServo(servoPin, min_angle=0, max_angle=180,min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=factory)

servo.angle(90)
