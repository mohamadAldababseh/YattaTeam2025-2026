from gpiozero import Motor


motor = Motor(12,13)

motor.forward(1)
motor.backward(0.5)
motor.stop()
