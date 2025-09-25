from YB_Pcb_Car import YB_Pcb_Car
from  time import sleep
robot = YB_Pcb_Car()

for i in range(140,20,-5):
    robot.Ctrl_Servo(3,i)
    sleep(0.1)
    
for i in range(20,140,5):
    robot.Ctrl_Servo(3,i)
    sleep(0.1)
'''   
for i in range(180,0,-5):
    robot.Ctrl_Servo(4,i)
    sleep(0.1)
    
for i in range(0,180,5):
    robot.Ctrl_Servo(4,i)
    sleep(0.1)
'''

