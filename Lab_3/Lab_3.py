from Motor import *
from Ultrasonic import *
import time

PWM=Motor()
ultrasonic=Ultrasonic()                
DIST=50
K=20
Speed=800

try:
    while True:
        data=ultrasonic.get_distance()
        print("The distance between the object is "+str(data)+" cm")
        diff=data-DIST
        if(diff==0):
            break
        elif(diff>0):
            diff=diff*K
            S=Speed+diff
            PWM.setMotorModel(S,S,S,S)
        else:
            diff=diff*K
            S=Speed+diff
            PWM.setMotorModel(-S,-S,-S,-S)
        time.sleep(0.1)
    PWM.setMotorModel(0,0,0,0)
except KeyboardInterrupt:
    PWM.setMotorModel(0,0,0,0)
    print ("\nProgram end due to exception")