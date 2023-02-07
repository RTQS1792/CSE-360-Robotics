from Motor import *
from Led import *
from Buzzer import *
import time

PWM=Motor()
led=Led()
buzzer=Buzzer()

try:
    PWM.setMotorModel(1000,1000,1000,1000)
    print("The car is moving forward")
    time.sleep(2)
    PWM.setMotorModel(-1500,-1500,2000,2000)       #Left 90 Degree
    print ("The car is turning left")
    time.sleep(0.65)
    led.ledIndex(0x01,255,0,0)      #Red
    print ("The led 0 has been lit with color Red")
    
    PWM.setMotorModel(1000,1000,1000,1000)
    print("The car is moving forward")
    time.sleep(2)
    PWM.setMotorModel(-1500,-1500,2000,2000)       #Left 90 Degree
    print ("The car is turning left")
    time.sleep(0.65)
    led.ledIndex(0x02,0,0,255)      #Blue
    print ("The led 0.7 has been lit with color Blue")
    
    PWM.setMotorModel(1000,1000,1000,1000)
    print("The car is moving forward")
    time.sleep(2)
    PWM.setMotorModel(-1500,-1500,2000,2000)       #Left 90 Degree
    print ("The car is turning left")
    time.sleep(0.65)
    led.ledIndex(0x04,0,255,0)      #Green
    print ("The led 2 has been lit with color Green")
    
    PWM.setMotorModel(1000,1000,1000,1000)
    print("The car is moving forward")
    time.sleep(2)
    PWM.setMotorModel(-1500,-1500,2000,2000)        #Left 90 Degree
    print ("The car is turning left") 
    time.sleep(0.65)
    led.ledIndex(0x08,255,255,0)      #Yellow
    print ("The led 3 has been lit with color Yellow")
    PWM.setMotorModel(0,0,0,0)
    print ("Motor Stopped") # Stop the car
    time.sleep(2)   # Wait for 2s to see the led
    led.colorWipe(led.strip, Color(0,0,0))  #turn off the led
    print ("Led Stopped")
    buzzer.run('1')    #Buzzer beep for 1s
    print ("Buzzer beep for 1s") 
    time.sleep(1)
    buzzer.run('0')    
except KeyboardInterrupt:
    PWM.setMotorModel(0,0,0,0)
    led.colorWipe(led.strip, Color(0,0,0))  #turn off the light
    buzzer.run('0')    
    print ("\nProgram end due to exception")