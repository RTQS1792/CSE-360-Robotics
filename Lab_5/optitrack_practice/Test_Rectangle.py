
import time
import socket
import math
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import copy

positions = {}
rotations = {}
Rect=[[2,2.4],[-1,2.4],[-1,0],[2,0]]

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz

# Connect to the robot
IP_ADDRESS = '192.168.0.204'

Lis_X=[]
Lis_Y=[]


# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')

if __name__ == "__main__":
    clientAddress = "192.168.0.8"
    optitrackServerAddress = "192.168.0.4"
    robot_id = 211

    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()

    Firstdest=[2,2.4]
    try:
        while is_running:
            if robot_id in positions:
                print('Last position', positions[robot_id], ' rotation', rotations[robot_id])
                print("Destination: "+str(Firstdest))

                xpos=positions[robot_id][0]
                ypos=positions[robot_id][1]

                xdest=Firstdest[0]
                ydest=Firstdest[1]

                xdiff=xdest-xpos
                ydiff=ydest-ypos
                dist = math.sqrt(xdiff ** 2 + ydiff ** 2)
                
                rotation=rotations[robot_id]
                print("Ratation: "+str(rotation))

                # Calculate the ideal rotation and change it to degrees
                ideal_rotation = np.arctan(ydiff / xdiff)
                ideal_rotation = np.rad2deg(ideal_rotation)

                print("Ideal Rotation: "+str(ideal_rotation))
                if(xdiff>0):
                    rotation_diff=ideal_rotation-rotation
                else:
                    rotation_diff=ideal_rotation-rotation+180
                if(rotation_diff>180):
                    rotation_diff=rotation_diff-360
                if(rotation_diff<-180):
                    rotation_diff=rotation_diff+360
                
                # Calculate the difference between the ideal rotation and the current rotation
                print("Rotation Difference: "+str(rotation_diff))
                Kv=500
                v=500+dist*Kv
                print("Speed: "+str(v))
                Komega=20
                omega=rotation_diff*Komega
                print("Angular Speed: "+str(omega))

                u = np.array([v - omega, v + omega])
                u[u > 1500] = 1500
                u[u < -1500] = -1500
                print("Moter Parameters: ",u[0],u[0],u[1],u[1])
                command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
                s.send(command.encode('utf-8'))
                time.sleep(0.5)
                if(dist<0.2):
                    print("Destination Reached")
                    command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(0, 0, 0, 0)
                    s.send(command.encode('utf-8'))
                    time.sleep(2)
                    break
                time.sleep(0.5)
    except KeyboardInterrupt:
        streaming_client.shutdown()
        command = 'CMD_MOTOR#00#00#00#00\n'
        s.send(command.encode('utf-8'))
        s.shutdown(2)
        s.close()


    i=0
    flag=0
    t=-0.05
    Start=copy.copy(Rect[0])
    try:
        while is_running:
            if robot_id in positions:

                # print('Last position', positions[robot_id], ' rotation', rotations[robot_id])
                print("Destination: "+str(Start))

                xpos=positions[robot_id][0]
                Lis_X.append(xpos)
                ypos=positions[robot_id][1]
                Lis_Y.append(ypos)

                xdest=Start[0]
                ydest=Start[1]

                xdiff=xdest-xpos
                ydiff=ydest-ypos
                dist = math.sqrt(xdiff ** 2 + ydiff ** 2)
                
                rotation=rotations[robot_id]
                # print("Ratation: "+str(rotation))
                
                # Calculate the ideal rotation and change it to degrees
                ideal_rotation = np.arctan(ydiff / xdiff)
                ideal_rotation = np.rad2deg(ideal_rotation)

                # print("Ideal Rotation: "+str(ideal_rotation))
                if(xdiff>0):
                    rotation_diff=ideal_rotation-rotation
                else:
                    rotation_diff=ideal_rotation-rotation+180
                if(rotation_diff>180):
                    rotation_diff=rotation_diff-360
                if(rotation_diff<-180):
                    rotation_diff=rotation_diff+360
                
                # Calculate the difference between the ideal rotation and the current rotation
                # print("Rotation Difference: "+str(rotation_diff))

                v=500+dist*800
                # print(v)
                omega=rotation_diff*60
                # print(omega)

                u = np.array([v - omega, v + omega])
                u[u > 1500] = 1500
                u[u < -1500] = -1500
                # print(u[0],u[0],u[1],u[1])
                command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
                s.send(command.encode('utf-8'))
                Start[flag]+=0.5*t
                # Calculate the difference between the current position and the destination
                Index=(i+1)%4
                print("Going to ",i+1,"th point")
                print(Rect)
                xd=Start[0]-Rect[Index][0]
                yd=Start[1]-Rect[Index][1]
                diff=math.sqrt(xd**2+yd**2)
                if(diff<0.1):
                    i+=1
                    if(i%4==1):
                        print("going in -x direction")
                        flag=1
                        t=-0.05
                    if(i%4==2):
                        print("going in -y direction")
                        flag=0
                        t=0.05
                    if(i%4==3):
                        print("going in +x direction")
                        flag=1
                        t=0.05
                    if(i%4==0):
                        print("going in +y direction")
                        flag=0
                        t=-0.05 
                    Start=copy.copy(Rect[Index])
                time.sleep(0.05)
                if(i>4):
                    break

    except KeyboardInterrupt:
        streaming_client.shutdown()
        command = 'CMD_MOTOR#00#00#00#00\n'
        s.send(command.encode('utf-8'))
        s.shutdown(2)
        s.close()
streaming_client.shutdown()
command = 'CMD_MOTOR#00#00#00#00\n'
s.send(command.encode('utf-8'))
s.shutdown(2)
s.close()


for i in range(len(Lis_X)):
    plt.plot(Lis_X[i], Lis_Y[i], marker="o", markersize=5, markeredgecolor="black", markerfacecolor="black")
ax = plt.gca()
ax.add_patch(Rectangle((-1, 0), 3, 2.4,color="red", fill=False))
ax.set_aspect('equal', adjustable='box')
plt.show()