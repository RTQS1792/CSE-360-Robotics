import time
import socket
import math
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import matplotlib.pyplot as plt

positions = {}
rotations = {}
theta=0
Destination = [np.sin(theta),np.cos(theta)]

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz

# Connect to the robot
IP_ADDRESS = '192.168.0.204'

# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')

# Datas for plot
Lis_X = []
Lis_Y = []

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
    Firstdest=[0,0]
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
                    time.sleep(2)
                    break
                time.sleep(0.5)
    except KeyboardInterrupt:
        streaming_client.shutdown()
        command = 'CMD_MOTOR#00#00#00#00\n'
        s.send(command.encode('utf-8'))
        s.shutdown(2)
        s.close()

    # Control policy
    try:
        while is_running:
            if robot_id in positions:
                print('Last position', positions[robot_id], ' rotation', rotations[robot_id])
                print("Destination: "+str(Destination))

                xpos=positions[robot_id][0]
                Lis_X.append(xpos)
                ypos=positions[robot_id][1]
                Lis_Y.append(ypos)

                xdest=Destination[0]
                ydest=Destination[1]

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
                Komega=25
                omega=rotation_diff*Komega
                print("Angular Speed: "+str(omega))

                u = np.array([v - omega, v + omega])
                u[u > 1500] = 1500
                u[u < -1500] = -1500
                print("Moter Parameters: ",u[0],u[0],u[1],u[1])
                command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
                s.send(command.encode('utf-8'))

                theta+=np.pi/46
                Destination=[np.sin(theta),np.cos(theta)]
                time.sleep(0.15)
                if(theta>2*np.pi):
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

print(Lis_X)

# Plot the path
for i in range(len(Lis_X)):
    plt.plot(Lis_X[i], Lis_Y[i], marker="o", markersize=5, markeredgecolor="black", markerfacecolor="black")
circle1 = plt.Circle((0, 0), 1, color='r', fill=False)
plt.gcf().gca().add_artist(circle1)
ax = plt.gca()
ax.set_aspect('equal', adjustable='box')
plt.show()

