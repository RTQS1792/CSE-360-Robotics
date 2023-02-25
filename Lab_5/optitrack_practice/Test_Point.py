import time
import socket
import matplotlib.pyplot as plt
import math
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

positions = {}
rotations = {}
Destination = [0, 0]

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

# Data for plot --------------------------------
Dist_Error = []
Time = []
Rotate_Error = []
# End of data for plot -------------------------

if __name__ == "__main__":
    clientAddress = "192.168.0.23"
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
    t=0
    try:
        while is_running:
            if robot_id in positions:
                print('Last position', positions[robot_id], ' rotation', rotations[robot_id])
                print(Destination)
                xpos=positions[robot_id][0]
                ypos=positions[robot_id][1]
                xdest=Destination[0]
                ydest=Destination[1]
                xdiff=xdest-xpos
                ydiff=ydest-ypos
                dist = math.sqrt(xdiff ** 2 + ydiff ** 2)
                print("Distance: "+str(dist))
                if (dist < 0.1):
                    break
                rotation=rotations[robot_id]
                print("Ratation: "+str(rotation))
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
                #Calculate the difference between the ideal rotation and the current rotation
                print("Rotation Difference: "+str(rotation_diff))
                Dist_Error.append(dist)
                Time.append(t)
                t=t+0.05
                Rotate_Error.append(rotation_diff)
                v=500+dist*200
                print(v)
                omega=rotation_diff*25
                print(omega)

                u = np.array([v - omega, v + omega])
                u[u > 1500] = 1500
                u[u < -1500] = -1500
                print(u[0],u[0],u[1],u[1])
                command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
                s.send(command.encode('utf-8'))
                time.sleep(0.05)
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

# plot the data
plt.plot(Time,Dist_Error)
plt.xlabel('Time')
plt.ylabel('Distance Error')
plt.show()

plt.plot(Time,Rotate_Error)
plt.xlabel('Time')
plt.ylabel('Rotation Error')
plt.show()