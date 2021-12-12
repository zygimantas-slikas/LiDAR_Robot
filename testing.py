import b0RemoteApi
import time
import math

global front_right_wheel 
global front_left_wheel 
global back_right_wheel 
global back_left_wheel
global lidar

def drive_forward(client, speed):
    client.simxSetJointTargetVelocity(front_left_wheel[1], speed, client.simxServiceCall())
    client.simxSetJointTargetVelocity(front_right_wheel[1], speed, client.simxServiceCall())
    client.simxSetJointTargetVelocity(back_left_wheel[1], speed, client.simxServiceCall())
    client.simxSetJointTargetVelocity(back_right_wheel[1], speed, client.simxServiceCall())

def turn_right(client, speed):
    client.simxSetJointTargetVelocity(front_left_wheel[1], speed, client.simxServiceCall())
    client.simxSetJointTargetVelocity(front_right_wheel[1], -1*speed, client.simxServiceCall())
    client.simxSetJointTargetVelocity(back_left_wheel[1], speed, client.simxServiceCall())
    client.simxSetJointTargetVelocity(back_right_wheel[1], -1*speed, client.simxServiceCall())

def turn_left(client, speed):
    client.simxSetJointTargetVelocity(front_left_wheel[1], -1*speed, client.simxServiceCall())
    client.simxSetJointTargetVelocity(front_right_wheel[1], speed, client.simxServiceCall())
    client.simxSetJointTargetVelocity(back_left_wheel[1], -1*speed, client.simxServiceCall())
    client.simxSetJointTargetVelocity(back_right_wheel[1], speed, client.simxServiceCall())


if __name__ == "__main__":
    with b0RemoteApi.RemoteApiClient('Robot_Python_Controller','b0RemoteApi',60) as client:    

        def callb(msg):
            print(msg)
        client.simxAddStatusbarMessage('Hello',client.simxDefaultPublisher())

        robot=client.simxGetObjectHandle('robot',client.simxServiceCall())
        front_right_wheel = client.simxGetObjectHandle('front_right_motor',client.simxServiceCall())
        front_left_wheel = client.simxGetObjectHandle('front_left_motor',client.simxServiceCall())
        back_right_wheel = client.simxGetObjectHandle('back_right_motor',client.simxServiceCall())
        back_left_wheel = client.simxGetObjectHandle('back_left_motor',client.simxServiceCall())
        
        lidar = client.simxGetObjectHandle('lidar_ray',client.simxServiceCall())
        lidar_pos = client.simxGetObjectHandle('lidar_pos',client.simxServiceCall())

        for i in range(0,10):
            direction = client.simxGetJointPosition(lidar_pos[1], client.simxServiceCall())
            print("Direction: " + str(180/math.pi*direction[1])) 
            points = client.simxReadProximitySensor(lidar[1], client.simxServiceCall())
            if (len(points) > 2):
                print("Distance: " + str(points[2]) + "\n")
            time.sleep(0.5)

        drive_forward(client, 0)
        #turn_left(client, 1)
        #time.sleep(1)
        #turn_right(client, 1)
        #time.sleep(1)
        #drive_forward(client, 0)
       
       
        client.simxCloseScene(client.simxServiceCall())
