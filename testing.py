import b0RemoteApi
import time

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
        lidar = client.simxGetObjectHandle('lidar_sensor',client.simxServiceCall())
        
        drive_forward(client, 0)
        #turn_left(client, 1)
        #time.sleep(1)
        #turn_right(client, 1)
        #time.sleep(1)
        #drive_forward(client, 0)
        child = client.simxGetObjectChild(lidar[1], 0, client.simxServiceCall())
        print(child)
        velodyne = client.simxGetObjectHandle('velodyneVPL_16_ptCloud', client.simxServiceCall())
        print(velodyne)

        velodyne_points=client.simxCallScriptFunction('lidar_sensor', 1, 'getVelodyneData_function',client.simxServiceCall())
        print(velodyne_points)

        client.simxCloseScene(client.simxServiceCall())
