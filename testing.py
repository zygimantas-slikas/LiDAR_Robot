import b0RemoteApi
import time
import math
import numpy as np
import matplotlib.pyplot as plt

class Robot_Controller:
    connection = 0
    robot = 0
    front_right_wheel = 0
    front_left_wheel  = 0
    back_right_wheel  = 0
    back_left_wheel   = 0 
    lidar = 0
    lidar_position = 0
    robot_rotation = 0
    
    def drive_forward(self, speed):
        self.connection.simxSetJointTargetVelocity(self.front_left_wheel, speed, self.connection.simxServiceCall())
        self.connection.simxSetJointTargetVelocity(self.front_right_wheel, speed, self.connection.simxServiceCall())
        self.connection.simxSetJointTargetVelocity(self.back_left_wheel, speed, self.connection.simxServiceCall())
        self.connection.simxSetJointTargetVelocity(self.back_right_wheel, speed, self.connection.simxServiceCall())

    def turn_right(self, speed):
        self.connection.simxSetJointTargetVelocity(self.front_left_wheel, speed, self.connection.simxServiceCall())
        self.connection.simxSetJointTargetVelocity(self.front_right_wheel, -1*speed, self.connection.simxServiceCall())
        self.connection.simxSetJointTargetVelocity(self.back_left_wheel, speed, self.connection.simxServiceCall())
        self.connection.simxSetJointTargetVelocity(self.back_right_wheel, -1*speed, self.connection.simxServiceCall())

    def turn_left(self, speed):
        self.connection.simxSetJointTargetVelocity(self.front_left_wheel, -1*speed, self.connection.simxServiceCall())
        self.connection.simxSetJointTargetVelocity(self.front_right_wheel, speed, self.connection.simxServiceCall())
        self.connection.simxSetJointTargetVelocity(self.back_left_wheel, -1*speed, self.connection.simxServiceCall())
        self.connection.simxSetJointTargetVelocity(self.back_right_wheel, speed, self.connection.simxServiceCall())

    def get_robot_position(self):
        return self.connection.simxGetObjectPosition(self.robot, -1, self.connection.simxServiceCall())[1][0:2]

    def get_lidar_data(self):
        direction = self.connection.simxGetJointPosition(self.lidar_position, self.connection.simxServiceCall())[1]
        distance = self.connection.simxReadProximitySensor(self.lidar, self.connection.simxServiceCall())[2]
        return [distance, direction]

    def get_robot_rotation(self):
        return self.connection.simxGetObjectOrientation(self.robot, -1, self.connection.simxServiceCall())[1][2]

    def get_lidar_point(self):
        pos = self.get_robot_position()
        rotation = self.get_robot_rotation()
        lidar = self.get_lidar_data()
        lidar[1] += rotation
        pos[0] -= lidar[0]*math.cos(lidar[1])
        pos[1] -= lidar[0]*math.sin(lidar[1])
        return pos

if __name__ == "__main__":
    with b0RemoteApi.RemoteApiClient('Robot_Python_Controller','b0RemoteApi',60) as client:    
        robot1 = Robot_Controller() 
        robot1.connection = client
        robot1.robot=client.simxGetObjectHandle('robot',client.simxServiceCall())[1]
        robot1.front_right_wheel = client.simxGetObjectHandle('front_right_motor',client.simxServiceCall())[1]
        robot1.front_left_wheel = client.simxGetObjectHandle('front_left_motor',client.simxServiceCall())[1]
        robot1.back_right_wheel = client.simxGetObjectHandle('back_right_motor',client.simxServiceCall())[1]
        robot1.back_left_wheel = client.simxGetObjectHandle('back_left_motor',client.simxServiceCall())[1]
        robot1.lidar = client.simxGetObjectHandle('lidar_ray',client.simxServiceCall())[1]
        robot1.lidar_position = client.simxGetObjectHandle('lidar_pos',client.simxServiceCall())[1]


        robot1.drive_forward(1)
        # robot1.turn_right(1.968)
        points = [[],[]]
        while True:
            plt.axis([-3,3,-3,3])
            plt.scatter(points[0], points[1])
            for i in range(0, 50):
                point = robot1.get_lidar_point()
                points[0].append(point[0])
                points[1].append(point[1])
                plt.scatter(point[0], point[1])
                plt.pause(0.001)
            plt.pause(0.5)
            robot1.drive_forward(1)
            time.sleep(2)
            robot1.turn_left(1)
            time.sleep(1)
            robot1.drive_forward(0)

        client.simxCloseScene(client.simxServiceCall())
