import b0RemoteApi
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from joblib import Parallel, delayed
import asyncio

class Robot_Controller:
    connection = 0
    robot = 0

    front_right_wheel = 0
    front_left_wheel  = 0
    back_right_wheel  = 0
    back_left_wheel   = 0 

    lidar = 0
    right_rear_lidar = 0
    right_front_lidar = 0
    lidar_position = 0

    robot_rotation = 0

    front_sonar = 0
    left_sonar = 0
    right_sonar = 0

    line_sensor = 0
    
    def drive_forward(self, speed):
        self.connection.simxSetJointTargetVelocity(self.front_left_wheel, speed, self.connection.simxServiceCall())
        self.connection.simxSetJointTargetVelocity(self.front_right_wheel, speed, self.connection.simxServiceCall())
        self.connection.simxSetJointTargetVelocity(self.back_left_wheel, speed, self.connection.simxServiceCall())
        self.connection.simxSetJointTargetVelocity(self.back_right_wheel, speed, self.connection.simxServiceCall())

    def drive_backward(self, speed):
        self.connection.simxSetJointTargetVelocity(self.front_left_wheel, -1*speed, self.connection.simxServiceCall())
        self.connection.simxSetJointTargetVelocity(self.front_right_wheel, -1*speed, self.connection.simxServiceCall())
        self.connection.simxSetJointTargetVelocity(self.back_left_wheel, -1*speed, self.connection.simxServiceCall())
        self.connection.simxSetJointTargetVelocity(self.back_right_wheel, -1*speed, self.connection.simxServiceCall())

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

    def get_right_rear_data(self):
        distance = self.connection.simxReadProximitySensor(self.right_rear_lidar, self.connection.simxServiceCall())[2]
        return distance

    def get_right_front_data(self):
        distance = self.connection.simxReadProximitySensor(self.right_front_lidar, self.connection.simxServiceCall())[2]
        return distance

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

    def getDistanceFront(self, max_dist):
        detected = self.connection.simxReadProximitySensor(self.front_sonar, self.connection.simxServiceCall())[1]

        if detected == 1:
            distance = self.connection.simxReadProximitySensor(self.front_sonar, self.connection.simxServiceCall())[2]
        elif detected == 0:
            distance = max_dist

        return distance

    def getDistanceLeft(self, max_dist):
        detected = self.connection.simxReadProximitySensor(self.left_sonar, self.connection.simxServiceCall())[1]

        if detected == 1:
            distance = self.connection.simxReadProximitySensor(self.left_sonar, self.connection.simxServiceCall())[2]
        elif detected == 0:
            distance = max_dist

        return distance

    def getDistanceRight(self, max_dist):
        detected = self.connection.simxReadProximitySensor(self.right_sonar, self.connection.simxServiceCall())[1]

        if detected == 1:
            distance = self.connection.simxReadProximitySensor(self.right_sonar, self.connection.simxServiceCall())[1]
        elif detected == 0:
            distance = max_dist

        return distance

    def getLineColor(self):
        value = self.connection.simxGetVisionSensorImage(self.line_sensor, True, self.connection.simxServiceCall())

        return value[2]

    def drawMap(points):
        plt.axis([-3,3,-3,3])
        plt.scatter(points[0], points[1])

        for i in range(0, 50):
            point = robot1.get_lidar_point()
            points[0].append(point[0])
            points[1].append(point[1])
            plt.scatter(point[0], point[1])
            plt.pause(0.001)


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

        robot1.front_sonar = client.simxGetObjectHandle('front_proximity_sensor',client.simxServiceCall())[1]
        robot1.left_sonar = client.simxGetObjectHandle('left_proximity_sensor',client.simxServiceCall())[1]
        robot1.right_sonar = client.simxGetObjectHandle('right_proximity_sensor',client.simxServiceCall())[1]

        robot1.right_rear_lidar = client.simxGetObjectHandle('right_rear_lidar',client.simxServiceCall())[1]
        robot1.right_front_lidar = client.simxGetObjectHandle('right_front_lidar',client.simxServiceCall())[1]

        robot1.line_sensor = client.simxGetObjectHandle('Vision_sensor',client.simxServiceCall())[1]

        max_dist = 2
        dist = max_dist

        # 1 - follow right wall
        # 2 - wall in front. Turn 90
        # 3 - wall not detected. Curve 90
        # 4 - finish line detected
        state = 1

        points = [[],[]]

        while True:
            if state == 1:
                # print("following right wall . . .")
                robot1.drive_forward(0.5)

                distFront = robot1.getDistanceFront(max_dist)
                distRight = robot1.getDistanceRight(max_dist)
                distLeft = robot1.getDistanceLeft(max_dist)

                print(distFront)

                color = robot1.getLineColor()

                if distFront < 0.5:
                    state = 2

                if distRight == max_dist:
                    state = 3

                if color == b'\x17':
                    state = 4

                

            elif state == 2:
                # print("wall ahead, turning left . . .")
                if distLeft < 0.5:
                    robot1.drive_backward(0.3)
                    time.sleep(0.3)
                    robot1.turn_right(0.3)
                    time.sleep(0.3)
                    state = 1
                else:
                    robot1.turn_left(0.6)
                    time.sleep(0.6)
                    state = 1

            elif state == 3:
                # print("no wall, lets curve right  . . .")
                robot1.turn_right(0.7)
                time.sleep(0.7)
                robot1.drive_forward(0.5)
                time.sleep(0.5)
                state = 1

            elif state == 4:
                robot1.drive_backward(0)
                time.sleep(60)

            plt.axis([-3,3,-3,3])
            plt.scatter(points[0], points[1])

            for i in range(0, 1):
                point = robot1.get_lidar_point()
                points[0].append(point[0])
                points[1].append(point[1])
                plt.scatter(point[0], point[1])
                plt.pause(0.001)


        # while True:
        #     right_rear_lidar = robot1.get_right_rear_data()
        #     right_front_lidar = robot1.get_right_front_data()

            
        #     distFront = robot1.getDistanceFront(max_dist)
        #     distLeft = robot1.getDistanceLeft(max_dist)
        #     distRight = robot1.getDistanceRight(max_dist)


        

            #     if right_rear_lidar == right_front_lidar:
            #         print("driving forward")
            #         robot1.drive_forward(0.5)
            #         time.sleep(0.5)
            #     elif right_front_lidar > right_rear_lidar:
            #         print("need to adjust to right")
            #         robot1.turn_right(0.5)
            #         time.sleep(0.5)
            #     elif right_front_lidar < right_rear_lidar:
            #         print("need to adjust to left")
            #         robot1.turn_left(0.5)
            #         time.sleep(0.5)

        #     robot1.drive_forward(0.5)


        #     if distFront == max_dist:
        #         print("Wall ahead !")

        #         if distLeft == max_dist:
        #             print("have to turn right !")
        #             robot1.turn_right(0.5)
        #             time.sleep(0.5)
        #         elif distRight == max_dist:
        #             print("have to turn left !")
        #             robot1.turn_left(0.5)
        #             time.sleep(0.5)
        #         else:
        #             print("need to turn somwhere")
        #             robot1.drive_backward(0.5)
        #             time.sleep(1)
        #             robot1.turn_left(0.5)
        #             time.sleep(1)
        
            # plt.axis([-3,3,-3,3])
            # plt.scatter(points[0], points[1])

            # for i in range(0, 50):
            #     point = robot1.get_lidar_point()
            #     points[0].append(point[0])
            #     points[1].append(point[1])
            #     plt.scatter(point[0], point[1])
            #     plt.pause(0.001)

        client.simxCloseScene(client.simxServiceCall())
