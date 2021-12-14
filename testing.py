import b0RemoteApi
import time
import math
import numpy as np
import matplotlib.pyplot as plt

class Robot_Controller:
    connection = 0
    robot = 0
    max_dist = 2
    
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

    def getDistanceFront(self):
        distance = self.connection.simxReadProximitySensor(self.front_sonar, self.connection.simxServiceCall())
        if (distance[1] == 1):
            return distance[2]
        else:
            return False
    def getDistanceLeft(self):
        distance = self.connection.simxReadProximitySensor(self.left_sonar, self.connection.simxServiceCall())
        if (distance[1] == 1):
            return distance[2]
        else:
            return False
    def getDistanceRight(self):
        distance = self.connection.simxReadProximitySensor(self.right_sonar, self.connection.simxServiceCall())
        if (distance[1] == 1):
            return distance[2]
        else:
            return False

    def getLineColor(self):
        value = self.connection.simxGetVisionSensorImage(self.line_sensor, True, self.connection.simxServiceCall())
        return value

    def followWall(self, max_dist):
        print("following right wall . . .")
        distRight = self.getDistanceRight(max_dist)
        if distRight == max_dist:
            print("not wall, lets turn right here !")
            self.turn_right(0.5)
        else:
            self.drive_forward(0.5)
    
    def lidar_scan_around(self):
        lidar_start_direction = self.get_lidar_data()[1]
        corners = []
        points = [[],[]]
        curren_direction = lidar_start_direction+ math.pi
        last_point = []
        while not(0.2>(lidar_start_direction-curren_direction)>0):
            pos = self.get_robot_position()
            rotation = self.get_robot_rotation()
            lidar = self.get_lidar_data()
            curren_direction = lidar[1]
            lidar[1] += rotation
            pos[0] -= lidar[0]*math.cos(lidar[1])
            pos[1] -= lidar[0]*math.sin(lidar[1])
            if (len(last_point) != 0):
                if (self.euclidean_distance(last_point, pos) > 0.5):
                    corners.append([(pos[0] + last_point[0])/2, (pos[1] + last_point[1])/2, curren_direction])
            last_point = pos
            points[0].append(pos[0])
            points[1].append(pos[1])
        return (points, corners)

    def euclidean_distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0])**2+(point1[1] - point2[1])**2)

    def go_to_point(self, point):
        walls = [False,False,False]
        target_rotation = self.get_robot_rotation() + point[2]
        while(self.euclidean_distance(self.get_robot_position(), point[0:2]) > 0.5):    
            while not(0.3>(target_rotation - self.get_robot_rotation())>-0.3):
                var = self.get_robot_rotation()
                self.turn_left(1)
                time.sleep(0.3)
                self.drive_forward(0)
            walls[0] = self.getDistanceLeft()
            walls[1] = self.getDistanceFront()
            walls[2] = self.getDistanceRight()
            for i in range(0, len(walls)):
                if (walls[i] != False and walls[i] < self.max_dist):
                    walls[i] = True
                else:
                    walls[i] = False
            if (walls[1] == False):
                self.drive_forward(1)
                time.sleep(0.5)
                self.drive_forward(0)
            elif (walls[2] == False):
                while (self.getDistanceFront() != False):
                    self.turn_right(1)
                    time.sleep(0.5)
                    self.drive_forward(0)
                self.drive_forward(1)
                time.sleep(1)
                self.drive_forward(0) 
            elif (walls[0] == False):
                while (self.getDistanceFront() != False):
                    self.turn_left(1)
                    time.sleep(0.5)
                    self.drive_forward(0)
                self.drive_forward(1)
                time.sleep(1)
                self.drive_forward(0)                


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

        robot1.drive_forward(0)
        points = [[],[]]

        while True:
            #scan with lidar
            data = robot1.lidar_scan_around()
            plt.axis([-3,3,-3,3])
            plt.plot(data[0][0], data[0][1], 'o')
            plt.plot([x[0] for x in data[1]],[x[1] for x in data[1]], 'x')
            robot_position = robot1.get_robot_position()
            plt.plot(robot_position[0], robot_position[1], '*r')
            plt.show()
            print(data[1])
            points[0].append(data[0][0])
            points[1].append(data[0][1])
            plt.plot(points[0], points[1], 'o')
            plt.plot(robot_position[0], robot_position[1], '*r')
            plt.show()
            robot1.go_to_point(data[1][0])
            print("end")
            # if state == 1:
            #     robot1.followWall(max_dist)

            #     distFront = robot1.getDistanceFront(max_dist)
            #     distRight = robot1.getDistanceRight(max_dist)

            #     color = robot1.getLineColor()

            #     print(distRight)

            #     if distFront == 1:
            #         state = 2

            #     if distRight == 0:
            #         state = 3

            #     # TODO: if vision sensor detect black color - stop simulation
            #     # if color == "black"
            #     #     state = 4
            # elif state == 2:
            #     print("wall ahead, turning left . . .")
            #     robot1.turn_left(0.5)
            #     time.sleep(0.5)
            #     state = 1
            # elif state == 3:
            #     print("no wall, lets turn here right  . . .")
            #     robot1.turn_right(0.9)
            #     time.sleep(0.9)
            #     robot1.drive_forward(0.5)
            #     time.sleep(0.5)
            #     state = 1
            # # elif state == 4:
            #     # TODO: stop simulation, goal reached
                

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
