import b0RemoteApi
import time
import math
import numpy as np
import matplotlib.pyplot as plt

class Robot_Controller:
    connection = 0
    robot = 0
    max_dist = 0.5
    robot_rotation = 0
    visited_points = []

    front_right_wheel = 0
    front_left_wheel  = 0
    back_right_wheel  = 0
    back_left_wheel   = 0 
    lidar = 0
    lidar_position = 0
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
            return 2
    def getDistanceLeft(self):
        distance = self.connection.simxReadProximitySensor(self.left_sonar, self.connection.simxServiceCall())
        if (distance[1] == 1):
            return distance[2]
        else:
            return 2
    def getDistanceRight(self):
        distance = self.connection.simxReadProximitySensor(self.right_sonar, self.connection.simxServiceCall())
        if (distance[1] == 1):
            return distance[2]
        else:
            return 2

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
                if (self.euclidean_distance(last_point, pos) > 0.45):
                    corners.append([(pos[0] + last_point[0])/2, (pos[1] + last_point[1])/2, curren_direction])
            last_point = pos
            points[0].append(pos[0])
            points[1].append(pos[1])
        return (points, corners)

    def euclidean_distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0])**2+(point1[1] - point2[1])**2)

    def go_to_point_2(self, point):
        front_dist = self.getDistanceFront()
        left_dist = self.getDistanceLeft()
        right_dist = self.getDistanceRight()
        position = self.get_robot_position()
        while self.euclidean_distance(position, point[0:2]) > 0.3:
            position = self.get_robot_position()
            self.visited_points.append(position)
            front_dist = self.getDistanceFront()
            left_dist = self.getDistanceLeft()
            right_dist = self.getDistanceRight()
            while ((self.euclidean_distance(position, point[0:2]) > 0.3) and
            (front_dist > 0.5)):
                position = self.get_robot_position()
                self.visited_points.append(position)
                front_dist = self.getDistanceFront()
                left_dist = self.getDistanceLeft()
                right_dist = self.getDistanceRight()
                if left_dist < 0.4:
                    self.turn_right(0.5)
                    time.sleep(0.5)
                    self.drive_forward(0)
                if right_dist < 0.4:
                    self.turn_left(0.5)
                    time.sleep(0.5)
                    self.drive_forward(0)
                if front_dist > 0.5:
                    self.drive_forward(1)
                    time.sleep(0.5)
                    self.drive_forward(0)
                if ((front_dist>0.8)and(left_dist>0.8)) or ((front_dist>0.8)and(right_dist>0.8)) or ((left_dist>0.8)and(right_dist>0.8)):
                    break
            if self.euclidean_distance(position, point[0:2]) < 0.3:
                break
            if (front_dist <= 0.5) or ((front_dist>0.8)and(left_dist>0.8)) or ((front_dist>0.8)and(right_dist>0.8)) or ((left_dist>0.8)and(right_dist>0.8)):
                self.rotate(self.get_relative_point_rotation(point[0:2]))

    def is_point_unvisited(self, point):
        for i in self.visited_points:
            if self.euclidean_distance(i, point[0:2]) < 0.8:
                return False
        return True

    def rotate(self, rotation):
        target_rotation = self.get_robot_rotation() + rotation
        if (abs(target_rotation)//math.pi > 0):
            if (target_rotation > 0):
                target_rotation = (-1*math.pi) + (target_rotation % math.pi)
            else: 
                target_rotation = math.pi - (abs(target_rotation) % math.pi) 
        if (rotation > 0):
            while not(0.3>(target_rotation - self.get_robot_rotation())>-0.3):
                self.turn_left(1)
                time.sleep(0.3)
                self.drive_forward(0)
        elif (rotation < 0):
            while not(0.3>(target_rotation - self.get_robot_rotation())>-0.3):
                self.turn_right(1)
                time.sleep(0.3)
                self.drive_forward(0)

    def get_relative_point_rotation(self, point):
        robot_point = self.get_robot_position()
        robot_rotation = self.get_robot_rotation()
        robot_to_point_distance = self.euclidean_distance(point, robot_point)
        vector_to_point = [0,0]
        vector_to_point[0] = point[0]-robot_point[0]
        vector_to_point[1] = point[1]-robot_point[1]
        points_absolute_rotation = math.acos(-1*vector_to_point[0]/(robot_to_point_distance))
        if (self.euclidean_distance(point, [robot_point[0] - math.cos(points_absolute_rotation)*robot_to_point_distance, robot_point[1] - math.sin(points_absolute_rotation)*robot_to_point_distance]) 
        < self.euclidean_distance(point, [robot_point[0] - math.cos(points_absolute_rotation)*robot_to_point_distance, robot_point[1] + math.sin(points_absolute_rotation)*robot_to_point_distance])):
            return points_absolute_rotation - robot_rotation
        else:
            return -1*(points_absolute_rotation) - robot_rotation
   
def zip_points(points, old_bit_map):
    points2 = [[],[]]
    points2[0] = [int((i//0.1)+30) for i in points[0]]
    points2[1] = [int((i//0.1)+30) for i in points[1]]
    for i in range(0, len(points2[0])):
            old_bit_map[points2[1][i], points2[0][i]] = 1
    return old_bit_map
def get_points_from_bit_map(bit_map):
    points = [[],[]]
    for i in range(0, 61):
        for j in range(0, 61):
            if (bit_map[j,i] == 1):
                points[0].append(((i-30)/10)+0.05)
                points[1].append(((j-30)/10)+0.05)
    return points
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
        robot1.line_sensor = client.simxGetObjectHandle('Vision_sensor',client.simxServiceCall())[1]

        robot1.drive_forward(0)
        zeros = np.zeros((61,61))

        while True:
            data = robot1.lidar_scan_around()
            plt.axis([-3,3,-3,3])
            plt.plot(data[0][0], data[0][1], 'o')
            plt.plot([x[0] for x in data[1]],[x[1] for x in data[1]], 'x')
            robot_position = robot1.get_robot_position()
            plt.plot(robot_position[0], robot_position[1], '*r')
            plt.show()
            print(data[1])
            zeros = zip_points(data[0], zeros)
            points = get_points_from_bit_map(zeros)
            plt.axis([-3,3,-3,3])
            plt.plot(points[0], points[1], 'o')
            plt.plot(robot_position[0], robot_position[1], '*r')
            plt.show()

            unvisited = [x for x in data[1] if robot1.is_point_unvisited(x)]
            if (len(unvisited) > 0):
                print("goint to " + str(unvisited[0]))
                robot1.go_to_point_2(unvisited[0])
            else:
                print("everything visited")
                break
            print("next iteration")
        client.simxCloseScene(client.simxServiceCall())
