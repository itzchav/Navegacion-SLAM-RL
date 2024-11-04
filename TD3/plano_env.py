import math
import os
import random
import subprocess
import time
from os import path

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from squaternion import Quaternion
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import csv

#GOAL_REACHED_DIST = 0.4
GOAL_REACHED_DIST = 0.35
COLLISION_DIST = 0.158
TIME_DELTA = 0.1


# Check if the random goal position is located on an obstacle and do not accept it if it is
def check_pos(x, y):
    goal_ok = True
    """
    if -3.8 > x > -6.2 and 6.2 > y > 3.8:
        goal_ok = False

    if -1.3 > x > -2.7 and 4.7 > y > -0.2:
        goal_ok = False

    if -0.3 > x > -4.2 and 2.7 > y > 1.3:
        goal_ok = False

    if -0.8 > x > -4.2 and -2.3 > y > -4.2:
        goal_ok = False

    if -1.3 > x > -3.7 and -0.8 > y > -2.7:
        goal_ok = False

    if 4.2 > x > 0.8 and -1.8 > y > -3.2:
        goal_ok = False

    if 4 > x > 2.5 and 0.7 > y > -3.2:
        goal_ok = False

    if 6.2 > x > 3.8 and -3.3 > y > -4.2:
        goal_ok = False

    if 4.2 > x > 1.3 and 3.7 > y > 1.5:
        goal_ok = False

    if -3.0 > x > -7.2 and 0.5 > y > -1.5:
        goal_ok = False

    if x > 4.5 or x < -4.5 or y > 4.5 or y < -4.5:
        goal_ok = False
    """
    return goal_ok
"""

"""

#Prueba 1 (5 puntos)
goal_list=[
[	0.858060035211268	,	-0.253895535714286	],
[	2.11991302816901	,	-0.050779107142857	],
[	4.34077429577465	,	0.101558214285714	],
[	5.75404964788732	,	-1.06636125	],
[	6.6625838028169	,	-1.77726875	],
[	6.56163556338028	,	-2.38661803571429	],
[	5.50167904929578	,	-3.19908375	],
[	5.95594612676056	,	-4.31622410714286	],
[	6.20831672535211	,	-5.94115553571429	],
[	6.20831672535211	,	-5.94115553571429	]]


#sin obstaculos menor a 10
goal_list=[
[	0.858060035211268	,	-0.253895535714286	],
[	1.81706830985916	,	-0.507791071428571	],
[	4.0884036971831	,	-0.660128392857143	],
[	6.35973908450704	,	-1.21869857142857	],
[	6.6625838028169	,	-2.03116428571429	],
[	6.35973908450704	,	-2.69129267857143	],
[	5.85499788732394	,	-2.99596732142857	],
[	6.1578426056338	,	-5.02713160714286	],
[	6.20831672535211	,	-5.94115553571429	],
[	6.20831672535211	,	-5.94115553571429	]]

#Prueba 1 (Meta menor a 10m)
goal_list = [
    [-0.090776199, 0.2],
    [0.774904263, 0],
    [1.793351865, -0.199807381],
    [2.251653286, -0.299759227],
    [2.913644227, -0.499662917],
    [3.728402309, -0.699566608],
    [4.23762611, -0.899470299],
    [4.899617052, -1.049398067],
    [5.663452753, -1.099373989],
    [6.019909414, -1.799036907],
    [5.918064654, -2.248820211],
    [5.765297513, -2.398747979],
    [5.256073712, -3.648146046],
    [4.848694671, -4.597688576],
    [5.612530373, -5.697158875]
]
#Prueba 2 (Meta mayor a 10m)
"""
goal_list=[
#[
    [-0.765680461811723, -1.09942214411248],
    [-0.714758081705151, -1.64915729349736],
    [-0.714758081705151, -2.29884428822496],
    [-0.409223801065719, -2.8985553602812],
    [-0.103689520426288, -3.54833866432337],
    [0.761990941385435, -3.59831458699473],
    [1.52582664298401, -2.39879613356766],
    [1.78043854351687, -2.1489165202109],
    [2.44242948490231, -1.94901282952548],
    [2.59519662522203, -2.04896467486819],
    [4.02102326820604, -2.69865166959578],
    [4.22471278863233, -2.79860351493849],
    [5.44684991119005, -3.59821827768014],
    [5.70146181172291, -3.7481460456942],
    [6.41437513321492, -4.19792934973638],
    [7.27452131438721, -4.39788119507909],
    [7.78374511545293, -3.59807381370826],
    [8.34942557726465, -3.54829050966608],
    [9.01141651865009, -3.24838681898067],
    [9.82617460035524, -2.84867574692443],
    [10.2390879218472, -2.44882021089631],
    [11.0992341030195, -1.24934991212654],
    [11.4556907637655, -1.29932583479789],
    [12.574138365897, -1.4992295254833],
    [13.6944307282416, -1.99898875219684],
    [14.0490426287744, -2.19889244288225],
    [14.0490426287744, -2.19889244288225]
]


#Prueba 3 (Meta mayor a 20m)
goal_list=[
#goal_list = [
    [1.2203, 0.4497],
    [1.7804, 0.9495],
    [2.8498, 1.4493],
    [4.2247, 1.8991],
    [5.0395, 1.9990],
    [5.9561, 2.0490],
    [7.1273, 3.5483],
    [8.2476, 5.1475],
    [9.2170, 6.5969],
    [9.7317, 7.7963],
    [11.4048, 7.7463],
    [12.6269, 6.8967],
    [13.5435, 6.1970],
    [14.2583, 5.5973],
    [15.0822, 5.4374],
    [17.1081, 5.4375],
    [18.7376, 5.2475],
    [19.3996, 5.0476],
    [20.2634, 4.5278],
    [21.2328, 3.9081],
    [22.6986, 2.9986],
    [23.8808, 2.1989],
    [25.1538, 1.5992],
    [25.6630, 1.1994],
    [26.4269, 0.6497],
    [26.8342, 0.0000]
]

"""
#Prueba 4 (Meta mayor a 20m)
goal_list = [
    [-1.016602841918295, 1.34939806678383],
    [-0.35461190053286, 2.04906098418278],
    [-0.201844760213144, 2.64877205623902],
    [0.561990941385435, 2.99860351493849],
    [1.22398188277087, 3.44838681898067],
    [2.49704138543517, 3.9481460456942],
    [3.36272184724689, 5.00064042179262],
    [3.36272184724689, 6.09711072056239],
    [3.36179946714032, 7.24655694200352],
    [3.31179946714032, 8.39595500878735],
    [3.25364422735346, 9.49528084358524],
    [3.20272184724689, 10.9947992970123],
    [3.26272184724689, 11.3945585237258],
    [3.18179946714032, 12.4940288224956],
    [3.25272184724689, 13.3935713532513],
    [3.25272184724689, 14.293113884007],
    [3.25272184724689, 15.34270456942],
    [3.30272184724689, 15.99915641476],
    [4.27932468916519, 16.1923915641476],
    [5.45053943161634, 16.1423193321617],
    [6.76898703374778, 16.2523193321617],
    [7.64020177619893, 16.252295254833],
    [8.40403747779751, 16.252367486819],
    [9.37156269982238, 16.092367486819],
    [10.1863207815275, 15.8925360281195],
    [10.7464669626998, 15.8426564147627],
    [11.5093802841918, 15.1928008787346],
    [12.2741383658970, 14.6429934973638],
    [12.8342845470693, 14.4430898066784],
    [13.6490426287744, 14.1932101933216],
    [14.1073440497336, 13.9933065026362],
    [14.9730245115453, 13.543523198594],
    [15.4313259325044, 13.2936435852373],
    [16.0423944937833, 13.0937880492091],
    [16.8062301953819, 12.6939565905097],
    [17.6719106571936, 12.0942214411248],
    [18.4866687388988, 11.7443899824253],
    [18.4940477797513, 11.7943177504394],
    [19.1069611012433, 12.9438602811951],
    [19.4597282415631, 13.7434268892794],
    [19.4597282415631, 13.7434268892794]
]









datos=[]
class GazeboEnv:
    """Superclass for all Gazebo environments."""

    def __init__(self, launchfile, environment_dim):
        self.environment_dim = environment_dim
        self.odom_x = 0
        self.odom_y = 0
        self.i=0
        self.goal_x = 0
        self.goal_y = 0
        self.collision=True

        self.upper = 5.0+10
        self.lower = -5.0+10
        self.velodyne_data = np.ones(self.environment_dim) * 10
        self.last_odom = None

        self.set_self_state = ModelState()
        self.set_self_state.model_name = "turtlebot3_burger"
        self.set_self_state.pose.position.x = 0.0
        self.set_self_state.pose.position.y = 0.0
        self.set_self_state.pose.position.z = 0.0
        self.set_self_state.pose.orientation.x = 0.0
        self.set_self_state.pose.orientation.y = 0.0
        self.set_self_state.pose.orientation.z = 0.0
        self.set_self_state.pose.orientation.w = 1.0
        """
        self.gaps = [[0 - 0.03, 0 + np.pi / self.environment_dim]]
        for m in range(0,int(self.environment_dim/2-2)):
            self.gaps.append(
                [self.gaps[m][1], self.gaps[m][1] + np.pi / self.environment_dim]
            )

        self.gaps.append([3*np.pi/2,3*np.pi/2+np.pi / self.environment_dim])
        for m in range(len(self.gaps)-1 ,self.environment_dim ):
            self.gaps.append(
                [self.gaps[m][1], self.gaps[m][1] + np.pi / self.environment_dim]
            )
        """
        self.gaps = [[3*np.pi/2, 3*np.pi/2 + np.pi / self.environment_dim]]
        for m in range(0,int(self.environment_dim/2-1)):
            print(m)
            self.gaps.append(
                [self.gaps[m][1], self.gaps[m][1] + np.pi / self.environment_dim]
            )
        
        self.gaps.append([0-0.03, 0 + np.pi / self.environment_dim])
        for m in range(len(self.gaps)-1 ,self.environment_dim-1 ):
            print(m)
            self.gaps.append(
                [self.gaps[m][1], self.gaps[m][1] + np.pi / self.environment_dim]
            )
        print(len(self.gaps))
        #print('*************************************')
        self.gaps[-1][-1] += 0.03
        port = "11311"
        subprocess.Popen(["roscore", "-p", port])

        print("Roscore launched!")

        # Launch the simulation with the given launchfile name
        rospy.init_node("gym", anonymous=True)
        if launchfile.startswith("/"):
            fullpath = launchfile
        else:
            fullpath = os.path.join(os.path.dirname(__file__), "assets", launchfile)
        if not path.exists(fullpath):
            raise IOError("File " + fullpath + " does not exist")

        subprocess.Popen(["roslaunch", "-p", port, fullpath])
        print("Gazebo launched!")

        # Set up the ROS publishers and subscribers
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.set_state = rospy.Publisher(
            "gazebo/set_model_state", ModelState, queue_size=10
        )
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.reset_proxy = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        self.publisher = rospy.Publisher("goal_point", MarkerArray, queue_size=3)
        self.publisher2 = rospy.Publisher("linear_velocity", MarkerArray, queue_size=1)
        self.publisher3 = rospy.Publisher("angular_velocity", MarkerArray, queue_size=1)
        self.odom = rospy.Subscriber(
            "odom", Odometry, self.odom_callback, queue_size=1
        )
        self.velodyne = rospy.Subscriber(
            "scan", LaserScan, self.velodyne_callback, queue_size=1
        )


    # Read velodyne pointcloud and turn it into distance data, then select the minimum value for each angle
    # range as state representation
    def velodyne_callback(self, lidar_data):
        self.velodyne_data = np.ones(self.environment_dim) * 10
        angles = np.arange(lidar_data.angle_min, lidar_data.angle_max, lidar_data.angle_increment)
        ranges = np.array(lidar_data.ranges)
        self.lidar_data = np.ones(self.environment_dim) * 10


        angles = np.arange(lidar_data.angle_min, lidar_data.angle_max + lidar_data.angle_increment, lidar_data.angle_increment)
        ranges = np.array(lidar_data.ranges)
        #print(self.gaps)
        #print()
        for i in range(len(angles)):
            if lidar_data.range_min < ranges[i] < lidar_data.range_max:
                # Calcular el ángulo azimutal en coordenadas polares
                angle = angles[i]
                # Actualizar los datos del LiDAR en función de los ángulos de medición
                for j in range(len(self.gaps)):
                    if self.gaps[j][0] <= angles[i] < self.gaps[j][1]:
                        self.lidar_data[j] = min(self.lidar_data[j], ranges[i])
                        break
        self.velodyne_data= self.lidar_data 
        #print(self.velodyne_data)                  
        

    def odom_callback(self, od_data):

        self.last_odom = od_data

    # Perform an action and read a new state
    def preprocess_lidar_distances(self, scan_range):
        return scan_range
    def step(self, action):
        target = False

        # Publish the robot action
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.angular.z = action[1]
        self.vel_pub.publish(vel_cmd)
        self.publish_markers(action)

        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        # propagate state for TIME_DELTA seconds
        time.sleep(TIME_DELTA)

        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            pass
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")
                        
        scan_range = []
        

        done, self.collision, min_laser = self.observe_collision(self.velodyne_data)
        v_state = []
        v_state[:] = self.velodyne_data[:]
        laser_state = [v_state]

        # Calculate robot heading from odometry data
        self.odom_x = self.last_odom.pose.pose.position.x
        self.odom_y = self.last_odom.pose.pose.position.y
        quaternion = Quaternion(
            self.last_odom.pose.pose.orientation.w,
            self.last_odom.pose.pose.orientation.x,
            self.last_odom.pose.pose.orientation.y,
            self.last_odom.pose.pose.orientation.z,
        )
        euler = quaternion.to_euler(degrees=False)
        angle = round(euler[2], 4)

        # Calculate distance to the goal from the robot
        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )

        # Calculate the relative angle between the robots heading and heading toward the goal
        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y
        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))
        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle
        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        # Detect if the goal has been reached and give a large positive reward
        if self.i >= (len(goal_list)):
            
            self.collision = True
            target = True
            self.reset()
            self.i=0
            done = True

        if distance < GOAL_REACHED_DIST:
            target = True
            
            self.change_goal()
            self.i=self.i+1
            done = False

        robot_state = [distance, theta, action[0], action[1]]
        state = np.append(laser_state, robot_state)
        reward = self.get_reward(target, self.collision, action, min_laser,distance)
        return state, reward, done, target

    def reset(self):
        
        

        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service("/gazebo/reset_world")
        try:
            self.reset_proxy()

        except rospy.ServiceException as e:
            print("/gazebo/reset_simulation service call failed")
        time.sleep(7)
        sleep_time = 7  # en segundos
        rospy.loginfo("Esperando %d segundos...", sleep_time)
        angle = np.random.uniform(-np.pi, np.pi)
        quaternion = Quaternion.from_euler(0.0, 0.0, 0*1*np.pi/2)
        object_state = self.set_self_state
        if self.collision is True:
            x=0
            y=0
            datos.append(self.i)

            # Nombre del archivo CSV
            nombre_archivo = 'sin_obs_m20_11_2.csv'

            # Abrir el archivo CSV en modo escritura para borrar contenido previo
            with open(nombre_archivo, mode='w', newline='') as archivo:
                escritor = csv.writer(archivo)

                # Escribir los datos en una columna
                for dato in datos:
                    escritor.writerow([dato])  # Cada dato en una nueva fila

            # Cerrar el archivo
            archivo.close()
            self.i=0
        else:
            x = self.goal_x
            y = self.goal_y
            x = self.last_odom.pose.pose.position.x        
            y = self.last_odom.pose.pose.position.y
        position_ok = False
        while not position_ok:
            #x = np.random.uniform(-4.5, 4.5)
            #y = np.random.uniform(-4.5, 4.5)
            position_ok = check_pos(x, y)
        object_state.pose.position.x = x
        object_state.pose.position.y = y
        object_state.pose.position.z = 0.12

        object_state.pose.orientation.x = quaternion.x
        object_state.pose.orientation.y = quaternion.y
        object_state.pose.orientation.z = quaternion.z
        object_state.pose.orientation.w = quaternion.w
        self.set_state.publish(object_state)

        self.odom_x = object_state.pose.position.x
        self.odom_y = object_state.pose.position.y
        
        # set a random goal in empty space in environment
        
        
        # randomly scatter boxes in the environment
        #self.random_box()
        #self.publish_markers([0.0, 0.0])

        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        time.sleep(TIME_DELTA)

        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")
        v_state = []
        v_state[:] = self.velodyne_data[:]
        laser_state = [v_state]

        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )

        
        

        self.change_goal()
        self.i=0
        

        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y

        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))

        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle

        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta
        
        robot_state = [distance, theta, 0.0, 0.0]
        state = np.append(laser_state, robot_state)
        return state

    def change_goal(self):
        # Place a new goal and check if its location is not on one of the obstacles
        print(self.i)

        if self.upper < 10 +10:
            self.upper += 0.004
        if self.lower > -10+10:
            self.lower -= 0.004

        goal_ok = False

        while not goal_ok:

            #self.goal_x = self.odom_x + random.uniform(self.upper, self.lower)
            #self.goal_y = self.odom_y + random.uniform(self.upper, self.lower)
            self.goal_x =goal_list[self.i][0]-0.1
            self.goal_y =goal_list[self.i][1]+0.1
            # self.odom_x + #random.uniform(self.upper, self.lower)
            # self.odom_y + #random.uniform(self.upper, self.lower)
            print(self.goal_x,self.goal_y)

            goal_ok = check_pos(self.goal_x, self.goal_y)

    def random_box(self):
        # Randomly change the location of the boxes in the environment on each reset to randomize the training
        # environment
        for i in range(4):
            name = "cardboard_box_" + str(i)

            x = 0
            y = 0
            box_ok = False
            while not box_ok:
                x = np.random.uniform(-6, 6)
                y = np.random.uniform(-6, 6)
                box_ok = check_pos(x, y)
                distance_to_robot = np.linalg.norm([x - self.odom_x, y - self.odom_y])
                distance_to_goal = np.linalg.norm([x - self.goal_x, y - self.goal_y])
                if distance_to_robot < 1.5 or distance_to_goal < 1.5:
                    box_ok = False
            box_state = ModelState()
            box_state.model_name = name
            box_state.pose.position.x = x
            box_state.pose.position.y = y
            box_state.pose.position.z = 0.0
            box_state.pose.orientation.x = 0.0
            box_state.pose.orientation.y = 0.0
            box_state.pose.orientation.z = 0.0
            box_state.pose.orientation.w = 1.0
            self.set_state.publish(box_state)

    def publish_markers(self, action):
        # Publish visual data in Rviz
        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
        marker.pose.position.z = 0

        markerArray.markers.append(marker)

        self.publisher.publish(markerArray)

        markerArray2 = MarkerArray()
        marker2 = Marker()
        marker2.header.frame_id = "odom"
        marker2.type = marker.CUBE
        marker2.action = marker.ADD
        marker2.scale.x = abs(action[0])
        marker2.scale.y = 0.1
        marker2.scale.z = 0.01
        marker2.color.a = 1.0
        marker2.color.r = 1.0
        marker2.color.g = 0.0
        marker2.color.b = 0.0
        marker2.pose.orientation.w = 1.0
        marker2.pose.position.x = 5
        marker2.pose.position.y = 0
        marker2.pose.position.z = 0

        markerArray2.markers.append(marker2)
        self.publisher2.publish(markerArray2)

        markerArray3 = MarkerArray()
        marker3 = Marker()
        marker3.header.frame_id = "odom"
        marker3.type = marker.CUBE
        marker3.action = marker.ADD
        marker3.scale.x = abs(action[1])
        marker3.scale.y = 0.1
        marker3.scale.z = 0.01
        marker3.color.a = 1.0
        marker3.color.r = 1.0
        marker3.color.g = 0.0
        marker3.color.b = 0.0
        marker3.pose.orientation.w = 1.0
        marker3.pose.position.x = 5
        marker3.pose.position.y = 0.2
        marker3.pose.position.z = 0

        markerArray3.markers.append(marker3)
        self.publisher3.publish(markerArray3)

    @staticmethod
    def observe_collision(laser_data):
        # Detect a collision from laser data
        min_laser = min(laser_data)
        if min_laser < COLLISION_DIST:
            return True, True, min_laser
        return False, False, min_laser

    @staticmethod
    def get_reward(target, collision, action, min_laser,distance):
        if target:
            return 100.0
        elif collision:
            return -100.0-distance
        else:
            r3 = lambda x: 1 - x if x < 1 else 0.0
            return action[0] / 2 - abs(action[1]) / 2 - r3(min_laser) / 2
