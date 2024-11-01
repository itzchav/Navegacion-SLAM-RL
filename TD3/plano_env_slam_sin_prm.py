import math
import os
import random
import subprocess
import time
from os import path

import signal
import psutil

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty


from tf.transformations import euler_from_quaternion
import tf

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from squaternion import Quaternion
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from collections import defaultdict
from math import sin, cos, pi, sqrt


import subprocess
import sys

import cv2
import networkx as nx

from multiprocessing import Process
from std_msgs.msg import Float32



GOAL_REACHED_DIST = 0.2


COLLISION_DIST = 0.18
TIME_DELTA = 0.1


def publish_markers(self):
        # Publish visual data in Rviz
        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = marker.CYLINDER
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.punto_desado_xh
        marker.pose.position.y = self.punto_desado_yh
        marker.pose.position.z = 0
        
        markerArray.markers.append(marker)
        self.publisher.publish(markerArray)


def verificar_region_blanca(imagen_binaria, x_pixeles, y_pixeles):
    if 0 <= x_pixeles < imagen_binaria.shape[1] and 0 <= y_pixeles < imagen_binaria.shape[0]:
        if imagen_binaria[int(y_pixeles), int(x_pixeles)] == 255:
            imagen_binaria[int(y_pixeles), int(x_pixeles)] == 0
            return True  # El punto esta en una region blanca
    return False  # El punto esta fuera de la imagen o en una region negra

def metros_a_pixeles(self, x, y):  
    pixel_x=self.pixel_origen_x+round(x/self.resx)
    pixel_y=self.pixel_origen_y-round(y/self.resy)
    return pixel_x, pixel_y


def pixeles_a_metros(self, pixel_x, pixel_y):
    y=(-self.origen_y+self.y_max+self.origen_global_y)
    metros_x =  self.origen_global_x + (pixel_x) * self.resx 
    metros_y =  y-(pixel_y) * self.resy     
    return metros_x, metros_y

def get_occupancy(image, pos):
    # Verificar si el píxel está dentro de los límites de la imagen
    if 0 <= pos[1] < image.shape[0] and 0 <= pos[0] < image.shape[1]:
        # Obtener el valor del píxel en la posición dada
        pixel_value = image[pos[1], pos[0]]
        # Devuelve 0 si el píxel es blanco (espacio libre), 1 si no lo es.
        return 0 if np.all(pixel_value == 255) else 1
    return 1


def dibujar_grafo(imagen_color, grafo, puntos):
    # Copiar la imagen original para dibujar sobre ella
    imagen_dibujo = imagen_color.copy()
    # Dibujar nodos
    for punto in puntos:
        cv2.circle(imagen_dibujo, (int(punto[1]), int(punto[0])), 5, (0, 255, 255), -1)  # Amarillo
    # Dibujar aristas
    for (p1, p2, data) in grafo.edges(data=True):
        cv2.line(imagen_dibujo, (int(p1[1]), int(p1[0])), (int(p2[1]), int(p2[0])), (255, 0, 0), 1)  # Rojo
    return imagen_dibujo

def encontrar_pixel_blanco_mas_cercano(image, x, y):
    # Verificar si el píxel está en una zona blanca
    if image[x,y] == 255:
        return x, y#, True    
    radio = 2
    while True:
        for i in range(-radio, radio + 1):
            for j in range(-radio, radio + 1):
                x_vecino = x + i
                y_vecino = y + j
                if 0 <= x_vecino < image.shape[0] and 0 <= y_vecino < image.shape[1]:
                    if image[x_vecino, y_vecino] == 255:
                        # Mostrar el punto blanco encontrado en la imagen
                        image[x_vecino,y_vecino] = 180                       
                        return x_vecino, y_vecino#, True   
        radio += 1
        # Si no se encontró un píxel blanco en el radio actual, continuar con un mayor radio de búsqueda.
        if radio > max(image.shape):
            break
    # Si no se encontró ningún píxel blanco cercano, devolver el punto original y bandera falsa.
    return x, y#, False


def generar_puntos_aleatorios_radio(self, imagen_binaria, imagen_color):
    pixeles_aleatorios = []
    self.distancia_minima = 40
    self.distancia_min_puntos_ruta = 15
    pcm = None
    distancia_pcm = None
    
    self.puntos_guardados_pixeles= [metros_a_pixeles(self, px, py) for px, py in self.centros]
    #self.puntos_ruta_pixeles= [metros_a_pixeles(self, px, py) for px, py in self.ruta]

    for (px, py) in self.puntos_guardados_pixeles:
        imagen_color[py, px] = [255, 105, 180]

    for _ in range(self.nodos):
        while True:
            rand_x = random.randint(0, imagen_binaria.shape[1] - 1)
            rand_y = random.randint(0, imagen_binaria.shape[0] - 1)
            # Verifica si el pixel en las coordenadas es blanco
            if imagen_binaria[rand_y, rand_x] == 255:
                odom_px, odom_py=metros_a_pixeles(self, self.odom_x, self.odom_y)
                
                distancia_al_centro = distancia_entre_puntos((rand_x, rand_y), (odom_px, odom_py))
                if distancia_al_centro <= 70:
                    # Calcula la distancia mínima con todos los puntos guardados
                    distancias = [distancia_entre_puntos((rand_x, rand_y), (y,x)) for y,x in self.puntos_guardados_pixeles]
                    # Verifica si la distancia mínima es al menos 40 píxeles de cada punto guardado
                    if all(distancia >= self.distancia_minima for distancia in distancias) :#and all(distancia >= self.distancia_min_puntos_ruta for distancia in distancia_2):
                        # El punto es válido
                        imagen_color[rand_y, rand_x] = [0, 100, 255]  # Amarillo
                        pixeles_aleatorios.append((rand_x, rand_y))
                        x_metros, y_metros = pixeles_a_metros(self, rand_x, rand_y)
                        distancia = distancia_entre_puntos((x_metros, y_metros), (self.meta_x, self.meta_y))
                        
                        if pcm is None or distancia < distancia_pcm:
                            pcm = (rand_x, rand_y)
                            distancia_pcm = distancia
                        break

    if pcm is not None:
        imagen_color[pcm[1], pcm[0]] = [0, 255, 0]  # Verde

    return pcm





def distancia_entre_puntos(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def hay_obstaculo_entre_puntos(punto1, punto2, imagen_binaria):
    # Verificar si hay obstáculos entre dos puntos utilizando la línea entre ellos
    linea = np.linspace(punto1, punto2, num=10).astype(int)
    for punto in linea:
        if imagen_binaria[punto[0], punto[1]] == 0:
            return True
    return False

def generar_muestras_validas(imagen_binaria, numero_de_muestras):
    # Obtener las coordenadas de los puntos blancos en la imagen binaria
    puntos_validos = np.column_stack(np.where(imagen_binaria == 255))  # Puntos blancos
    
    # Seleccionar aleatoriamente un número específico de muestras
    indices_muestras = np.random.choice(len(puntos_validos), numero_de_muestras, replace=False)
    muestras = puntos_validos[indices_muestras]
    
    # Convertir la imagen binaria a una imagen en color (BGR)
    imagen_color = cv2.cvtColor(imagen_binaria, cv2.COLOR_GRAY2BGR)
    
    # Marcar los puntos de muestra en amarillo (255, 255, 0) en la imagen en color
    for (y, x) in muestras:
        imagen_color[y, x] = [0, 255, 255]  # Color amarillo en formato BGR
        
    return muestras


def distancia_entre_puntos(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)



def check_pos(x, y):
    goal_ok = True
    return goal_ok


class GazeboEnv:
    """Superclass for all Gazebo environments."""

    def __init__(self, launchfile, environment_dim):
        self.environment_dim = environment_dim
        self.odom_x = 0
        self.odom_y = 0
        self.i=0
        self.goal_x = 0.01
        self.goal_y = 0.01
        self.collision=True
        self.contar_metas=0

        self.upper = 5.0+10
        self.lower = -5.0+10
        self.num_img=0
        self.velodyne_data = np.ones(self.environment_dim) * 10
        self.last_odom = None
        self.lectura=0
        self.contador=0
        self.limite=2
        self.nuevo_pd=[0,0]
        self.centros = []  # Vector para almacenar los puntos generados

        self.numero_de_imagen=0
        self.num_new_img=0
        self.nodos=30


        self.meta_x=9
        self.meta_y=-10.5
        

        self.set_self_state = ModelState()
        self.set_self_state.model_name = "turtlebot3_burger"
        self.set_self_state.pose.position.x = 0.0
        self.set_self_state.pose.position.y = 0.0
        self.set_self_state.pose.position.z = 0.0
        self.set_self_state.pose.orientation.x = 0.0
        self.set_self_state.pose.orientation.y = 0.0
        self.set_self_state.pose.orientation.z = 0.0
        self.set_self_state.pose.orientation.w = 1.0
        self.map_file = "map"

        self.pixeles_anterior=0
        self.bandera_espacios=False

        

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
            print("fullpath")
            print(fullpath)
        if not path.exists(fullpath):
            raise IOError("File " + fullpath + " does not exist")

        subprocess.Popen(["roslaunch", "-p", port, fullpath])
        print("Gazebo launched!")

        time.sleep(5)
        sleep_time = 5  # en segundos
        rospy.loginfo("Esperando %d segundos...", sleep_time)
        
        


        

        # Set up the ROS publishers and subscribers
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        self.listener = tf.TransformListener()
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
        self.reward_pub = rospy.Publisher('reward', Float32, queue_size=10)
        
        

       

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
               
        

    def odom_callback(self, od_data):
        print("X={0} Y={1}".format(self.odom_x,self.odom_y))
        print("Xhd={0} Yhd={1}".format(self.goal_x,self.goal_y))

        self.last_odom = od_data

    def map_callback(self, msg):
        # Establecer un flag para indicar que estamos procesando el mapa
        self.processing_map = True

        try:
            # Obtener la resolución, origen y dimensiones del mapa
            resolution = msg.info.resolution
            origen_x = msg.info.origin.position.x
            self.origen_y = msg.info.origin.position.y
            self.width = msg.info.width
            self.height = msg.info.height

            # Dimensiones del mapa en metros
            self.x_min = origen_x
            self.x_max  = origen_x + (self.width * resolution)
            self.y_min = self.origen_y
            self.y_max = self.origen_y + (self.height * resolution)

            # Crear un listener de tf
            time.sleep(0.5)    # Pause 5.5 seconds

            listener = tf.TransformListener()

            # Esperar hasta que la transformación de /map a /base_link esté disponible
            listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(3.0))
            
            # Obtener la transformación de /map a /base_link
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            
            # La posición del robot en el marco de referencia /map
            self.robot_x = trans[0]
            self.robot_y = trans[1]
            
            
            if self.lectura == 0:
                self.punto_desado_xh = self.robot_x
                self.punto_desado_yh = self.robot_y

                self.plo_x = self.robot_x
                self.plo_y = self.robot_y

                self.inicio_x = self.robot_x
                self.inicio_y = self.robot_y

                self.plo_en_metros=[self.robot_x,self.robot_y]
                self.path_metros=[(self.robot_x,self.robot_y),(self.robot_x,self.robot_y)]
                

            # Esperar hasta que la transformación de /map a /odom esté disponible
            listener.waitForTransform('/map', '/odom', rospy.Time(0), rospy.Duration(1.0))
            
            # Obtener la transformación de /map a /odom
            (trans, rot) = listener.lookupTransform('/odom', '/map', rospy.Time(0))
            
            # Calcular el Local Origin en el marco global /odom
            self.origen_global_x = origen_x + trans[0]
            self.origen_global_y = self.origen_y + trans[1]

            self.resx = (abs(self.x_min) + abs(self.x_max )) / self.width
            self.resy = (abs(self.y_min) + abs(self.y_max)) / self.height


            self.pixel_origen_x=round(abs(self.origen_global_x/self.resx))#width-round(abs(origen_x/resx))
            self.pixel_origen_y=self.height-round(abs(self.origen_global_y/self.resy))#height-round(abs(origen_y/resy))
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Error de transformación: %s", str(e))
        
        finally:
            # Marcar que hemos terminado de procesar el mapa
            self.processing_map = False



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
       

        distance_meta = np.linalg.norm(
            [self.odom_x - self.meta_x, self.odom_y - self.meta_y]
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
        if distance < GOAL_REACHED_DIST:
            target = True
            self.i=self.i+1
            print(self.i)
            print("meta")

            self.change_goal()
            print("genere bien")
            done=False
        
        
        
        
        
            
        
        if distance_meta < GOAL_REACHED_DIST:
            done=True
            self.reset()
        
        



        robot_state = [distance, theta, action[0], action[1]]
        state = np.append(laser_state, robot_state)
        reward = self.get_reward(target, self.collision, action, min_laser,distance)
        return state, reward, done, target
    

    
            

    def reset(self):
        

        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service("/gazebo/reset_world")
        #rospy.init_node('reset_map_node', anonymous=True)
        

        #self.map.publish(None)
        
    
        
        try:
            self.reset_proxy()

        except rospy.ServiceException as e:
            print("/gazebo/reset_simulation service call failed")
            

        

        angle = np.random.uniform(-np.pi, np.pi)
        quaternion = Quaternion.from_euler(0.0, 0.0, angle)
        object_state = self.set_self_state
        if self.collision is True:
            x=0
            y=0
            self.i=0

        else:
            x = self.goal_x
            y = self.goal_y
            x = self.last_odom.pose.pose.position.x        
            y = self.last_odom.pose.pose.position.y
        position_ok = False
        while not position_ok:

            position_ok = check_pos(x, y)
        object_state.pose.position.x = x
        object_state.pose.position.y = y
        # object_state.pose.position.z = 0.
        object_state.pose.orientation.x = quaternion.x
        object_state.pose.orientation.y = quaternion.y
        object_state.pose.orientation.z = quaternion.z
        object_state.pose.orientation.w = quaternion.w
        self.set_state.publish(object_state)

        self.odom_x = object_state.pose.position.x
        self.odom_y = object_state.pose.position.y

            

        print("lectura")
        print(self.lectura)

        if self.lectura is not 0:
            self.punto_desado_xh = self.robot_x
            self.punto_desado_yh = self.robot_y

            self.plo_x = self.robot_x
            self.plo_y = self.robot_y

            self.inicio_x = self.robot_x
            self.inicio_y = self.robot_y

            self.plo_en_metros=[self.robot_x,self.robot_y]
            self.path_metros=[(self.robot_x,self.robot_y),(self.robot_x,self.robot_y)]
        
    
        # set a random goal in empty space in environment
        self.leer_imagen()
        self.change_goal()
        self.i=0
        self.lectura=1
        
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

        print("fin")
        return state

    def leer_imagen(self):
            
            # Comando a ejecutar en segundo plano
            
            nombre_imagen = "imagen" #+ str(self.num_img)
            command = "rosrun map_server map_saver -f " + nombre_imagen 

            # Ejecutar el comando y capturar la salida
            process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            process.wait()


    def change_goal(self):
        self.contar_metas=self.contar_metas+1
        print("contar metas")
        print(self.contar_metas)


        # Obtén la carpeta personal del usuario
        carpeta_raiz = os.path.expanduser("~")

        # Define la ruta a la carpeta "pruebas"
        carpeta_pruebas = os.path.join(carpeta_raiz, 'Pruebas_RL')
        carpeta = os.path.join(carpeta_pruebas, 'Prueba_3_entrada_aula1')

        
        if not os.path.exists(carpeta):
            os.makedirs(carpeta)


        nombre_imagen = "imagen" #+ str(self.num_img)
        command = "rosrun map_server map_saver -f " + nombre_imagen 

        # Ejecutar el comando y capturar la salida
        process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        process.wait()


        #Obtener imagen del mapa
        print("plo")
        
        
        #self.distancia_k_vecinos=self.distancia_k_vecinos+1
        ruta_completa = os.path.join(carpeta, self.map_file)
    
        cmd = ["rosrun", "map_server", "map_saver", "-f", ruta_completa]
        proc = subprocess.Popen(cmd)
        proc.wait()
        
        # Carga la imagen del mapa
        imagen_original = cv2.imread(ruta_completa+ '.pgm', cv2.IMREAD_GRAYSCALE)
        umbral = 240  
        _, imagen_binaria = cv2.threshold(imagen_original, umbral, 255, cv2.THRESH_BINARY)
        
        # Guardar la imagen_original binarizada
        nombre_imagen = "imagen_binarizada" + str(self.numero_de_imagen)
        nombre_imagen_con_extension = nombre_imagen + ".png"
        ruta_completa = os.path.join(carpeta, nombre_imagen_con_extension)

        cv2.imwrite(ruta_completa, imagen_binaria)    
        
        self.lectura=1
        self.nodos=self.nodos+5
        if self.contador > self.limite:
            self.fin=True
        
        print(self.contador)
        print("contador")

        kernel = np.ones((9,9),np.uint8)
        img_bin = cv2.erode(imagen_binaria,kernel,iterations = 1)

        kernel = np.ones((16,16),np.uint8)
        img = cv2.erode(imagen_binaria,kernel,iterations = 1)

        kernel = np.ones((17,17),np.uint8)
        imagen_reducida = cv2.erode(imagen_binaria,kernel,iterations = 1)

        imagen_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        imagen_color2 = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        imagen_origen = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        sp,j=metros_a_pixeles(self,self.robot_x,self.robot_y)

        imagen_origen[self.pixel_origen_y,self.pixel_origen_x] = [0, 0, 255]  # Morado

        ruta_completa = os.path.join(carpeta, "inicio.jpg")
        cv2.imwrite(ruta_completa, imagen_origen)
                            
        xd,yd=metros_a_pixeles(self,self.meta_x,self.meta_y)
        print(xd,yd)
        if verificar_region_blanca(img_bin,xd,yd):
                
            # try:q
                # Si esta en un area blanca, simplemente devuelve el punto deseado  
                self.goal_x=self.meta_x
                self.goal_y=self.meta_y
                
    
                goal_ok = check_pos(self.goal_x, self.goal_y)

                print("LLegue")
                
                
        else:
                #try:
                    xs,ys=metros_a_pixeles(self, self.punto_desado_xh, self.punto_desado_yh)
                    start=(ys,xs)
                    print("No blanco")
                    self.meta=[]

                    imagen_origen[ys,xs] = [0, 100, 255]  # Morado

                    self.plo = generar_puntos_aleatorios_radio(self, imagen_reducida, imagen_color)
                    self.plo_en_metros = pixeles_a_metros(self, self.plo[1], self.plo[0])
                    self.centros.append(self.plo_en_metros)
                    self.plo_x, self.plo_y = pixeles_a_metros(self, self.plo[0], self.plo[1])
                    
                    self.goal_x=self.plo_x
                    self.goal_y=self.plo_y
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
