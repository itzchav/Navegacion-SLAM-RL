# Navegacion con Aprendizaje por Refuerzo
Basado en el [repositorio](https://github.com/reiniscimurs/DRL-robot-navigation) de Reinis Cimurs https://github.com/reiniscimurs/DRL-robot-navigation

En el repositorio se encuentran los pasos necesarios para la intalación.


Para clonar este repositorio:
```shell
$ cd ~
### Clone this repo
$ git clone https://github.com/itzchav/Navegacion-con-Aprendizaje-por-Refuerzo.git
```



En la terminal ejecutar los siguientes comandos:
```shell
$ export ROS_HOSTNAME=localhost
$ export ROS_MASTER_URI=http://localhost:11311
$ export ROS_PORT_SIM=11311
$ export GAZEBO_RESOURCE_PATH=~/DRL-robot-navigation/catkin_ws/src/multi_robot_scenario/launch
$ source ~/.bashrc
$ cd ~/DRL-robot-navigation/catkin_ws
$ source devel_isolated/setup.bash
```

Para ejecutar el entrenamiento:
```shell
$ cd ~/DRL-robot-navigation/TD3
$ python3 train_turtlebot.py.py
```



Para terminar el proceso:
```shell
$ killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient python python3
```

Para ejecutar el código de prueba: 
```shell
$ cd ~/DRL-robot-navigation/TD3
$ python3 test_velodyne_td3.py
```


Para ejecutar el código de Navegación con trayectoria (off-line): 

Para cambiar el entorno

Sin obtaculos:
```shell
env = GazeboEnv("plano_completo_obstaculos_rviz.launch", environment_dim)
```
Con obtaculos:
```shell
env = GazeboEnv("plano_completo.launch", environment_dim)
```

```shell
$ cd ~/DRL-robot-navigation/TD3
$ python3 test_plano_offline.py
```

Para ejecutar el código de Navegación con trayectoria (on-line): 
Modificar el enviroment que contiene SLAM

Para el entorno sin obstaculos:
```shell
env = GazeboEnv("plano_completo_gmapping_noetic.launch", environment_dim)
```
Para el entorno con obstaculos:
```shell
env = GazeboEnv("plano_completo_obstaculos_noetic.launch", environment_dim)
```

```shell
$ cd ~/DRL-robot-navigation/TD3
$ python3 test_plano_online.py
```

Gazebo environment:
<p align="center">
    <img width=80% src="https://github.com/itzchav/Navegacion-con-Aprendizaje-por-Refuerzo/blob/main/turtle_cimurs.png">
</p>

Rviz:
<p align="center">
    <img width=80% src="https://github.com/reiniscimurs/DRL-robot-navigation/blob/main/velodyne.png">
</p>

