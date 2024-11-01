# Navegacion con Aprendizaje por Refuerzo
Basado en el [repositorio](https://github.com/reiniscimurs/DRL-robot-navigation) de Reinis Cimurs https://github.com/reiniscimurs/DRL-robot-navigation

En el repositorio se encuentran los pasos necesarios para la intalación.


Para clonar este repositorio:
```shell
$ cd ~
### Clone this repo
$ git clone https://github.com/itzchav/Navegacion-con-Aprendizaje-por-Refuerzo.git
```

Compile the workspace:
```shell
$ cd ~/DRL-robot-navigation/catkin_ws
### Compile
$ catkin_make_isolated
```

Open a terminal and set up sources:
```shell
$ export ROS_HOSTNAME=localhost
$ export ROS_MASTER_URI=http://localhost:11311
$ export ROS_PORT_SIM=11311
$ export GAZEBO_RESOURCE_PATH=~/DRL-robot-navigation/catkin_ws/src/multi_robot_scenario/launch
$ source ~/.bashrc
$ cd ~/DRL-robot-navigation/catkin_ws
$ source devel_isolated/setup.bash
```

Run the training:
```shell
$ cd ~/DRL-robot-navigation/TD3
$ python3 train_velodyne_td3.py
```

To check the training process on tensorboard:
```shell
$ cd ~/DRL-robot-navigation/TD3
$ tensorboard --logdir runs
```

To kill the training process:
```shell
$ killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient python python3
```

Once training is completed, test the model:
```shell
$ cd ~/DRL-robot-navigation/TD3
$ python3 test_velodyne_td3.py
```

Gazebo environment:
<p align="center">
    <img width=80% src="https://github.com/reiniscimurs/DRL-robot-navigation/blob/main/env1.png">
</p>

Rviz:
<p align="center">
    <img width=80% src="https://github.com/reiniscimurs/DRL-robot-navigation/blob/main/velodyne.png">
</p>

