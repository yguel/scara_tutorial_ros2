# SCARA Tutorial ROS2 Docker Containers
Provides a basic preconfigured docker container for tutorial purposes.

To use it, make sure you have [Docker](https://docs.docker.com/get-docker/) installed, then build and run the image :

```shell
$ docker build --tag scara_tutorial_ros2:humble --file .docker/remote/Dockerfile .
$ docker run scara_tutorial_ros2:humble ros2 launch scara_bringup scara.launch.py
```

### Run with GUI
To run the docker image with GUI, use the [rocker tool](https://github.com/osrf/rocker):
```shell
$ sudo apt install python3-rocker
$ rocker --net=host --x11 scara_tutorial_ros2:humble ros2 launch scara_bringup scara.launch.py
```

### Run with bash
To interact with the environment, run docker using:
```shell
$ docker run -it scara_tutorial_ros2:humble
```
and inside docker run:
```shell
$ cd ros2_dev/scara_tutorial_ros2/
$ source install/setup.bash
$ ros2 launch scara_bringup scara.launch.py
```
The `scara_tutorial_ros2` nodes should now be running.