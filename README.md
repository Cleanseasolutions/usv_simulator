# usv_simulator
A USV simulator for ROS Melodic and Gazebo 9.

Forked from https://bitbucket.org/osrf/vrx

## Installation
See docker instructions.


## Simulate the Otter USV
Alternative 1 - Launch script :
```
./catkin_ws/src/usv_simulator/scripts/run.sh
```

Alternative 1 - lanch manually :
```
roslaunch otter_gazebo otter.launch 
```
Faster than real-time simulation is also available, if your computer is fast enough. Without a dedicated graphics card this is only useful for simulation without gui, i.e. with argument gui:=false. 
```
roslaunch otter_gazebo otter_fast.launch 
```
Check out the launch files for available arguments.

### Control the Otter USV with the keyboard
```
roslaunch otter_gazebo keydrive.launch 
```

### Docker
Clone the repo, then cd into usv_simulator.
To build the docker do
```
docker build --build-arg GIT_USER_EMAIL=$(git config --get user.email) --build-arg GIT_USER_NAME=$(git config --get user.name) -t ros-melodic-gazebo .
```
To run the docker run
```
docker run -it --rm     -e DISPLAY=$DISPLAY     -v /tmp/.X11-unix:/tmp/.X11-unix  -v ~/.ssh:/home/rosuser/.ssh   --memory 2g  --memory-swap 4g --name ros-melodic-container     ros-melodic-gazebo'
```
