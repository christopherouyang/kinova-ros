# This package runs a simulation of the UR robot in a car 
# the UR robot can be controlled with the velocity/position-resolved controllers from the standard pr2 control manager
# the car can move in the direcetion of x and y in a planner.


test environment:ubuntu16.04

In order to run this code, install the following packages beforehand:
 
```
$ sudo apt-get install ros-kinetic-pr2-mechanism-model ros-kinetic-pr2-controller-manager ros-kinetic-control-toolbox ros-kinetic-pr2-mechanism-controllers
```

after you install the dependencies and clone the code in your workspace,you can run the following lines of code



```
$ roslaunch arm_car_rviz_simulation ur10_car_simulation.launch
```

To test the simulation, you can manually move the UR robot in **velocity control mode** like so:

```
$ rostopic pub -r 10 /amm_vel/command iai_control_msgs/MultiJointVelocityImpedanceCommand '{velocity:[0.05,0,0,0,0,0,0,0,0]}'
```

Here you are commanding the first joint with a velocity of 1rad/s

To test the UR robot simulation in **position control mode** do the following:

```
$ rostopic pub -r 10 /amm_pos_controller/command std_msgs/Float32MultiArray '{data:[0,0,0,0,0,0,0,0,0]}'

```


