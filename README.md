# TC_DMP_constrainedVelAcc
An  implementation for Temporal Coupling of Dynamical Movement Primitives for Constrained Velocities and Accelerations

## Matlab ##
Run script main.m

## ROS ##
**1. Build**  
In directory catkin_ws:
```bash
catkin_make
source devel/setup.bash
```

**2. Initate**  
To run kinematic UR10 simulation:
```bash
roslaunch experiment_dmp_tc bringup.launch
```
To run with real UR10 manipulator:
```bash
roslaunch experiment_dmp_tc bringup.launch mode:=urscript robot_ip:=<robot_ip> 
```
where <robot_ip> is the IP of the UR10 robot.

**3. Initialize**  
Open a new terminal
```bash
source devel/setup.bash
roslaunch experiment_dmp_tc init.launch param_file:=<yaml_file>
```
where <yaml_file> is replaced by *tc.yaml* or *no_tc.yaml* depending on if temporal coupling should be used or not.

**4. Start**
```bash
roslaunch experiment_dmp_tc start.launch [bagname:=<filename>]
```
A rosbag \<filename\>.bag is created if the optional parameter bagname is specified, logging the experiment. 
