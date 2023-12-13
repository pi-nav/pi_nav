# πNAV: Priority Intersection Navigation for Autonomous Vehicles

This project aims to simulate an intersection with multiple autonomous vehicles, including regular traffic crossing continuously and priority vehicles like ambulances needing expedited right of way. The goal is to demonstrate safe and efficient coordination, with regular vehicles detecting and yielding to priority ones. Developing this capability has numerous applications, from enabling faster emergency response to improving warehouse efficiency. Key benefits include increased safety, efficiency, and capability for autonomous vehicles and robots performing critical functions in complex environments alongside human drivers.

# Hardware/Software Requirements

- 3 AgileX LIMOs
- 1 computer installed with Ubuntu 18.04 LTS (or an extra AgileX LIMO) 
- ROS Melodic Morenia

# Installation

In your current ROS workspace directory (or make a new one), check out the repository.

```
git clone https://github.com/pi-nav/pi_nav.git
```

In your workspace, compile the project. 

```
catkin_make --only-pkg-with-deps pi_nav
```

# General Setup

The multiple machines setup steps include general instructions to setup the system to use multiple machines with one computer (or LIMO) running as the master. It may be useful to add the following three environment variable configurations to each LIMO's shell configuration file if this will be running repeatedly (replace ROS_MASTER_IP with the IP address of the machine designated as the master).

```
export ROS_MASTER_URI=http://(ROS_MASTER_IP):11311

export ROS_HOSTNAME=$(hostname -I | awk '{print $1}')

export ROS_IP=  
```

To run a specific scenario, each limo needs to have launched with the specific launch file for that scenario, as well has running the appropriate script. For example, the following commands are used to start Scenario 1.


**Priority vehicle (Vehicle #1)**

```
roslaunch pi_nav pi_nav_priority.launch  

rosrun pi_nav priority_vehicle_sc1.py
```

**Regular Vehicle (Vehicle #2)** 

```  
roslaunch pi_nav pi_nav_regular_2.launch

rosrun pi_nav regular_vehicle_2_sc1.py
```

**Regular Vehicle #2 (Vehicle #3)**

```
roslaunch pi_nav pi_nav_regular_2.launch  

rosrun pi_nav regular_vehicle_2_sc1.py
```

# Multiple Machines Setup

Setup the computer (or extra LIMO) to be the master.

```
export ROS_HOSTNAME=$(hostname -I | awk '{print $1}')  

roscore
```

Setup each vehicle.  

```
export ROS_MASTER_URI=http://(ROS_MASTER_IP):11311 (if not in configuration file)

export ROS_HOSTNAME=$(hostname -I | awk '{print $1}') (if not in configuration file)  

export ROS_IP= (if not in configuration file)  

Source in workspace  
```

Once setup, just launch appropriate scenario launch files and scripts to start the robot.

# Testing Scenarios

For testing and analyzing robot behavior, three scenarios were selected to demonstrate the traffic handling capabilities of the system.

**Scenario 1**

- Two regular robots approach from opposite sides
- Priority vehicle crosses intersection  
- Regular vehicles stop, yield to priority vehicle, then proceed

**Scenario 2**

- Two regular robots approach intersection
- System allows first arrival to exit first, mimicking real-world traffic rules

**Scenario 3**

- First regular vehicle enters intersection first
- Priority vehicle approaches second, stops to avoid collision
- Second regular vehicle must stop, yielding to priority vehicle 
- Combines conditions of Scenario 1 and Scenario 2
