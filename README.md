# AgileRobots-IndustrialAutomationCompetition2023
Performed ARIAC in ENPM-663(Building a Manufacturing Robotics Software System) course.
The repository was made to execute multiple challenges part of ARIAC competition - https://ariac.readthedocs.io/en/latest/
The package executes the following challenges:
1. Faulty parts 
2. Insufficient parts
3. Order priority 
4. Picking and Placing parts 

Dependencies: 
1. ROS2-galactic (Along with Gazebo and RViz)
2. Moveit 2

The Package made is based on https://github.com/usnistgov/ARIAC/. The services provided in the competition environment are accessed to complete the tasks given.

Method to build the required packages:

-> colcon build --packages-select competitor_interfaces
-> colcon build --packages-select group6
-> colcon build --packages-select rqt_joint_trajectory_controller

Method to run the package:

To run the package use the following commands strictly in the given order:

-> ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa4 competitor_pkg:=group6 sensor_config:=group6_sensors
-> ros2 launch ariac_moveit_config ariac_robots_moveit.launch.py
-> ros2 launch group6 group6.launch.py

