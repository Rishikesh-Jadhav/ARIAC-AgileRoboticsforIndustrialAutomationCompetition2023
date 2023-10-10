# ARIAC - Agile Robotics for Industrial Automation Challenge 2023

Performed by NIST (National Institute of Standards and Technology) as part of the ENPM-663 (Building a Manufacturing Robotics Software System) course.

This repository has been created to address various challenges presented in the ARIAC competition. You can learn more about the competition [here](https://ariac.readthedocs.io/en/latest/).

## Table of Contents
1. [Introduction](#introduction)
2. [Competition Overview](#competition-overview)
3. [Key Components](#key-components)
4. [Our Team's Work](#our-teams-work)
5. [Challenges](#challenges)
6. [Architecture](#architecture)
7. [Order Parsing and Processing](#order-parsing-and-processing)
8. [Sensors and Cameras](#sensors-and-cameras)
9. [Difficulties Encountered](#difficulties-encountered)
10. [Dependencies](#dependencies)
11. [Building and Running the Package](#building-and-running-the-package)
12. [Additional Information](#additional-information)

---

## Introduction

ARIAC (Agile Robotics for Industrial Automation Challenge) is an annual robotics competition that aims to advance industrial automation by pushing the capabilities of robotic systems in simulated manufacturing environments. This repository showcases our team's participation and solutions for various challenges within the ARIAC competition.

## Competition Overview

**Objective**: ARIAC challenges participants to develop robotic systems capable of autonomously performing complex tasks in industrial settings. The goal is to create and deploy robots that can efficiently and accurately complete various tasks in a simulated factory environment.

**Simulated Environment**: The competition utilizes ROS2 and the Gazebo simulator, providing participants with a realistic platform to interact with factory elements such as robotic arms, sensors, and conveyor belts.

**Challenges**: ARIAC presents participants with diverse challenges, including material handling, assembly, quality inspection, and process optimization. These challenges test perception, planning, and control capabilities.

**Scoring**: Teams are evaluated based on task completion time, accuracy, and resource utilization. The team with the highest score wins.

## Key Components

ARIAC involves several key components:

1. **Simulation Environment**: The factory environment is simulated using Gazebo, providing a realistic testing and development platform. We developed competition control software to operate within this simulation.

2. **Tasks and Challenges**: ARIAC includes various tasks like picking and placing objects, assembling components, inspecting products, and optimizing processes.

3. **ROS Control Interfaces**: We used ROS2 to interface with and control robots and sensors. ROS2 offers a standardized framework for robotic software development.

4. **Scoring System**: A scoring system calculates team performance based on task completion, efficiency, and accuracy, determining competition winners.

## Our Team's Work

Our team, composed of Rishikesh Jadhav, Vishal Sivakumar, Abhinav Garg, Nishant Pandey, and Saketh Banagiri, participated in the 2023 ARIAC competition as "Group6." Here's an overview of our work:

- **Team Name**: Group6
- **Team Members**:
  - Rishikesh Jadhav
  - Vishal Sivakumar
  - Abhinav Garg
  - Nishant Pandey
  - Saketh Banagiri

### Highlights of Our Work

#### Task 1: Material Handling

Our robotic system excelled in dynamically picking and placing objects from conveyor belts and bins, demonstrating advanced control algorithms for kitting tasks with precision and efficiency.

#### Task 2: Assembly

We achieved exceptional accuracy in the assembly task, where the gantry robot assembled complex components. Real-time feedback using sensors and meticulous planning contributed to our success.

#### Task 3: Quality Inspection

Our robotic vision system performed admirably in quality inspection. It proactively identified defective parts, replaced them, and ensured task completion without compromising quality.

## Challenges

The ARIAC competition presented several challenges, each requiring specific solutions. Here's an overview of the challenges we faced:

1. **Faulty Parts Challenge**: Detecting and handling unsuitable parts by initiating quality checks to ensure high-quality kits.

2. **High-priority Orders Challenge**: Fulfilling high-priority orders before low-priority ones by identifying and prioritizing tasks promptly.

3. **Insufficient Parts Challenge**: Addressing situations with a lack of parts by utilizing alternative components to complete orders without delays.

4. **Flipped Parts Challenge**: Correcting upside-down parts by using both gantry and floor robots.

5. **Dynamic Pick and Place from Conveyor Belt**: Efficiently picking parts from a moving conveyor belt by considering relative velocity between the floor robot and conveyor.

## Architecture

Our project architecture revolves around pick-and-place tasks, including kitting and assembly, with the possibility of combined tasks:

- **Kitting**: Identifying parts on a conveyor belt, picking and placing them in bins, placing empty trays on Automated Guided Vehicles (AGVs), and arranging parts on trays based on kitting requirements. AGVs transport them to the assembly destination.

- **Assembly**: The ceiling robot picks up parts from trays and inserts them into respective slots.

- **Combined**: Combines kitting and assembly tasks, with the floor robot performing kitting and the ceiling robot handling assembly.

## Order Parsing and Processing

Orders are received and stored in vectors based on priority. The `RoboCircus` node processes these orders, ensuring part sufficiency and optimizing task completion efficiency.

## Sensors and Cameras

Sensors and cameras play a vital role in detecting part type, color, and location. Various cameras, including RGB and Basic Logical Cameras, provide information about parts, while a perception pipeline involving HSV masking determines part color and type.

## Difficulties Encountered

During the project, we encountered challenges with order handling, dynamic pick-and-place tasks, and quality checker service limitations. We addressed these challenges by fine-tuning algorithms, implementing resource management techniques, and ensuring code robustness. Our final demo showcased effective problem-solving and coordination, resulting in a successful demonstration and a full score.

## Dependencies

Our project relies on the following dependencies:

1. ROS2-galactic (including Gazebo and RViz)
2. Moveit 2

## Building and Running the Package

To build and run the package, follow these commands in order:

1. `colcon build --packages-select competitor_interfaces`
2. `colcon build --packages-select group6`
3. `colcon build --packages-select rqt_joint_trajectory_controller`

To run the package:

1. `ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa4 competitor_pkg:=group6 sensor_config:=group6_sensors`
2. `ros2 launch ariac_moveit_config ariac_robots_moveit.launch.py`
3. `ros2 launch group6 group6.launch.py`

## Additional Information

For detailed rules, resources, and updates related to ARIAC, please visit the official ARIAC website: [ARIAC Official Website](https://www.nist.gov/el/intelligent-systems-division-73500/advanced-robotics-industrial-automation-challenge-ariac).

ARIAC offers a fantastic opportunity for researchers and robotics enthusiasts to showcase their skills, advance industrial automation,
