# ARIAC - Agile Robotics for Industrial Automation Challenge 2023
Performed ARIAC by NIST (National Institute of Standards and Technology) in ENPM-663 (Building a Manufacturing Robotics Software System) course.
The repository was made to execute multiple challenges part of ARIAC competition - [ARIAC Official Documentation](https://ariac.readthedocs.io/en/latest/)

https://github.com/Rishikesh-Jadhav/ARIAC-AgileRoboticsforIndustrialAutomationCompetition2023/blob/main/images/Screenshot%202023-03-21%20224033.png
#### ARIAC WORKFLOOR
![ARIAC WORKFLOOR](images/Screenshot%202023-03-21%20224033.png)

# Table of Contents
1. [ARIAC - Agile Robotics for Industrial Automation Challenge 2023](#ariac---agile-robotics-for-industrial-automation-challenge-2023)
2. [Competition Overview](#competition-overview)
3. [Key Components](#key-components)
4. [Our Team's Work](#our-teams-work)
5. [Agility Challenges](#agility-challenges)
      - Faulty Parts Challenge
      - High-priority Orders Challenge
      - Insufficient Parts Challenge
      - Flipped Parts Challenge
      - Dynamic Pick and Place from Conveyor Belt
6. [Architecture](#architecture)
7. [Order Parsing and Processing](#order-parsing-and-processing)
8. [Sensors and Cameras](#sensors-and-cameras)
9. [Difficulties Encountered](#difficulties-encountered)
10. [Dependencies](#dependencies)
11. [Running the Package](#running-the-package)
12. [Additional Information](#additional-information)

ARIAC is an annual robotics competition that focuses on advancing industrial automation by testing and pushing the capabilities of robotic systems in simulated manufacturing environments. Below, you will find detailed information about ARIAC, its objectives, how it operates, and our team's participation.

## 🤖 Competition Overview

**Objective**: ARIAC aims to promote the development of robotic systems capable of performing complex tasks in industrial settings autonomously. Participants are challenged to create and deploy robotic solutions that can efficiently and accurately complete various tasks in a simulated factory environment.

**Simulated Environment**: The competition takes place in a simulated factory using the ROS2 and the Gazebo simulator. Participants interact with the factory environment, including robotic arms, sensors, and conveyor belts, through ROS-based control interfaces.

**Challenges**: ARIAC presents participants with a series of challenges that typically include tasks such as material handling, assembly, quality inspection, and process optimization. Each challenge is designed to test different aspects of robot capabilities, including perception, planning, and control.

**Scoring**: Participants were evaluated based on their ability to complete tasks accurately and efficiently. Scoring was determined by factors such as task completion time, accuracy, and resource utilization. The team with the highest score wins.

## Key Components

ARIAC involved several key components, each contributing to the overall challenge:

1. **Simulation Environment**: The factory environment is simulated using Gazebo, providing a realistic yet safe platform for testing and development. We wrote a competition control software to operate within this simulation.

2. **Tasks and Challenges**: ARIAC includes a variety of tasks and challenges that require robots to perform actions like picking and placing objects, assembling components, inspecting products, and optimizing manufacturing processes.

3. **ROS Control Interfaces**: We used ROS2 to interface with and control the robots and sensors. ROS2 provides a standardized framework for robotic software development.

4. **Scoring System**: A scoring system calculates the performance of each team based on task completion, efficiency, and accuracy. This system determines the winners of the competition.

## Our Team's Work

Our team, consisting of members Rishikesh Jadhav, Vishal Sivakumar, Abhinav Garg, Nishant Pandey and Saketh Banagiri, participated in the ARIAC competition during 2023 in our ENPM663 class. Here's an overview of our work:

- **Team Name**: Group6
- **Team Members**:
  - Rishikesh Jadhav
  - Vishal Sivakumar
  - Abhinav Garg
  - Nishant Pandey
  - Saketh Banagiri

### Here are some highlights of our work:

#### Task 1: Material Handling

In this challenge, our robotic system demonstrated exceptional material handling capabilities. We successfully picked and placed objects from conveyor belts **dynamically** and bins to complete kitting tasks with high precision and efficiency, showcasing our advanced control algorithms.

#### Task 2: Assembly

Our package excelled in the assembly task, where the gantry robot required to assemble complex components. Through meticulous planning and real-time feedback using sensors, we achieved perfect assembly accuracy in the competition.

#### Task 3: Quality Inspection

Quality inspection is crucial in manufacturing, and our robotic vision system shone in this area. Our package was agile enough to call the services for performing QI to get rid of defective parts and replace them before completing the task. 

## Agility Challenges

The ARIAC competition presented several Agility challenges, including:

1. **Faulty Parts Challenge**: The Faulty Parts Challenge revolves around detecting and managing parts that are deemed unsuitable for use in the competition. The primary objective is to leverage the quality checker service to identify faulty parts and subsequently replace them with new ones.
Within our system, the handling of faulty parts is accomplished by initiating quality checks before any parts are placed on Automated Guided Vehicles (AGVs). If a part is identified as faulty during this process, it is promptly substituted with either an exact matching part or an appropriate alternative. This meticulous approach guarantees that faulty parts do not compromise the overall quality of the assembled kits.

2. **High-priority Orders Challenge**: The High-priority Orders Challenge requires participants to fulfill a high-priority order ahead of a low-priority order. In this scenario, the Central Control System (CCS) plays a critical role in identifying high-priority orders and swiftly transitioning between tasks.
To effectively manage high-priority orders within our package, we maintain two distinct vectors: one for high-priority orders and another for low-priority orders. When a high-priority order is introduced, it is given immediate precedence, and our system is configured to prioritize its execution before addressing low-priority tasks. This prioritization ensures the timely completion of high-priority orders, aligning with the challenge's requirements.

3. **Insufficient Parts Challenge**: The Insufficient Parts Challenge replicates scenarios in which the work cell confronts a shortage of components required to fulfill one or more orders. In this challenge, the Central Control System (CCS) faces the task of recognizing insufficient parts and responding appropriately, which may involve employing alternative parts or submitting orders in an incomplete state.
Our approach to addressing the Insufficient Parts Challenge revolves around the use of an unordered map to meticulously monitor part quantities and their specific locations within the work cell. When the CCS detects a shortage of required parts, the system proactively identifies available alternatives or parts of different colors that can be utilized to ensure order completion without unnecessary delays. This strategic handling of resource shortages optimizes the efficiency of our system when facing the challenge of insufficient parts.

4. **Flipped Parts Challenge**: Flipped parts were the parts upside down. Our CCS had the capability of flipping these parts to the correct orientation. We used both the gantry and floor robot for this task.

5. **Dynamic Pick and Place from Conveyor Belt**: In the context of dynamic pick and place operations from a continuously moving conveyor belt, our system incorporates a sophisticated approach that takes into account the relative velocity between the floor robot and the parts on the conveyor. This method plays a pivotal role in ensuring the precise calculation of pickup zones, thereby facilitating the seamless and efficient retrieval of parts directly from the conveyor. This dynamic approach enhances the accuracy and reliability of our pick and place processes when dealing with items in motion on the conveyor belt.

## Architecture

The project's architecture revolves around pick-and-place tasks, specifically kitting and assembly, with the possibility of combined tasks. Here's an overview:

- **Kitting**: The robot identifies parts on a conveyor belt using an Advanced Logical Camera. These parts are picked up by the floor robot and placed in bins (empty slots). Empty trays are picked up using a tray gripper and placed on Automated Guided Vehicles (AGVs). The parts from the bins are placed on the trays based on kitting requirements, and AGVs transport them to the assembly destination, taking agility challenges like high-priority orders and faulty parts into account.

- **Assembly**: In the assembly process, the ceiling robot picks up parts from the tray and inserts them into their respective slots.

- **Combined**: This represents a combination of both kitting and assembly tasks, with the floor robot performing kitting tasks and the ceiling robot executing assembly tasks.

## Order Parsing and Processing

Orders are received and stored in vectors based on their priority. The `RoboCircus` node processes these orders, ensuring the sufficiency of parts and completing tasks as efficiently as possible. When parts are insufficient, a replacement strategy is applied, either using exact replacements or alternative parts.

## Sensors and Cameras

Sensors and cameras play a vital role in detecting part type, color, and location within the environment. Different cameras, including RGB and Basic Logical Cameras, are used to provide information about parts' characteristics and positions. A perception pipeline involving HSV masking is used to determine part color and type.

## Difficulties Encountered

Throughout the project, several challenges and difficulties were encountered, including issues with order handling, dynamic pick-and-place tasks, and quality checker service limitations. However, the team effectively addressed these challenges by fine-tuning algorithms, implementing resource management techniques, and ensuring robustness in the code. The final demo showcased the team's problem-solving skills and efficient coordination, resulting in a successful demonstration and a full score.

In summary, this project demonstrates a comprehensive approach to tackling the ARIAC competition challenges, combining innovative solutions, efficient algorithms, and effective communication between nodes to achieve agile robotic performance in an industrial environment.

Our collective efforts, innovative solutions, and relentless pursuit of excellence allowed us to secure a prominent position in the ARIAC competition during [Year]. We look forward to continuing our journey in the field of industrial automation and robotics.

## Dependencies
1. ROS2-galactic (Along with Gazebo and RViz)
2. Moveit 2

## Running the Package
The Package made is based on [ARIAC Official GitHub Repository](https://github.com/usnistgov/ARIAC/). The services provided in the competition environment are accessed to complete the tasks given.

Method to build the required packages:

- `colcon build --packages-select competitor_interfaces`
- `colcon build --packages-select group6`
- `colcon build --packages-select rqt_joint_trajectory_controller`

Method to run the package:

To run the package use the following commands strictly in the given order:

- `ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa4 competitor_pkg:=group6 sensor_config:=group6_sensors`
- `ros2 launch ariac_moveit_config ariac_robots_moveit.launch.py`
- `ros2 launch group6 group6.launch.py`

## 📄 Additional Information

For more detailed information about ARIAC, including rules, resources, and updates, please visit the official ARIAC website: [ARIAC Official Website](https://www.nist.gov/el/intelligent-systems-division-73500/advanced-robotics-industrial-automation-challenge-ariac).

ARIAC represents an exciting opportunity for researchers and robotics enthusiasts to showcase their skills, advance the field of industrial automation, and explore the potential of robotic systems in real-world manufacturing scenarios.
