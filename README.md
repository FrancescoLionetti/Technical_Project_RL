# ✈️ Airport Multi-Robot Baggage Handling System

**Robotics Lab Project** *A collaborative autonomous system for baggage sorting simulated in ROS 2 and Gazebo.*

## 📖 Project Description

This project implements a multi-robot system consisting of a **KUKA LBR IIWA** manipulator and a **Fra2Mo** mobile robot. The goal is to simulate an automated baggage sorting process in an airport environment.

The system operates according to the following collaborative logic:
1.  **Recognition:** The KUKA IIWA uses a camera to identify packages on the table via computer vision (OpenCV).
    * 🔴 **Red Suitcase:** Destined for the Red Area.
    * 🟢 **Green Suitcase:** Destined for the Green Area.
2.  **Manipulation & Handover:** The robotic arm picks up the object and places it onto the mobile robot.
3.  **Autonomous Navigation:** Fra2Mo receives the order and navigates towards the specific drop-off area using **Nav2**.
4.  **Visual Docking:** After unloading, Fra2Mo returns to the base station, performing a precise docking maneuver based on **Visual Servoing** using ArUco Tags.

## 🛠️ System Architecture

The project is based on a distributed node architecture:

* **`iiwa_manager`**: C++ node that manages the manipulator logic and sends commands to the mobile robot.
* **`fra2mo_manager`**: C++ node that implements the mobile robot's Finite State Machine (FSM).
* **`color_detector`**: Python node that processes images to identify the color of the cubes.
* **`kdl_action_server`**: Server for the robotic arm's inverse kinematics.
* **`nav2_stack`**: Manages localization and path planning.

## ⚙️ Getting Started
To successfully set up and test the project, follow these steps within your ROS 2 workspace. 
1.  **Clone the Repository**
    ```shell
    cd /ros2_ws
    git clone https://github.com/FrancescoLionetti/Technical_Project_RL.git
    ```
 
2.  **Build the Workspace: Return to the workspace root, build the packages, and source the environment.**
    ```shell
     colcon build
     source install/setup.bash
    ```

## 🏃 Execution Instructions
To properly initiate the collaborative mission between the two robots, the following commands must be executed in separate terminals.
## **1. Environment and Gazebo Simulation Setup**
This command initializes the warehouse environment, loads the robot models (IIWA and Fra2Mo), and starts aruco nodes.
```shell
ros2 launch airport_baggage_system iiwa_launch.py
```
## **2. Color Detector Node**
This launches the color detector.
```shell
ros2 run airport_baggage_system color_detector.py
```

## **3. KDL Action Server Configuration (IIWA)**
This launches the action server based on the KDL library to manage the kinematics and motion control for the IIWA manipulator arm.
```shell
ros2 launch ros2_kdl_package kdl_action.launch.py
```
## **4. Autonomous Navigation Activation (Fra2Mo)**
This initializes the Nav2 stack, loading the warehouse map and configuring the localization systems required for the mobile robot's movement.
```shell
ros2 launch ros2_fra2mo fra2mo_navigation.launch.py
```
 
## **5. Fra2Mo Task Manager Execution**
This starts the node responsible for delivery logic, visual docking via Aruco markers, and synchronization with the manipulator arm.
```shell
ros2 run airport_baggage_system fra2mo_manager_cpp
```
 
## **6. IIWA Task Manager Execution**
This activates the robotic arm supervisor, which coordinates the object-picking phases and triggers the transportation missions.
```shell
ros2 run airport_baggage_system iiwa_manager_cpp
```
