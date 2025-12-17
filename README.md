# ✈️ Airport Multi-Robot Baggage Handling System

**Robotics Lab Project** *Un sistema collaborativo autonomo per lo smistamento bagagli simulato in ROS 2 e Gazebo.

## 📖 Descrizione del Progetto

Questo progetto implementa un sistema multi-robot composto da un manipolatore **KUKA LBR IIWA** e un robot mobile **Fra2Mo**. L'obiettivo è simulare un processo automatizzato di smistamento bagagli in un aeroporto.

Il sistema opera secondo la seguente logica collaborativa:
1.  **Riconoscimento:** Il KUKA IIWA utilizza una telecamera per identificare i pacchi sul tavolo tramite visione artificiale (OpenCV).
    * 🔴 **Valigia Rosso:** Destinato all'Area Rossa.
    * 🟢 **Valigia Verde:** Destinato all'Area Verde.
2.  **Manipolazione & Handover:** Il braccio robotico preleva l'oggetto e lo posiziona sul robot mobile.
3.  **Navigazione Autonoma:** Fra2Mo riceve l'ordine, naviga verso l'area di scarico specifica utilizzando **Nav2**.
4.  **Visual Docking:** Dopo lo scarico, Fra2Mo ritorna alla stazione base effettuando una manovra di docking precisa basata su **Visual Servoing** tramite ArUco Tag.

## 🛠️ Architettura del Sistema

Il progetto si basa su un'architettura a nodi distribuiti:

* **`iiwa_manager`**: Nodo C++ che gestisce la logica del manipolatore e invia i comandi al robot mobile.
* **`fra2mo_manager`**: Nodo C++ che implementa la Macchina a Stati Finiti (FSM) del robot mobile.
* **`color_detector`**: Nodo Python che elabora le immagini per identificare il colore dei cubi.
* **`kdl_action_server`**: Server per la cinematica inversa del braccio robotico.
* **`nav2_stack`**: Gestisce la localizzazione e la pianificazione del percorso.

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
 
## **5. IIWA Task Manager Execution**
This activates the robotic arm supervisor, which coordinates the object-picking phases and triggers the transportation missions.
```shell
ros2 run airport_baggage_system iiwa_manager_cpp
```
