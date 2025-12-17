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

## 🚀 Installazione e Setup

Assicurati di trovarti nella root del tuo workspace ROS 2:

1.  **Clone the Repository: Navigate to your workspace source directory and clone the repository.**
    ```shell
    cd /ros2_ws
    git clone https:
    '''

2.  **Launch the world**
   ```shell
    ros2 launch airport_baggage_system iiwa_launch.py
    '''
    
3.  **In another terminal run the color detection node**
    ```shell
     ros2 run airport_baggage_system color_detector.py
    ```
    
4.  **In another terminal launch the MoveArm controller**
   ```shell
     ros2 launch ros2_kdl_package kdl_action.launch.py
    ```

5.  **In another terminal launch the Nav2 controller for Fra2mo**
   ```shell
     ros2 launch ros2_fra2mo fra2mo_navigation.launch.py
    ```

6.  **In another terminal run Fra2mo manager node**
   ```shell
     ros2 launch ros2_fra2mo fra2mo_manager_cpp
    ```

7.  **In another terminal run IIWA manager node**
   ```shell
     ros2 launch ros2_fra2mo iiwa_manager_cpp
    ```


