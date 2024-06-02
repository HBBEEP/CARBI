# CARBI _mecanum mobile robot_

## 📖 Description 

CARBI is a mecanum mobile robot equipped with an RPLidar mounted on top. In its current phase, CARBI can generate 2D maps using the RPLidar in conjunction with the slam_toolbox and navigate to specified points (go-to-goal). However, a persistent issue is the inaccurate yaw orientation of CARBI due to slippage of the mecanum wheels during turning, affecting the calculation of wheel odometry. Additional problems will be discussed in this section [🔴 Problem](#-problem) 


## 👍 Members
This project is a part of the FRA532 Mobile robotics course. Our team includes:

1. Kullakant Kaewkallaya 64340500006 [@hbbeep](https://github.com/hbbeep)
2. Thamakorn Tongyod 64340500028 [@TheGot](https://github.com/TheGotGithub)

## CARBI Controller

The CARBI Controller manages the operation of motors to enable the robot to move in specified directions and at set speeds. It is divided into two main parts the ROS Node and the ESP32 Controller, which communicate via Serial communication.

### **ROS Node**

The ROS Node converts commands and sends data through Serial to the ESP32 microcontroller, as well as reading the speed of each wheel.

![image](https://github.com/HBBEEP/CARBI/assets/104858772/2862cf5f-c482-4875-ac7e-277ab000ea0b)

**Node subscriber:**

- **`/cmd_vel`**: Sets the speed of each wheel.
- **`/Emergency`**: Controls the emergency stop of the robot.

**Node publisher:**

- **`/wheel_vel`**: Publishes the speed of each wheel.

**Serial communication:**

- **Motor command:** Instructs the ESP32 to control each motor to operate at the specified speed.
    
    Serial format:
    
    ```xml
    <v,velocity_motorA,velocity_motorB,velocity_motorC,velocity_motorD>
    ```
    
- **Sensor callback:** Data read from sensors sent by the ESP32.
    
    Serial format
    
    ```xml
    sensor1,sensor2,sensor3,sensor4
    ```
    

**Functions:**

- **`set_velocity_cmd`**: Converts the **`cmd_vel`** values (Twist) into the specified format to send to the ESP32.
- **`serial_wheel_cmd`**: Sends data through Serial to the ESP32.
- **`Emergency_control`**: Manages the emergency state by instructing the ESP32 to stop when this function is triggered.
- **`Serial_read`**: Receives data sent from the ESP32 via Serial and publishes the values.

### ESP32 Controller

The ESP32 Controller operates on the microcontroller, sending and receiving commands and data through Serial. It controls motor speed and reads raw data from the Encoder to convert into speed.

![image](https://github.com/HBBEEP/CARBI/assets/104858772/d810cf52-3764-434a-9f14-652b31f4e931)

**Functions:**

- **`serial_read`**: Reads Serial data sent from the ROS Node.
- **`parse_data`**: Parses the received message and categorizes the data.
- **`state_control`**: Processes the parsed data according to the specified mode and sets the target values.
- **`PID`**: Controls the motor speed to maintain the specified target speed.
- **`motor_drive`**: Supplies power to the motor in the form of PWM.
- **`motor_velocity_callback`**: Reads values from the encoder and converts them into speed in rad.
- **`serial_write`**: Formats and sends the motor speed data through Serial.

### **Node Run**

```bash
ros2 run carbi_controller controller.py
```

## CARBI Electronic and Electrical component

### Control part

- Single board computer (Master) : Raspberry Pi5+Active Cooler
- Microcontroller (Slave) : ESP32 36Pin+shield
- Step down : DC to DC 24V to 5V 5A
- Motor Drive : MDD10A Cytron
- Motor : Motor 24V 50 rpm
- Emergency Switch
- Power Switch

### Sensor part

- IMU : mpu9250
- Lidar : rplidar A1M8-R6
- Encoder : AMT103-V

--------
## Wheel odometry of mecanum 

Forward Kinematics 

![Screenshot 2024-06-03 015238](https://github.com/HBBEEP/CARBI/assets/75566343/d3cfd636-c434-46fd-9713-8f07a521a21d)

Inverse Kinematics

![Screenshot 2024-06-03 015245](https://github.com/HBBEEP/CARBI/assets/75566343/e90d1a5a-819c-4c48-bf4e-4e4f05daf23e)

When :

![Screenshot 2024-06-03 015149](https://github.com/HBBEEP/CARBI/assets/75566343/e4ed9fb9-9bad-4c96-84a1-b0e09e07784f)


reference : https://cdn.intechopen.com/pdfs/465/InTechOmnidirectional_mobile_robot_design_and_implementation.pdf

--------

## ⚙️ Installation 

### 0. Clone this repo to your folder and CARBI will be your workspace (in both Raspberry Pi 5 & your computer)
```
git clone https://github.com/HBBEEP/CARBI.git
```

### 1. Config docker-compose.yml file (In both Raspberry Pi 5 & your computer)
![docker_yml](https://github.com/HBBEEP/CARBI/assets/75566343/a597f1df-866e-487f-a902-41e23b126b56)

- For Raspberry Pi 5, there is no need to edit this section. However, if you change the workspace name or move it to another folder, you should update it accordingly.
- Edit name in volumes section
 ```
  - /home/[path to CARBI workspace]/ros:/home/ros 
  - /home/[path to CARBI workspace]/CARBI:/ros2
 ```
- Example:
  ```
  - /home/hbbeep/CARBI:/ros2 (hbbeep is my computer name)
  - /home/hbbeep/CARBI:/ros2
  ```

(but, in this repo, you can see that I named my workspace CARBI-1.)

### 2. Set Up CARBI environment with Docker (your computer)

```
cd ~/CARBI-1/docker
```

```
docker compose build # If docker environment is not built
```

```
docker compose up -d
```
```
docker exec -it docker-ros2-1 bash
```
then RUN this command in root
```
cd ros2/carbi
```

- Then your terminal should look like this

![computer_terminal](https://github.com/HBBEEP/CARBI/assets/75566343/8ff10b2c-9715-49b8-8c0e-77bf75dd91cf)

**Note:**  If you are not familiar with Docker, please follow this tutorial (which I followed too :) )

- https://www.kevsrobots.com/learn/learn_ros/04_docker_install.html 


### 3. remote to Raspberry Pi 5 by using SSH and then repeat step 3
```
ssh carbi@[IPv4 of raspberry pi 5]  # example -> ssh carbi@99.9.999.99
```

- Then your terminal should look like this
  
![rpi5_terminal](https://github.com/HBBEEP/CARBI/assets/75566343/83c6a396-9aed-42ff-a758-136bd0050b4b)

**Note:** 
1. Don't forget to connect internet in Raspberry Pi 5
2. You can check the IPv4 address of Raspberry Pi 5 by using some tools like advance IP scanner program.

### 4. Clone this rplidar repo to your workspace  (remote to Raspberry Pi 5) 
```
git clone https://github.com/babakhani/rplidar_ros2
```

**Note:** before running Rviz2, run this command in your terminal to allow root access to the X server (your computer)
```
xhost local:root
```

--------


## 📡 Teleop


https://github.com/HBBEEP/CARBI/assets/75566343/d1c1fac2-81c5-4e31-a960-9e404e430300


### Terminal 1 (remote to Raspberry Pi 5)

```
ros2 launch carbi_bridge carbi.launch.py
```

### Terminal 2 (remote to Raspberry Pi 5)

```
ros2 run carbi_control carbi_controller.py
```

### Terminal 3 (your computer or remote to Raspberry Pi 5 )
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


## Mapping

https://github.com/HBBEEP/CARBI/assets/75566343/b04095f6-ef08-48d1-8287-39a9b05bdae1

### Terminal 1 (remote to Raspberry Pi 5)

```
ros2 launch carbi_bridge carbi.launch.py
```

### Terminal 2 (remote to Raspberry Pi 5)
```
ros2 run carbi_control carbi_controller.py
```

### Terminal 3 (your computer)
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Terminal 4 (your computer)
```
ros2 launch carbi_navigation mapping.launch.py
```

**Note** : if map doesn't show -> change **Durability Policy** topic to _**Trasient Local**_

![fix_map](https://github.com/HBBEEP/CARBI/assets/75566343/9f5976b7-7fd2-439b-8be9-d3da4fcde1de)


## 💡 Navigation (go to goal)


https://github.com/HBBEEP/CARBI/assets/75566343/6a9d6a66-413e-4adc-943a-990ac49c4f6c



### Terminal 1 (remote to Raspberry Pi 5)
```
ros2 launch carbi_bridge carbi.launch.py
```

### Terminal 2 (remote to Raspberry Pi 5)
```
ros2 run carbi_control carbi_controller.py
```

### Terminal 3 (your computer) 
```
ros2 launch carbi_navigation navigation.launch.py
```

### Terminal 4 (your computer) 
```
ros2 launch carbi_navigation justdisplay.launch.py 
```

--------

## Demo 

### Teleop
https://github.com/HBBEEP/CARBI/assets/75566343/6e0ab325-5b5f-4c99-aeb5-c4cbf7db826a

### Go to Goal
https://github.com/HBBEEP/CARBI/assets/75566343/d1e5f386-5c25-4314-a8aa-83ececcffe45

#### global cost map
  
![global_cost_map](https://github.com/HBBEEP/CARBI/assets/75566343/f164cef7-0c90-47cc-85fa-914b470180db)

#### local cost map
  
![local_cost_map](https://github.com/HBBEEP/CARBI/assets/75566343/59c2f7a7-7f45-43d1-9841-0280b5625365)

--------

## 🔴 Problem

1. Raspberry Pi reads values from USB Serial unreliably when reading values from RPLIDAR simultaneously.
2. Mecanum wheels slip on smooth and slippery surfaces.
3. 
