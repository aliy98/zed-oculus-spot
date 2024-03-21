## Immersive control of a quadruped robot with Virtual Reality Eye-wear
This work describes an immersive control system for a quadruped robot, designed to track the head movements of
the operator wearing a virtual reality eye-wear, while also utilizing joystick commands for locomotion control. The
article details the implemented closed-loop velocity control approach, and the locomotion task specifications.
The proposed method has been implemented on Spot robot from Boston Dynamics, with Meta Quest 2 virtual reality system.
Evaluation of the approach involved a user study, where participants engaged in immersive control of the quadruped
robot within an indoor experimental environment and provided feedback through standardized questionnaires.
Pairwise comparison of the resulting data revealed significant advantages for the proposed immersive control
system over a standard remote controller, with enhanced performance observed in the second trial of using the
control system. However, participants lacking experience with virtual reality systems reported increased distress
symptoms following the experiment. 

**Authors:** 
  - Ali Yousefi, ali.yousefi@edu.unige.it
  - Zoe Betta, zoe.betta@edu.unige.it
  - Giovanni Mottola, giovanni.mottola@unige.it
  - Carmine Tommaso Recchiuto, carmine.recchiuto@dibris.unige.it
  - Antonio Sgorbissa, antonio.sgorbissa@unige.it
    
©2024 RICE Lab - DIBRIS, University of Genova
<p align="left">
<img src="https://github.com/aliy98/zed-oculus-spot/assets/65722399/6e9d73eb-292f-40a0-b181-d8918b1fa0ad" width="150" title="rice_logo">
</p>


### Package Description
This repository provides a modified version of the software available on [zed-oculus](https://github.com/stereolabs/zed-oculus). Since the IMU and touch input data is required for this work, the ``main.cpp`` is modified in such a way that it reads the angular velocities, and touch input data using ``ts.HeadPose.AngularVelocity``, ``InputState.Thumbstick[ovrHand_Right]``, and ``InputState.Thumbstick[ovrHand_Left]``  class attributes, and sends them to the process executed by ``main.py``. Additionally, considering the fact that the ZED camera is not connected to the user PC with a USB cable, another modification is done in ``main.cpp`` file, in order to open the ZED camera from the socket input, by changing the ``init_paramters`` values in ``zed.open(init_parameters)`` same as the method [HERE](https://github.com/stereolabs/zed-sdk/tree/master/camera%20streaming/receiver/cpp).

Furthermore, the ``scripts`` folder is added to this package, which contains the sofware developed for the headtracking task, and locomotion with joysticks. The script files are described as follows:

| Script | Description |
| ------ | ----------- |
| main.py | Gets executed by the ``main.cpp`` file. Uses the ``SpotInterface`` and ``Controller`` classes methods for the headtracking and locomotion tasks. |
| controller.py | Provides a simple closed loop controller using the [simple-pid](https://pypi.org/project/simple-pid/) python module with the method ``get_hmd_controls(setpoints)``. Additionally, computes the locomotion control signal based on the touch input reference signals with the method ``get_touch_controls(setpoints)``. |
| spot_interface.py | Initializes the Lease, eStop, Power, RobotState, and RobotCommand clients. Provides the required method for sending the control signals to the robot ``set_controls(controls, dt)``, and receving robot angular velocities ``get_body_vel()`` |


### System Architecture
The system architecture for this work is shown as it follows:

<p align="center">
<img src="https://github.com/aliy98/zed-oculus-spot/assets/65722399/ccb0fc22-0ab7-46c1-be9e-0b52e6abde9a" width="700" title="network">
</p>

A Wi-Fi bridge could be implemented on the Raspberry Pi board using this [tutorial](https://pimylifeup.com/raspberry-pi-wifi-bridge/). Once it is ready, the components of the local network could be configured with the following ip addresses:


|        Component       |              IP Address            |
| ---------------------- | ---------------------------------- |
| Raspberry Pi 4 Model B |        192.168.220.1 (Server - Ethernet and Wireless)      |
|     Jetson Nano        | 192.168.220.50 (Client - Ethernet) |
|        OMEN PC         |   10.42.0.210 (Client - Wireless)  |
|       Spot Robot       |   10.42.0.211 (Client - Wireless)  |


### Interprocess Communication
For the purpose of this work, the IMU data generated by the HMD, is transmitted to the ``main.py`` control process, using a **named pipe**. Moreover, the stereo camera images is transmitted from the robot to the HMD, using a **socket**, with the method of this [tutorial](https://github.com/stereolabs/zed-sdk/tree/master/camera%20streaming). Sending the control signals and receiving the robot state data is done using **gRPC**.

### Dependencie
- Windows 64 bits
- [Spot SDK](https://dev.bostondynamics.com/)
- [simple-pid](https://pypi.org/project/simple-pid/)
- [ZED SDK 3.x](https://www.stereolabs.com/developers) and its dependencies ([CUDA](https://developer.nvidia.com/cuda-downloads)). Last tested with ZED SDK 3.2.2
- [Oculus SDK](https://developer.oculus.com/downloads/package/oculus-sdk-for-windows/) (1.17 or later)
- [GLEW](https://glew.sourceforge.net/) included in the ZED SDK dependencies folder
- [SDL](https://github.com/libsdl-org/SDL/releases/tag/release-2.30.1)

### Build
Download the sample and follow the instructions below: More
1. Create a folder called "build" in the source folder
2. Open cmake-gui and select the source and build folders
3. Generate the Visual Studio Win64 solution
4. Open the resulting solution and change configuration to Release. You may have to modify the path of the dependencies to match your configuration
5. Build solution

### Usage
1. On the robot side (Linux/Jetson), build and run the streaming sender using the method shown [HERE](https://github.com/stereolabs/zed-sdk/tree/master/camera%20streaming/sender/cpp).
2. On the user side (Windows), run the ''ZED_Stereo_Passthrough.exe'' in a terminal as it follows:
```
    ./ZED_Streaming_Receiver <ip:port>
```
Once it is executed, the stereo passthourgh from ZED camera to Oculus starts. Moreover, it will automatically run the ``main.py`` script, that provides the control system and iterface with the robot.

### System hypothesis and future work
For future work, we aim to address the limitations of the current study. This includes implementing a more efficient communication system to control the robot from greater distances, and ensuring a comparable field of view between the HMD and the tablet-based controller.
