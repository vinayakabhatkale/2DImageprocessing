# ModProFT

## Setup the environment
### Download Docker Container

Use Git to clone the ModProFT docker container included in this repository.

```
git clone -b master https://gitlab.elektrotechnik.hs-augsburg.de/ttz/modproft.git
```

Navigate into the cloned repository and use the convenience script provided to build the image of the docker container.

```
cd modproft/
```
```
./BUILD-DOCKER-CONTAINER.sh
```
Run the container via another convenience script in the current terminal.
```
./RUN-DOCKER-CONTAINER.sh
```

### Build ROS workspace
Within the container navigate to the ModProFT ROS workspace.
```
cd ~/ROS_Foxy_ModProFT/
```
Build the workspace via colcon.
```
colcon build
```
Source the workspace for ROS.
```
source install/setup.bash
```

## Using the code
### How to run the code as simulation
After sourcing the workspace the simulation can be started.
```
ros2 launch modproft_ur_bringup sim_bringup.launch.py 
```

### How to run the code on a real robot
#### Networking setup
##### Connecting the docker host device

Check the host IP outside the docker container.
```
ifconfig
```
If the networking device connecting the docker host with the robot does **not** have the IP *192.168.1.101* the IP has to be changed.
It can be changed via `ifconfig` or within `ros_control.urscript` located in `/ur_driver_override/resources/` outside the docker container. There, in line 102 the information has to be edited.
```
socket_open("<your_host_ip>", {{SERVER_PORT_REPLACE}}, "reverse_socket")
```
***Attention: The rebuiling of the docker container is required for these changes to take effect!***

##### Connecting the robot

Check the IP assigned to the robot via the robot control panel.
Set the robots IP to *192.168.1.20* or edit the IP within the docker container. The file required to be edited is `real_ur_bringup.launch.py` located in `ros2_ws/src/modproft_ur_bringup/launch/`. In line 22 and 29 the robots IP has to be set.

This project was developed with Universal Robots UR5e robot in mind, other Universal Robot robots from the UR series are available, but not supported. To change the series set the chosen UR type in line 22 and 29 within `real_ur_bringup.launch.py` located in `ros2_ws/src/modproft_ur_bringup/launch/`.

***Attention: For either of the changes to take effect a rebuild the docker container is required!***

#### Running the code
After sourcing the workspace the real robot can be controlled.
```
ros2 launch modproft_ur_bringup real_ur_bringup.launch.py
```

## Developing for the code
### Development Standards
- Do not override the robot driver. Use native driver functions and create wrappers to achive the desired behavior.
- Do create new packages within the `ROS_Foxy_ModProFT` workspace where appropriate to keep the code structured.
- Do chose self-descriptive names for packages, nodes, functions, variables, etc.
- Do comment appropriately, such as function in- and outputs, function descriptions, complex code statements, etc.
- For the sake of standardisation it is highly recommended to use Visual Studio Code with extensions as IDE for the development.

### Used sources
#### Robot integration
This project uses the [ROS2 drivers](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/foxy) (ROS2 Foxy branch) provided by Universal Robots for the UR series. This driver is modified when the docker container is build. This is done for the native Gazebo integration, which is provided by the Universal Robots robot description for ROS2 (ROS2 Galactic branch). Futhermore, the hardware interface of the driver is adapted using the files within `ur_driver_override`.


### Common fallacies
***The meshes of models are not shown in Gazebo and Gazebo needs a long time to start.***

- Problem: Gazebo does not find the meshes of the models.
- Debugging: Look for information in Gazebo logs located in `~/.gazebo/<server-11345 or client-11345>/`.
- Solution: Add a soft link to connect your meshes with `~/.gazebo/models/`.
- Explanation: Even when using the `package://` syntax within XACRO files, Gazebo starts to search for models in its models directory (default). This is a bug not a feature.

***Instable TF2 transformation tree.***

- Problem: Multiple publishers (`static_transform_publisher`, `robot_state_publisher`) for the same transformation.
- Debugging: Difficult, using `tf_tools` and common sense.
- Solution: Identify and eliminate the superfluous publisher.
- Explanation: Every link contained in the robots URDF is published via `robot_state_publisher`. Using `static_transform_publisher` for links contained into the robots URDF lead to inconsistencies in the tf tree.

***Message filter dropping message.***

- Problem: The message is invalid.
- Debugging: Difficult, using `tf_tools` and look into `topics`.
- Solution: Fix timestamp, for the simulation look if `{"use_sim_time":True}` parameter is set in node context, fix node publish timing, fix tf transformation to sensor.
- Explanation: A broad variety of problems could be the the source of this error. However it is (always) related to the tf tree and the time sync of the nodes in question.

### Integration of components mounted on the robot
In order to integrate further components such as grippers or cameras some standards must be complied with.
- Each component has to be defined as XACRO.
- Each component has to be parameterisable via .yaml.
- Each component gets its own package, containing meshes, configurations and XACROs.

In order to load the components append their macro to `ur.urdf.xacro` located in `~/ROS_Foxy_ModProFT/src/modproft_ur_description/urdf/`.
