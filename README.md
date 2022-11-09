# Build ROS2 Workspace for ModProFT

## Build and run Docker Container

    ```
    cd ~/ros2-foxy-modprof
    ./BUILD-DOCKER-CONTAINER.sh
    ./RUN-DOCKER-CONTAINER.sh
    ```

## Build UR Driver

    ```
    cd /root/ur_driver
    rosdep install --ignore-src --from-paths src -y -r
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/setup.bash
    ```

## Build ROS2 Workspace:

    ```
    cd /root/ModProFT
    rosdep install --ignore-src --from-paths src -y -r
    colcon build
    source install/setup.bash
    ```

# Start Driver

## Planning in RViz / Simulation in Gazebo

   Run in Terminal:

    ```
    ros2 launch modproft_ur_bringup sim_bringup.launch.py
    ```

    ```
    ros2 launch modproft_ur_bringup sim_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=false
    ros2 launch modproft_ur_bringup ur_moveit.launch.py ur_type:=ur5e robot_ip:=xxx.xxx use_fake_hardware:=true launch_rviz:=true
    python3 /root/ModProFT/src/robotcontrol/scripts/moveit_action_client.py
    ```

## Planning in RViz / Real Robot

   When Host-IP isn't "192.168.1.101" replace it in ros_control.urscript in /overwrite/ur_robot_driver/resources/ in line 102:

   ```
   socket_open("<your_host_ip>", {{SERVER_PORT_REPLACE}}, "reverse_socket")
   ```

   After that rebuild your ur_driver workspace.

   When your UR Type isn't UR5e and your your Robot IP isn't 192.168.1.20 adjust ur_type and robot_ip in ros2_ws/src/modproft_ur_bringup/launch/bringup.launch.py and rebuild your ROS2 workspace. Then start the driver with the following command:

   ```
   ros2 launch modproft_ur_bringup bringup.launch.py
   ```

# Driver Changes

https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/foxy

## UR Description

   When building the Container the description package of the ur driver is replaced with a newer version for Gazebo support

   https://github.com/UniversalRobots/Universal_Robots_ROS2_Description

## Hardware Interface

   For support of the new description changes have been made in the hardware interface of the ur_driver. The two files hardware_interface.cpp and hardware_interface.hpp had been adjusted with supplements of the same files of the same drivers galactic branch.

   https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/galactic/ur_robot_driver