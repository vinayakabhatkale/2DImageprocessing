FROM ubuntu:20.04

LABEL maintainer="maximilian.schnitzler@hs-augsburg.de"

#########################
##    System Setup     ##
#########################

# Setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata 

# Setup frontend
RUN DEBIAN_FRONTEND=noninteractive apt-get install lightdm -y

# Setup packages
RUN apt-get update \
	&& apt-get install -y \
       	apt-utils \
		build-essential \
       	cmake \
       	curl \
       	git \
       	gnupg2 \
       	libyaml-cpp-dev \
       	lsb-release \
       	nano \
       	unzip \
       	usbutils \
       	software-properties-common \
       	sudo \
       	tmux \
       	vim \
       	wget \
       	zip \
       && rm -rf /var/lib/apt/lists/*

# Setup encoding
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# Create user
ENV USER=developer

# Create user right, home directory and copy the scripts
# Password for user is the same as the user name itself
RUN useradd -u 1000 ${USER} \
	&& echo "${USER}:${USER}" | chpasswd \
	&& adduser ${USER} sudo
RUN mkdir -p /home/${USER} \
	&& chown -R ${USER}:${USER} /home/${USER}

COPY --chown=${USER} ./docker/default_vim_rc /home/${USER}/.vimrc
COPY --chown=${USER} ./docker/default_bash_rc /home/${USER}/.bashrc
COPY --chown=${USER} ./docker/default_bash_profile /home/${USER}/.bash_profile
COPY --chown=${USER} ./docker/default_tmux_conf /home/${USER}/.tmux.conf

#########################
##    Python Setup     ##
#########################

ARG USE_PYTHON_3_NOT_2=True
ARG _PY_SUFFIX=${USE_PYTHON_3_NOT_2:+3}
ARG PYTHON=python${_PY_SUFFIX}
ARG PIP=pip${_PY_SUFFIX}

RUN apt-get update \
	&& apt-get install -y \
	    ${PYTHON} \
	    ${PYTHON}-pip

RUN ${PIP} --no-cache-dir install --upgrade \
    pip \
    setuptools==58.2.0

RUN ln -s $(which ${PYTHON}) /usr/local/bin/python

#########################
##    ROS Foxy Setup   ##
#########################

# Set ROS environment variables
ENV ROS2_DISTRO foxy
ENV ROS_MASTER_URI http://localhost:11311
ENV ROS_PYTHON_VERSION 3

# Import keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list

# Install ROS
RUN apt-get update && apt-get install -y \
    ros-foxy-ros-core=0.9.2-1* \
    && rm -rf /var/lib/apt/lists/*
    
# Install ROS tools
RUN apt-get update \
	&& apt-get install -y \
		build-essential \
		git \
		liburdfdom-tools \
		${PYTHON}-colcon-common-extensions \
		${PYTHON}-colcon-mixin \
		${PYTHON}-rosdep \
		${PYTHON}-vcstool \
		ros-${ROS2_DISTRO}-gazebo* \
		ros-${ROS2_DISTRO}-gazebo-ros2-control \
		ros-${ROS2_DISTRO}-hls-lfcd-lds-driver \
		ros-${ROS2_DISTRO}-joint-state-publisher \
		ros-${ROS2_DISTRO}-joint-state-publisher-gui \
		ros-${ROS2_DISTRO}-moveit \
		ros-${ROS2_DISTRO}-moveit-common \
		ros-${ROS2_DISTRO}-moveit-servo \
		ros-${ROS2_DISTRO}-moveit-ros-perception \
		ros-${ROS2_DISTRO}-pcl-conversions \
		ros-${ROS2_DISTRO}-rosbag2 \
		ros-${ROS2_DISTRO}-ros2-control \
		ros-${ROS2_DISTRO}-ros2-controllers \
		ros-${ROS2_DISTRO}-rqt \
		ros-${ROS2_DISTRO}-rviz2 \
		ros-${ROS2_DISTRO}-ur-client-library \
		ros-${ROS2_DISTRO}-ur-msgs \
		ros-${ROS2_DISTRO}-warehouse-ros-mongo \
	&& rm -rf /var/lib/apt/lists/*
	
# Setup colcon mixin and metadata
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
	&& colcon mixin update \
	&& colcon metadata add default https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml \
	&& colcon metadata update
	
# Initialize rosdep
RUN rosdep init 

# ________________________________________ !Commands from this point are executed as user! ________________________________________________________________________________
USER ${USER}

RUN rosdep update --rosdistro $ROS2_DISTRO

# Setup ROS for container console
RUN echo "\n#ROS Foxy Setup\nsource /opt/ros/foxy/setup.bash" >> ~/.bashrc

#########################
##      UR Setup       ##
#########################

# UR driver temporary directory
RUN mkdir -p ~/temp/Universal_Robots_ROS2_Driver/src/ \
	&& git clone -b foxy --depth 1 https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git ~/temp/Universal_Robots_ROS2_Driver/src/ \
	&& cd ~/temp/Universal_Robots_ROS2_Driver/src/ \
	&& git fetch --all --tags \
	&& git checkout tags/2.0.1 \
	&& rm -rf ur_description
	
	#&& git checkout -b 01cd1ee47e1913a04557718e23034521caefeaa4 \

RUN mkdir -p ~/temp/Universal_Robots_ROS2_Description/src/ \ 
	&& git clone -b ros2 --depth 1  https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git ~/temp/Universal_Robots_ROS2_Description/src/ \
	&& cd ~/temp/Universal_Robots_ROS2_Description/src/ \
	&& git fetch --all --tags \
	&& git checkout tags/2.0.0
	
	#&& git reset --hard c4d1724ac32a4f0e3bcfbcfa420f82608b07b17f
	#&& git checkout -b c4d1724ac32a4f0e3bcfbcfa420f82608b07b17f

# UR driver directory
RUN mkdir -p ~/Universal_Robots_ROS2_Driver/src/ur_description \
	&& cd ~/Universal_Robots_ROS2_Driver/src/ \
	&& cp -r ~/temp/Universal_Robots_ROS2_Description/src/* ./ur_description \
	&& cp -r ~/temp/Universal_Robots_ROS2_Driver/src/* . \
	&& rm -rf ~/temp/
	
# Override some UR driver packages for hardware setup
COPY --chown=${USER} ./ur_driver_override/include/ur_robot_driver/hardware_interface.hpp ~/Universal_Robots_ROS2_Driver/src/ur_robot_driver/include/ur_robot_driver/
COPY --chown=${USER} ./ur_driver_override/src/hardware_interface.cpp ~/Universal_Robots_ROS2_Driver/src/ur_robot_driver/src/
COPY --chown=${USER} ./ur_driver_override/resources/ros_control.urscript ~/Universal_Robots_ROS2_Driver/src/ur_robot_driver/resources/

# Build UR driver
RUN cd ~/Universal_Robots_ROS2_Driver \
	&& . /opt/ros/$ROS2_DISTRO/setup.sh \
	&& rosdep install --ignore-src --from-paths src --rosdistro $ROS2_DISTRO -y -r \
	&& colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
	&& echo "\n#UR Driver Setup\nsource ~/Universal_Robots_ROS2_Driver/install/setup.bash" >> ~/.bashrc
	
#########################
##      ModProFT       ##
#########################
WORKDIR /home/${USER}
COPY --chown=${USER} ./ROS_Foxy_Utility_Code ./ROS_Foxy_Utility_Code

RUN cd ~/ROS_Foxy_Utility_Code \
	&& . /opt/ros/$ROS2_DISTRO/setup.sh \
	&& rosdep install --ignore-src --from-paths src -y -r \
	&& colcon build \
	&& echo "\n#ModProFT Setup\nsource ~/ROS_Foxy_Utility_Code/install/setup.bash" >> ~/.bashrc

RUN mkdir -p ~/ROS_Foxy_ModProFT/src \
	&& cd ~/ROS_Foxy_ModProFT 
#	&& . /opt/ros/$ROS2_DISTRO/setup.sh \
#	&& rosdep install --ignore-src --from-paths src -y -r \
#	&& colcon build \
#	&& echo "\n#ModProFT Setup\nsource ~/ROS_Foxy_ModProFT/install/setup.bash" >> /root/.bashrc


# !Command is run as root!
USER root
RUN chown -R ${USER}:${USER} /home/${USER}
USER $USER

#########################
##    Model paths      ##
#########################
# symlink to Models
RUN mkdir -p /home/${USER}/.gazebo/models
#RUN ln -s /home/${USER}/Universal_Robots_ROS2_Driver/install/ur_description/share/#ur_description/ /home/${USER}/.gazebo/models/ur_description
RUN ln -s /home/${USER}/ROS_Foxy_ModProFT/install/modproft_pcb_description/share/modproft_pcb_description/ /home/${USER}/.gazebo/models/modproft_pcb_description

RUN ln -s /home/${USER}/ROS_Foxy_ModProFT/install/modproft_adapter_description/share/modproft_adapter_description/ /home/${USER}/.gazebo/models/modproft_adapter_description

RUN ln -s /home/${USER}/ROS_Foxy_ModProFT/install/modproft_camera_description/share/modproft_camera_description/ /home/${USER}/.gazebo/models/modproft_camera_description

WORKDIR /home/${USER}/
