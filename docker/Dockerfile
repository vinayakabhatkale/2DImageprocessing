FROM ubuntu:20.04

LABEL maintainer="vinayakabhatkale@gmail.com"

EXPOSE 50001-50004

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
       	libopencv-dev \
       	libyaml-cpp-dev \
       	lsb-release \
       	nano \
       	python3-opencv \
       	unzip \
       	usbutils \
       	software-properties-common \
       	sudo \
       	tmux \
       	vim \
       	wget \
       	libdmtx0b \
       	v4l2loopback-dkms  \
       	gphoto2 \
       	ffmpeg  \
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
    setuptools==58.2.0 \
    pylibdmtx

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
		${PYTHON}-opencv \
		${PYTHON}-gphoto2 \
		${PYTHON}-matplotlib \
		ros-${ROS2_DISTRO}-rosbag2 \
		ros-${ROS2_DISTRO}-rqt \
		ros-${ROS2_DISTRO}-rviz2 \
		ros-${ROS2_DISTRO}-cv-bridge \
		
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

	

RUN mkdir -p /home/${USER}/ROS_Foxy_bhatkale45/src/


# !Command is run as root!
USER root
RUN chown -R ${USER}:${USER} /home/${USER}
USER $USER

WORKDIR /home/${USER}/

