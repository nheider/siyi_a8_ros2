FROM dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04
#FROM ros:humble-ros-base
SHELL ["/bin/bash", "-c"]

# install foxglove bridge

RUN apt-get update 


#RUN apt-cache search foxglove

RUN apt install ros-$ROS_DISTRO-foxglove-bridge -y &&\
	apt-get install nano -y 

RUN git clone https://github.com/AprilRobotics/apriltag.git && \
    cd apriltag && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd ../..

RUN mkdir -p ~/px4_ros_uxrce_dds/src &&\
	cd ~/px4_ros_uxrce_dds/src &&\
	git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
	
# for dusty container it is  /opt/ros/$ROS_DISTRO/install/setup.bash

RUN mkdir -p ~/drone_ws/src/ 

COPY /src/urdf_node /root/drone_ws/src

RUN cd ~/drone_ws/src/ &&\
	git clone https://github.com/nheider/siyi_a8_ros2.git &&\
	git clone https://github.com/christianrauch/apriltag_ros.git &&\
	git clone https://github.com/christianrauch/apriltag_msgs.git && \
 	cd .. &&\
 	. /opt/ros/jazzy/install/setup.bash &&\           
	colcon build 
	
	
# for dusty container it is  /opt/ros/$ROS_DISTRO/install/setup.bash
#RUN mkdir -p ~/drone_ws/src/ &&\
#    cd ~/drone_ws/src/ &&\
#	git clone https://github.com/PX4/px4_msgs.git &&\
#	git clone https://github.com/nheider/siyi_a8_ros2.git &&\
# 	cd .. &&\
# 	. /opt/ros/jazzy/install/setup.bash &&\           
#	colcon build

# Install GStreamer and OpenCV
RUN apt-get update && apt-get install -y \
    libgstreamer1.0-0 \
    gstreamer1.0-tools \
    gstreamer1.0-libav \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-pulseaudio \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    python3-opencv \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro 
    
COPY /src/apriltag_cfg /root/drone_ws/install/apriltag_ros/share/apriltag_ros/cfg/
#COPY /src/apriltag_cfg/tag_36h11_2.yaml /root/drone_ws/install/apriltag_ros/share/apriltag_ros/cfg/tags_36h11.yaml

# ROS is already sourced in bashrc in dustys container 
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc &&\
    echo "source ~/drone_ws/install/setup.bash" >> ~/.bashrc

