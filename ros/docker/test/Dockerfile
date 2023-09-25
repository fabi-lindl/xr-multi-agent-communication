FROM ros:iron-ros-base

RUN apt-get update &&\
    apt-get install python3-pip -y &&\
    apt-get install ros-iron-cv-bridge -y &&\
    apt-get install ros-iron-vision-opencv -y

COPY ./docker/requirements.txt .
RUN pip3 install -r requirements.txt

ENV PYTHONUNBUFFERED=1

ARG HOST_PKG_PATH=./ros2_ws/src/
ARG WORKSPACE_PATH=/home/ros2_ws/src

RUN mkdir -p $WORKSPACE_PATH

COPY ${HOST_PKG_PATH}rosagent $WORKSPACE_PATH/rosagent
COPY ${HOST_PKG_PATH}rosrouter $WORKSPACE_PATH/rosrouter
COPY ${HOST_PKG_PATH}rosutility $WORKSPACE_PATH/rosutility
COPY ${HOST_PKG_PATH}demo $WORKSPACE_PATH/demo
COPY ${HOST_PKG_PATH}unity_robotics_demo $WORKSPACE_PATH/unity_robotics_demo
COPY ${HOST_PKG_PATH}unity_robotics_demo_msgs $WORKSPACE_PATH/unity_robotics_demo_msgs

RUN cd home/ros2_ws && . /opt/ros/iron/setup.sh && colcon build

RUN echo "cd home/ros2_ws" >> ~/.bashrc
RUN echo "source install/setup.bash" >> ~/.bashrc
