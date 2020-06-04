FROM ros:melodic-robot

RUN apt-get update
RUN apt-get install -y python-catkin-tools
RUN apt-get install -y ros-melodic-moveit ros-melodic-moveit-visual-tools ros-melodic-robot-state-publisher

RUN mkdir -p /home/catkin_ws

COPY ros_packages/ /home/catkin_ws/src/
