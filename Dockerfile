FROM osrf/ros:humble-desktop-full

RUN mkdir -p /autonav/autonav_ws/src

COPY autonav_ws/src /autonav/autonav_ws/src

WORKDIR /autonav/autonav_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash; colcon build && colcon test"