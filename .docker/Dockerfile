ARG ROS_DISTRO="humble"
FROM ros:${ROS_DISTRO}

COPY . /ros2_dev/scara_tutorial_ros2
RUN apt update && apt upgrade -y
RUN DEBIAN_FRONTEND=noninteractive apt-get install -qqy \
    mesa-utils \
    net-tools \
    novnc \
    onboard \
    tigervnc-standalone-server \
    tigervnc-xorg-extension \
    x11-apps \
    xfce4 \
 && rm -rf /var/lib/apt/lists/* \
 && mkdir ~/.vnc \
 && echo "#!/bin/sh\nstartxfce4" > ~/.vnc/xstartup \
 && chmod +x ~/.vnc/xstartup

RUN cd ros2_dev && \
    apt update && \
    cd scara_tutorial_ros2 && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --ignore-src --from-paths . -y -r && \
    colcon build --symlink-install
COPY .docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]