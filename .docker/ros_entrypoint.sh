#!/bin/bash

rm -rf /tmp/.X*lock
rm -rf /tmp/.X11-unix

tigervncserver -SecurityTypes None
sed -i 's/$(hostname)/localhost/g' /usr/share/novnc/utils/launch.sh
/usr/share/novnc/utils/launch.sh --vnc localhost:5901

set -e

# setup ros environment
export ROS_LOCALHOST_ONLY=1
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/ros2_dev/scara_tutorial_ros2/install/setup.bash"
exec "$@"