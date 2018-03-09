#!/bin/sh

# Get IP address on ethernet
# If you're on a desktop, change wlan0 to eth0
function my_ip() {
    MY_IP=$(/sbin/ifconfig eth0 | awk '/inet/ { print $2 } ' | sed -e s/addr://)
    echo ${MY_IP:-"Not connected"}
}

# Terminal prompt formatting, optional.
# Makes your terminal look like [host (c1) ~/dir], in purple.
# Search for "bash ps1" online to learn more.
PS1='\[\e[1;35m\][\h \w ($ROS_MASTER_HOST)]$ \[\e[m\]'

# Run "setrobot sim" to go to simulation.
# Run "setrobot c1" to connect to Rosie.
# Run "setrobot softshell" to connect to a turtlebot.
# Note "setrobot c1" is equivalent to the realrobot command that is on most machines.
function setrobot() {
  if [ "$1" = "sim" ]; then
    export ROS_HOSTNAME=localhost;
    export ROS_MASTER_HOST=localhost;
    export ROS_MASTER_URI=http://localhost:11311;
    export ROBOT=sim;
  elif [ "$1" = "c1" ]; then
    unset ROBOT;
    unset ROS_HOSTNAME;
    export ROS_MASTER_HOST=$1;
    export ROS_MASTER_URI=http://mayarobot-wired:11311;
    export ROS_IP=`my_ip`;
  elif [ "$1" = "mayarobot-wired" ]; then
    unset ROBOT;
    unset ROS_HOSTNAME;
    export ROS_MASTER_HOST=c1;
    export ROS_MASTER_URI=http://mayarobot-wired:11311;
    export ROS_IP=`my_ip`;
  else
    unset ROBOT;
    unset ROS_HOSTNAME;
    export ROS_MASTER_HOST=$1;
    export ROS_MASTER_URI=http://$1.cs.washington.edu:11311;
    export ROS_IP=`my_ip`;
  fi
}

# comment out if debugging
# setrobot c1

WATCH_DIRS=( \
    ~/catkin_ws/src/tangible/tangible_projection \
    ~/catkin_ws/src/tangible/tangible_projection/src \
    ~/catkin_ws/src/tangible/tangible_projection/src/offline \
    ~/catkin_ws/src/tangible/tangible_projection/include \
)

inotifywait -m ${WATCH_DIRS[@]} -e close_write |
    while read path action file; do
        if [[ "$file" =~ .*cpp$ ]] || [[ "$file" =~ .*h$ ]] || [[ "$file" =~ CMakeLists.txt ]]; then
            package=""
            if [[ $path == *"tangible_projection"* ]]; then
                package="tangible_projection"
            fi

            echo "$file modified, rebuilding $package package and deps..."

            if catkin --force-color build $package | grep "succeeded!"; then
                # rosnode kill process_cloud_main
                echo "succeeded"
            fi
        fi
    done