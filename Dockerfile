FROM ros:noetic-ros-base-focal

ARG OVERLAY_WS=<Insert workspace path here>

ENV DEBIAN_FRONTEND noninteractive

ENV ROS_DISTRO noetic

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt update

RUN apt install python3-rosdep --allow-unauthenticated -y

########
# Task 5:
COPY 
# Copy source files into the docker image from the host workspace
# Remember to ignore build, devel and resources
# folder structure can mimic the host folder structure 
########

WORKDIR $OVERLAY_WS
########
# Task 5:
RUN
# Install dependencies for the required packages
########
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin_make

ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
    '$isource "<Insert workspace path here>/devel/setup.bash"' \
    /ros_entrypoint.sh

########
# Task 5:
# Insert worspace path wherever necessary
########