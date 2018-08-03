FROM osrf/ros:kinetic-desktop-full

LABEL maintainer="Sebastian Höffner <shoeffner@tzi.de>"

# Some packages seem to need cython
RUN apt-get update && apt-get install -y cython

RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
RUN echo "[ ! -f /catkin_ws/devel/setup.bash ] || source /catkin_ws/devel/setup.bash" >> ~/.bashrc

EXPOSE 9090

WORKDIR /catkin_ws
CMD ["/ros_entrypoint.sh", "roscore"]
