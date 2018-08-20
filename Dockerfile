FROM osrf/ros:kinetic-desktop-full

LABEL maintainer="Sebastian Höffner <shoeffner@tzi.de>"

# Some packages seem to need cython
RUN apt-get update \
    && apt-get install -y cython \
    && echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc \
    && echo "[ ! -f /catkin_ws/devel/setup.bash ] || source /catkin_ws/devel/setup.bash" >> ~/.bashrc

EXPOSE 9090

COPY ros_entrypoint.sh /ros_entrypoint.sh
COPY .rosinstall /catkin_ws/.rosinstall

WORKDIR /catkin_ws

RUN /bin/bash -c " \
       rosws update \
    && rosdep update \
    && rosdep install -y --ignore-src --from-paths src/ \
    && /ros_entrypoint.sh catkin_make -j1"

CMD ["roscore"]
