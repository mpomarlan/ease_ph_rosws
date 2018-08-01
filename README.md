# ease_ph_rosws
A ROS workspace for the PH subprojects of EASE

# Installation

Assuming you already have ROS Kinetic installed, git clone this repository at some convenient location. Then cd into the repository folder and run
```
source /opt/ros/kinetic/setup.bash
rosws update
catkin_make
source devel/setup.bash
```

# Activating the workspace

Whenever you want to catkin_make, build, or just use something from this workspace, cd into it and run
```
source devel/setup.bash
```

Remeber to do this whenever you open a new terminal tab in which you want to use the workspace!

Alternatively you can have bash do this for you automatically, by adding the source /path/to/your/workspace/devel/setup.bash command to your bash.rc file, if you expect this to be the default ROS workspace you will be using.

