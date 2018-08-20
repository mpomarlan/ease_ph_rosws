# ease_ph_rosws

A ROS workspace for the P01/H02 subprojects of EASE.

- [Local installation](#local-installation)
- [Docker installation (macOS)](#docker-installation-macos)
- [Using this project](#using-this-project)


## Local installation

[Install ROS Kinetic](http://wiki.ros.org/kinetic/Installation). Note that on macOS (and Windows) this is very experimental and difficult (up to almost impossible), hence we recommend using [Docker](#docker-installation-macos) there.

Once you have ROS Kinetic installed, git clone this repository at some convenient location. Then cd into the repository folder and run

```
source /opt/ros/kinetic/setup.bash
rosws update
rosdep update
rosdep install --ignore-src --from-paths src/
catkin_make
source devel/setup.bash
```

If you don't have it yet, now is a good time to also install [Unity](https://unity3d.com).


### Activating the workspace

Whenever you want to catkin_make, build, or just use something from this workspace, cd into it and run

```
source devel/setup.bash
```

Remeber to do this whenever you open a new terminal tab in which you want to use the workspace!

Alternatively you can have bash do this for you automatically, by adding the source /path/to/your/workspace/devel/setup.bash command to your bash.rc file, if you expect this to be the default ROS workspace you will be using.


## Docker installation (macOS)

If you decided to use Docker, the steps to get you on speed are different. First, install [Docker](https://www.docker.com/community-edition), [XQuartz](https://www.xquartz.org), and [Unity](https://unity3d.com) using [brew](https://brew.sh): `brew cask install docker XQuartz unity`.

You need to once build the docker image and install the dependencies. This should be, in most cases, a one-off task:

```
make init
```

To build and run the project, you can use a simple make:

```
make
```

This will run the nodding example scene. For other scenes, do the following:

```
make init  # once, as before
docker-compose up --detach  # Run the roscore service
docker-compose exec roscore /ros_entrypoint.sh catkin_make  # Compile changes (skip if none)
docker-compose exec roscore /ros_entrypoint.sh  roslaunch ease_ph_pr2_scenes scenario_nodding.launch
    # Replace "scenario_nodding" with the scene you want
docker-compose down  # Once you are done, to shut down the container

Whenever a code change happens and you need to run `catkin_make`, make sure to run it inside the container (note the `ros_entrypoint.sh`! This ensures proper environment variables.):

```
docker-compose exec roscore /ros_entrypoint.sh catkin_make
```

The easiest way is to once run a bash and just call `catkin_make` inside it:

```
% docker-compose exec roscore /bin/bash
root@5a42853ba845:/catkin_ws# catkin_make
```

In that case, however, you will have to run `source devel/setup.bash` after each `catkin_make` which introduces new packages.


## Using this project

After your first `catkin_make`, if all worked well, you will end up with several different directories inside this repository.

One of them is src/[ease_ph_pr2_scenes](https://github.com/mpomarlan/ease_ph_pr2_scenes).

Run one of the scenarios (a [PR2 robot](https://www.willowgarage.com/pages/pr2/overview) nodding its head). Remember to source (local installation) or use the container (docker installation)!

```
roslaunch ease_ph_pr2_scenes scenario_nodding.launch
```

Now start up Unity and load the `src/ease_ph_pr2_scenes` directory as a project. Use `File > Open Scene` and select the `nodding` scene.

If all works, your unity should now be able to play the scene and you should see a robot nodding its head, controlled by ROS!
