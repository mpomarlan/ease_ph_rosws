# ease_ph_rosws

A ROS workspace for the P01/H02 subprojects of EASE.

- [Local installation](#local-installation)
- [Docker setup (macOS)](#docker-setup-macos)
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


## Docker setup (macOS)

If you decided to use Docker, the steps to get you on speed are different. First, install [Docker](https://www.docker.com/community-edition), [Unity](https://unity3d.com) and optionally (if you want to use any ROS UI tools) [XQuartz](https://www.xquartz.org) using [brew](https://brew.sh): `brew cask install docker unity XQuartz`.

There are two scenarios available: nodding and simulated_markers. To run either, use one of the following commands:

```
docker-compose up nodding
docker-compose up simulated_markers
```

You will get some warnings about a missing HOST_IP environment variable – this is fine, unless you plan on using the ROS UI tools.
In that case, run XQuartz and change its settings to allow network connections.
Then add your IP to the list of allowed connections and set HOST_IP accordingly:

```
export HOST_IP=$(ipconfig getifaddr en0)
xhost + ${HOST_IP}
docker-compose up nodding
```


## Using this project

After your first `catkin_make`, if all worked well, you will end up with several different directories inside this repository.

One of them is src/[ease_ph_pr2_scenes](https://github.com/mpomarlan/ease_ph_pr2_scenes).

Run one of the scenarios (a [PR2 robot](https://www.willowgarage.com/pages/pr2/overview) nodding its head). Remember to source (local installation) or use the container (docker installation)!

```
roslaunch ease_ph_pr2_scenes scenario_nodding.launch
```

Now start up Unity and load the `src/ease_ph_pr2_scenes` directory as a project. Use `File > Open Scene` and select the `nodding` scene.

If all works, your unity should now be able to play the scene and you should see a robot nodding its head, controlled by ROS!
