# ease_ph_rosws

A set of ROS workspaces for the P01/H02 subprojects of EASE.

- [Local installation](#local-installation)
- [Docker installation (macOS)](#docker-installation-macos)
- [Using this project](#using-this-project)


## Local installation

[Install ROS Kinetic](http://wiki.ros.org/kinetic/Installation). Note that on macOS (and Windows) this is very experimental and difficult (up to almost impossible), hence we recommend using [Docker](#docker-installation) there.

Once you have ROS Kinetic installed, git clone this repository at some convenient location. Run the command

```
source /opt/ros/kinetic/setup.bash
```

to make sure the ROS environment variables are set properly. Then cd into the folder `0_unity`. This is a ROS workspace, and to fill it up, run

```
rosws update
rosdep update
rosdep install --ignore-src --from-paths src/
catkin_make
source devel/setup.bash
```

It is important to source the setup.bash from this workspace after you finish a `catkin_make` with no errors. This updates the ROS environment variables, and allows us to use packages in this workspace to compile other packages that depend on them. To continue installation, cd into `1_cram` and again do

```
rosws update
rosdep update
rosdep install --ignore-src --from-paths src/
catkin_make
source devel/setup.bash
```

If you don't have it yet, now is a good time to also install [Unity](https://unity3d.com).


### Activating the workspace

Whenever you want to catkin_make, build, or just use something from one of the workspaces, cd into it and run

```
source devel/setup.bash
```

Remeber to do this whenever you open a new terminal tab in which you want to use the workspace!

Alternatively you can have bash do this for you automatically, by adding the source /path/to/your/workspace/devel/setup.bash command to your bash.rc file, if you expect this to be the default ROS workspace you will be using.


## Docker installation (macOS)

If you decided to use Docker, the steps to get you on speed are different. First, install [Docker](https://www.docker.com/community-edition), [XQuartz](https://www.xquartz.org), and [Unity](https://unity3d.com): `[brew](https://brew.sh) cask install docker XQuartz unity`.

You need to once build the docker image. This should be, in most cases, a one-off task:

```
docker build -t roscore .
```

Now, and whenever your en0 interface address changes, you need to run `xhost + $(ipconfig getifaddr en0)`. This allows your docker container to connect to the XQuartz display. In case XQuartz is not running, you can start it using `open -g -a XQuartz`.

As a next step, you need to run the container. This is a more difficult step, as it requires many parameters:

```
docker run -d \
    --mount type=bind,source="$(pwd)",target=/catkin_ws \
    -e DISPLAY=$(ipconfig getifaddr en0):0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -p 9090:9090 \
    --name roscore roscore
```

This command will
- Start a container named `roscore` which runs the image `roscore`.
- Mount the current directory into the docker container, so that it can read its contents. This is important so that you can change files from the outside and don't have to recreate the image after each change.
- Map the /tmp/.X11-unix sockets between the OS and the container, so that the GUIs can be used from macOS (the environment variable DISPLAY is also part of this process).
- Expose port 9090 used by the rosbridge to allow communication over this port.

The last setup step is to use the container to run rosws and rosdep:

```
docker exec roscore rosws update
docker exec roscore rosdep update
docker exec roscore rosdep install -y --ignore-src --from-paths src/
```

Whenever a code change happens and you need to run `catkin_make`, make sure to run it inside the container (note the `ros_entrypoint.sh`! This ensures proper environment variables.):

```
docker exec roscore /ros_entrypoint.sh catkin_make
```

The easiest way is to once run a bash and just call `catkin_make` inside it:

```
% docker exec -it roscore /bin/bash
root@5a42853ba845:/catkin_ws# catkin_make
```

In that case, however, you will have to run `source devel/setup.bash` after each `catkin_make` which introduces new packages.


## Using this project

After your first `catkin_make`, if all worked well, you will end up with several different directories inside this repository.

One of them is `src/[ease_ph_pr2_scenes](https://github.com/mpomarlan/ease_ph_pr2_scenes)`.

Run one of the scenarios (a [PR2 robot](https://www.willowgarage.com/pages/pr2/overview) nodding its head). Remember to source (local installation) or use the container (docker installation)!

```
roslaunch ease_ph_pr2_scenes scenario_nodding.launch
```

Now start up Unity and load the `src/ease_ph_pr2_scenes` directory as a project. Use `File > Open Scene` and select the `nodding` scene.

If all works, you unity should now be able to play the scene and you should see a robot nodding its head, controlled by ROS!
