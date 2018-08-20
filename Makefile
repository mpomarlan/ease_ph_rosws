DC = HOST_IP="$(shell ipconfig getifaddr en0):0" docker-compose
EXEC = ${DC} exec roscore /ros_entrypoint.sh

launch:
	${DC} up --detach
	${EXEC} catkin_make
	${EXEC} roslaunch ease_ph_pr2_scenes scenario_nodding.launch
	${DC} down

init:
	${DC} build
	${EXEC} rosws update
	${EXEC} rosdep update
	${EXEC} rosdep install -y --ignore-src --from-paths src/

