SwarmbotGazebo-DQN
================

INSTALL

- Simulating multiple agents of a swarm in Gazebo 2.2.3 (http://gazebosim.org/) on ROS Indigo (http://wiki.ros.org/indigo) using 'rlenvs' and 'Atari' repositories from https://github.com/Kaixhin/Atari for the Deep-Q-Network (DQN). 

- Clone this project in a folder that your LUA_PATH can access. If you do not know what this means, read the following to understand how to setup environment variables: [get ref].

- Add in Atari/run.sh the `elif [ "$PAPER" == "demo-async-swarm" ]; then` statement you can find in ModifiedAtariFiles/run.sh (or simply replace Atari/run.sh with this file).

DEPENDENCIES

- Install all dependencies for 'Atari' and 'rlenvs'
- gnome-terminal

RUN

- Run from repos folder
	`Atari/run.sh demo-async-swarm`
