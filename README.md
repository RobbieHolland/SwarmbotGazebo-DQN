SwarmbotGazebo-DQN
===
---

#### INSTALL

1. Simulating multiple agents of a swarm in [Gazebo 2.2.3](http://gazebosim.org/) on [ROS Indigo](http://wiki.ros.org/indigo) using [rlenvs](https://github.com/Kaixhin/rlenvs) and [Atari](https://github.com/Kaixhin/Atari) repositories for the Deep-Q-Network (DQN).
2. Clone this project in a folder that your LUA_PATH can access.
3. Clone the [swarmbot-gazebo ROS Package](https://github.com/RobbieHolland/swarm-gazebo-simulator) which contains the robots and training worlds.
4. Add in Atari/run.sh the `elif [ "$PAPER" == "demo-async-swarm" ]; then` statement you can find in ModifiedAtariFiles/run.sh (or simply replace Atari/run.sh with this file).

#### DEPENDENCIES

- Install all dependencies for 'Atari' and 'rlenvs'
- Luarocks install [torch-ros](https://luarocks.org/modules/xamla/torch-ros)
- gnome-terminal

#### RUN

- Run from repos folder
        `./Atari/run.sh demo-async-swarm`

Theory
===
---

#### Abstract
Transferring techniques for learning Deep Q Networks trained to play 2D Atari games to learn accurately simulated 3D robots performing asynchronous tasks. Gazebo with ROS Indigo provides a realistic robot simulator used to train networks. Learnt behaviours work in physical robots to the extent that the divide between simulation and reality permits.

#### Objectives

1. Configure Environment
    1. Create training worlds
    2. Create creatures (swarmbots)
2. Demonstrate individual control of robots
    1. Receive input from robot sensors via torch-ros
    2. Create commands based on input using Torch
    3. Send commands to robot via torch-ros
3. Train robot to collect food
    1. Use real time sensor information as input to a neural network that outputs commands
    2. Optimise robot's neural network
        1. Genetic Algorithm with Fitness Proportionate Selection
        2. Reinforcement Learning using Deep Q Networks
