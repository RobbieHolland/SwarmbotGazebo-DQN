**SwarmbotGazebo-DQN**
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

**Theory**
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
3. Train robot to collect food and avoid walls
    1. Use real time sensor information as input to a neural network that outputs commands
    2. Optimise robot's neural network
        1. Genetic Algorithm with Fitness Proportionate Selection
        2. Reinforcement Learning using Deep Q Networks

# **Code Design**
---
### Environment Design
The environment discussed here is completely encapsulated in a [ROS package](https://github.com/RobbieHolland/swarm-gazebo-simulator).

#### Robot Design
ROS offers two different coding formats for robot design: SDF and URDF. While URDF is more widely used SDF is easier to learn and use which suited my relatively simple robot design.
As Gazebo is as close to real life as possible it is important to have a well built robot that offers stability and fine motor control while remaining simple and scalable. My basic 'swarmbot' can be built using [this tutorial](http://gazebosim.org/tutorials?tut=build_robot).

Gazebo offers a wide variety of sensors that can be placed on robots. I am using one RGB camera and one scan sensor, each with 90 degree field of view. Since I set both to have the same range and resolution, I can represent the state in RGBD. Plugins in the SDF file then publish the sensory input to ROS topics which I can subscribe to and read in Lua using Torch-ROS. [My basic 'swarmbot' with sensors and plugins attached](https://github.com/RobbieHolland/swarm-gazebo-simulator/blob/master/sdf/swarm_robot_v2.sdf).
At the front of each robot is a mouth part, or bumper, which acts as a collision sensor. When this link collides with something, I can detect what type of object it is and allocate rewards accordingly.

#### Virtual Environment Design
In training I divided the overall task into different problems which required different worlds. I can change the environment the swarmbots inhabit by creating different world files in my ROS package.
I also need to model [food](https://github.com/RobbieHolland/swarm-gazebo-simulator/blob/master/sdf/food.sdf) that the robots can detect and interact with. Food blocks can also be modeled as SDF robots, with the SDF file consisting of a single cube.

###Connecting ROS with Lua and Torch
Each of the following sections describes a script, each of which runs on independent shells on runtime.
####Reward Calculation - [rewards.lua](https://github.com/RobbieHolland/SwarmbotGazebo-DQN/blob/continuous/rewards.lua)
Creates tables of abstract [food](https://github.com/RobbieHolland/SwarmbotGazebo-DQN/blob/continuous/food.lua) and [swarmbot](https://github.com/RobbieHolland/SwarmbotGazebo-DQN/blob/continuous/swarmbot.lua) objects that hold information such as position. Each object offers services such as re-positioning the model. In particular the swarmbot object offers methods such as 'consume' and listens to the collision link topic (previously mentioned as the virtual mouth part).
This script then creates a ROS service which takes a swarmbot ID when called, waits for that given swarmbot's collision status to be updated, and then returns the current energy of that swarmbot. In this way the energy difference, or reward, can be calculated for a given step.
In 'training mode' rewards.lua also ensures that food spawns around the robot.

####From Sensory Input to Action - [GazeboEnv.lua](https://github.com/RobbieHolland/SwarmbotGazebo-DQN/blob/continuous/GazeboEnv.lua)
This file fits the specification provided by the [rlenvs API](https://github.com/Kaixhin/rlenvs) which is used by the Atari Deep Q Learning libraries written with Torch. It is important to know that each swarmbot has it's own instance of this class, and each runs on a different thread where the thread ID corresponds to the swarmbot ID.
The most important method provided by GazeboEnv.lua is **step**. **step** provides the latest action chosen by the action-value function which is then published to the command buffer. After a short duration the reward as a function of all previous actions is found and returned, which is then used to train the network using back-propagation. 
**step** also waits until the swarmbot sensors have been updated so that the latest observation of the environment can be returned. This ensures a meaningful history which can then be used to provide the next action for the subsequent call of **step**. Finally, a terminal flag is also returned that signifies whether the current training episode has concluded.

####Command Buffer - [buffers/](https://github.com/RobbieHolland/SwarmbotGazebo-DQN/tree/continuous/buffers)
The swarmbots are acting asynchronously and on different threads but typical RL problems involve one agent. In the interest of keeping the problem in a familiar domain a command_buffer.lua waits to receive all commands from the GazeboEnv.lua objects (i.e. from the network of each agent) and then sends all commands to the physical swarmbots simultaneously.

####Network Design - [SwarmbotModel.lua](https://github.com/RobbieHolland/SwarmbotGazebo-DQN/blob/continuous/SwarmbotModel.lua)
SwarmbotModel.lua provides a method 'createBody' that is used by the Atari code. It returns a Torch Neural Network - I've been experimenting with simple Linear modules and CNN modules. This is the network that will be trained and validated.
