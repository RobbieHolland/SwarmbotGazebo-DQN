**SwarmbotGazebo-DQN**
===
---

#### INSTALL

1. Simulating multiple agents of a swarm in [Gazebo 2.2.3](http://gazebosim.org/) on [ROS Indigo](http://wiki.ros.org/indigo) using [rlenvs](https://github.com/Kaixhin/rlenvs) and [Atari](https://github.com/Kaixhin/Atari) repositories for the Deep-Q-Network (DQN).
2. Clone this project in a folder that your LUA_PATH can access.
3. Clone the [swarmbot-gazebo ROS Package](https://github.com/RobbieHolland/swarm-gazebo-simulator) which contains the robots and training worlds.
4. Replace 'Atari/run.sh' with 'SwarmbotGazebo-DQN/ModifiedAtariFiles/run.sh'.

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

| Two robots - one is sensing three food blocks while the other sees nothing  |
| ------------- |
|  <img src="http://i.imgur.com/AwFvLFu.png" width="568">  |

At the front of each robot is a mouth part, or bumper, which acts as a collision sensor. When this link collides with something, I can detect what type of object it is and allocate rewards accordingly.

| Top           | Bottom        |
| ------------- |:-------------:|
| <img src="http://i.imgur.com/IjTXonz.png" height="200">      | <img src="http://i.imgur.com/kotaFuC.png" height="200"> |


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

# **Training**
---
Note that this project is still in progress. Currently I am attempting to translate successes found in simpler tasks to more complex tasks. The results given in this section are those most up to date at the date of writing.

### Genetic Algorithms
Initially I attempted to learn the swarm robots using genetic algorithms. I used roulette wheel selection to allocate genes from the old generation of robots to the new generation.

This approach was flawed, as explained in the following analysis.

1. Population Size - Since Gazebo is an accurate simulator of real robots processing a robot is resource intensive. For Physics to work as intended the real time factor should be as close to 1.0 as possible, which constrains the number of robots to roughly 10.
Therefore the search space of the fitness function covered by the robots was not well covered so all useful traits would have to come by mutation.
A small population size means that the population quickly converges on an a local maximum. I used 'fitness proportionate selection' to pass on weights which meant that the median robot was identical to the best robot in a short number of generations.
2. Since real time factor needs to be close to 1.0 there is no way to speed up the simulation so that enough mutations can occur in a reasonable time.
3. At the start of a new generation robots and rewards are randomly placed and oriented so the problem changes each time; this is so that the robots have good general intelligence. This also means that a good trait will not necessarily make its way into the gene pool of the next generation.
4. The robots' neural networks have inputs of size 4x4x30 with the last layer holding 512 nodes to an output of size 3, corresponding to 3 actions. Genetic algorithms are typically used for smaller networks.

### Deep Q Networks and Reinforcement Learning
A more appropriate method for learning is Reinforcement Learning, especially as I am trying to learn behaviours that involve maximising the cumulative reward of an epoch. RL can be seen as a more active approach to learning; adjusting the network in the loss space with a meaningful direction - whereas GAs randomly choose directions and see what works.
RL can be combined with Q function, or an action-value function. When given a state of the environment (for example, sensory input) this function will output an action which is then performed. After the action is complete a new environmental state is reached and the consequential reward is given. The reward can act as an error value and thus can be used to correct the network with back propagation.
The creator of the code base I used to build the network [explains this in more detail](http://torch.ch/blog/2016/04/30/dueling_dqn.html).

# **Results**
---
As with the Training section the results described here are the most up to date for each learning technique. Each task is presented in chronological order of attempt. Note that changes in reward allocation mean comparisons in scores between experiments is meaningless.

### Genetic Algorithms
#### **Task**: Maximise food consumption
##### **Environment**: 16 robots and 80 food in an Infinite space
As previously discussed Genetic Algorithms produced unsatisfactory results. I attempted to keep the problem simple by allowing the robots to have the same starting positions and orientations. This environmental decision, in combination with the effect of gene pool convergence to a local optima, actually led to a decline in score.
<img src="http://i.imgur.com/lBxGW2U.png" height="450">
### Deep Q Networks and Reinforcement Learning
#### **Task**: Maximise speed
##### **Environment**: 2 robots in a Walled arena
In this scenario two robots (one validation robot for statistics, one robot for learning) attempt to maximise their speed. Since the strategy of moving forwards could be learned without sensory input I make the robots move forward by default with options to turn left or right and go straight. The results were as follows:

![2 robots with walls - E15](http://i.imgur.com/C1F4GGp.png)

It takes roughly 100 epochs to learn the strategy of turning away from walls on approach so as not to stop moving. Once this strategy is learned there is no other obvious strategy for improvement since the task is simple.

Anomaly analysis:

 - The large spikes in score at around 120 epochs was likely due to a simulation bug by which the robot interacted with another object in such a way that a large force was created. 
 - The frequent loss of score is due to robots tipping over due to speeding.



#### **Task**: Maximise food consumption
##### **Environment**: 2 robots and 40 food in an Infinite space
In this scenario two robots (one validation robot for statistics, one robot for learning) attempt to maximise the food they consume. 
One concept I had learned over the previous experiments was that continuous feedback (i.e. a non zero reward on most steps) led to better results. Consequently, I also rewarded the robot on how much of it's camera was taken up by food to reward moving towards food, comsumption of which would result in a much higher final reward. This sort of half-way reward measure is crucial in tackling problems where rewards can be sparse. Consuming some food in orders of magnitude more steps makes food consumption a sparse problem.

![2 robots 40 food](http://i.imgur.com/NKjzbT7.png)

Unlike maximising speed in a walled environment this task can be more refinely completed so the score increase takes a more linear trend. The generality of the task results in frequent drops, but this does not explain the sustained drop in score from 170-190 epochs. I observed this phenomenon in another, earlier, experiment.

![Score drop phenomenon](http://i.imgur.com/VYw0qun.png)

Currently, I suspect these drops to be a bug since performance before the drop is resumed afterwards. I do not think that they are the result of explorative learning since a failed explorative attempt would not persist over a number of epochs.

