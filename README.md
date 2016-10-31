SwarmbotGazebo-DQN
================
- Simulating multiple agents of a swarm in Gazebo 2.2.3 (http://gazebosim.org/) on ROS Indigo (http://wiki.ros.org/indigo) using 'rlenvs' and 'Atari' repositories from https://github.com/Kaixhin/Atari for the DQN.

- Clone this project into Atari/async of Atari and then add the following to `run.sh`:

- Insert the following into `run.sh` of Atari:

```sh
#Swarm  
elif [ "$PAPER" == "demo-async-swarm" ]; then  
	#Load gazebo with arena world  
	gnome-terminal -e "bash -c \"roslaunch swarm_simulator soup.launch gui:=false ; exec bash\""  
	#Get number of food and number of bots to be used  
	NUM_FOOD=2  
	NUM_BOTS=$((3 - 1))  
	args=$NUM_FOOD  
	args="$args $NUM_BOTS"  
	#Load models into the world  
	th async/SwarmbotGazebo-DQN/setup.lua $args  
	#Load program to allocate rewards  
	setup_command="th async/SwarmbotGazebo-DQN/rewards.lua "  
	setup\_command="$setup_command $args"  
	gnome-terminal -e "bash -c \"$setup_command ; exec bash\""  
  th main.lua -threads $NUM_BOTS -zoom 4 -env async/SwarmbotGazebo-DQN/GazeboEnv -modelBody async/SwarmbotGazebo-DQN/SwarmbotModel -histLen 4 -async A3C -entropyBeta 0.001 -eta 0.0007 -momentum 0.99 -bootstraps 0 -rewardClip 0 -batchSize 5 -hiddenSize 32 -doubleQ false -duel false -optimiser adam -steps 1000000000 -tau 7250 -memSize 20000 -epsilonSteps 10000 -valFreq 60000 -valSteps 40000 -bootstraps 0 -PALpha 0 "$@"
```
