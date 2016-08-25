SwarmbotGazebo-DQN
================
- Simulating multiple agents of a swarm in Gazebo 2.2.3 (http://gazebosim.org/) on ROS Indigo (http://wiki.ros.org/indigo) using 'rlenvs' and 'Atari' repositories from https://github.com/Kaixhin/Atari for the DQN.

- Clone this project into Atari/async of Atari and then add the following to run.sh:

- Insert the following into run.sh of Atari:

\#Swarm
elif [ "$PAPER" == "demo-async-swarm" ]; then__
	#Load gazebo with arena world__
	gnome-terminal -e "bash -c \"roslaunch swarm_simulator soup.launch gui:=false ; exec bash\""__
	#Get number of food and number of bots to be used__
	NUM_FOOD=2__
	NUM_BOTS=$((3 - 1))__
	args=$NUM_FOOD__
	args="$args $NUM_BOTS"__
	#Load models into the world__
	th async/SwarmbotGazebo-DQN/setup.lua $args__
	#Load program to allocate rewards__
	setup_command="th async/SwarmbotGazebo-DQN/rewards.lua "__
	setup\_command="$setup_command $args"__
	gnome-terminal -e "bash -c \"$setup_command ; exec bash\""__
  th main.lua -threads $NUM_BOTS -zoom 4 -env async/SwarmbotGazebo-DQN/GazeboEnv -modelBody async/SwarmbotGazebo-DQN/SwarmbotModel -histLen 4 -async A3C -entropyBeta 0.001 -eta 0.0007 -momentum 0.99 -bootstraps 0 -rewardClip 0 -batchSize 5 -hiddenSize 32 -doubleQ false -duel false -optimiser adam -steps 1000000000 -tau 7250 -memSize 20000 -epsilonSteps 10000 -valFreq 60000 -valSteps 40000 -bootstraps 0 -PALpha 0 "$@"__
