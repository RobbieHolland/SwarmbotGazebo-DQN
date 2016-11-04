#!/bin/bash

# Switch to script directory
cd `dirname -- "$0"`

# Specify paper/hyperparameters
if [ -z "$1" ]; then
  echo "Please enter paper, e.g. ./run nature"
  echo "Atari Choices: nature|doubleq|duel|prioritised|priorduel|persistent|bootstrap|recurrent|async-nstep|async-a3c"
  echo "Catch Choices: demo|demo-async|demo-async-a3c"
  echo "Example Choices: demo-grid"
  exit 0
else
  PAPER=$1
  shift
fi

# Specify game
if ! [[ "$PAPER" =~ demo ]]; then
  if [ -z "$1" ]; then
    echo "Please enter game, e.g. ./run nature breakout"
    exit 0
  else
    GAME=$1
    shift
  fi
fi

if [[ "$PAPER" =~ async ]]; then
  echo "Async mode specified, setting OpenMP threads to 1"
  export OMP_NUM_THREADS=1
fi

if [ "$PAPER" == "demo" ]; then
  # Catch demo
  th main.lua -gpu 0 -zoom 4 -hiddenSize 32 -optimiser adam -steps 500000 -learnStart 50000 -tau 4 -memSize 50000 -epsilonSteps 10000 -valFreq 10000 -valSteps 6000 -bootstraps 0 -memPriority rank -PALpha 0 "$@"
elif [ "$PAPER" == "nature" ]; then
  # Nature
  th main.lua -env rlenvs.Atari -modelBody models.Atari -game $GAME -height 84 -width 84 -colorSpace y -duel false -bootstraps 0 -epsilonEnd 0.1 -tau 10000 -doubleQ false -PALpha 0 -eta 0.00025 -gradClip 0 "$@"
elif [ "$PAPER" == "doubleq" ]; then
  # Double-Q (tuned)
  th main.lua -env rlenvs.Atari -modelBody models.Atari -game $GAME -height 84 -width 84 -colorSpace y -duel false -bootstraps 0 -PALpha 0 -eta 0.00025 -gradClip 0 "$@"
elif [ "$PAPER" == "duel" ]; then
  # Duel (eta is apparently lower but not specified in paper)
  # Note from Tom Schaul: Tuned DDQN hyperparameters are used
  th main.lua -env rlenvs.Atari -modelBody models.Atari -game $GAME -height 84 -width 84 -colorSpace y -bootstraps 0 -PALpha 0 -eta 0.00025 "$@"
elif [ "$PAPER" == "prioritised" ]; then
  # Prioritised (rank-based)
  th main.lua -env rlenvs.Atari -modelBody models.Atari -game $GAME -height 84 -width 84 -colorSpace y -duel false -bootstraps 0 -memPriority rank -alpha 0.7 -betaZero 0.5 -PALpha 0 -gradClip 0 "$@"
elif [ "$PAPER" == "priorduel" ]; then
  # Duel with rank-based prioritised experience replay (in duel paper)
  th main.lua -env rlenvs.Atari -modelBody models.Atari -game $GAME -height 84 -width 84 -colorSpace y -bootstraps 0 -memPriority rank -alpha 0.7 -betaZero 0.5 -PALpha 0 "$@"
elif [ "$PAPER" == "persistent" ]; then
  # Persistent
  th main.lua -env rlenvs.Atari -modelBody models.Atari -game $GAME -height 84 -width 84 -colorSpace y -duel false -bootstraps 0 -epsilonEnd 0.1 -tau 10000 -doubleQ false -eta 0.00025 -gradClip 0 "$@"
elif [ "$PAPER" == "bootstrap" ]; then
  # Bootstrap
  th main.lua -env rlenvs.Atari -modelBody models.Atari -game $GAME -height 84 -width 84 -colorSpace y -duel false -tau 10000 -PALpha 0 -eta 0.00025 -gradClip 0 "$@"
elif [ "$PAPER" == "recurrent" ]; then
  # Recurrent (note that evaluation methodology is different)
  th main.lua -env rlenvs.Atari -modelBody models.Atari -game $GAME -height 84 -width 84 -colorSpace y -histLen 10 -duel false -bootstraps 0 -recurrent true -memSize 400000 -memSampleFreq 1 -epsilonEnd 0.1 -tau 10000 -doubleQ false -PALpha 0 -optimiser adadelta -eta 0.1 "$@"

# Async modes
elif [ "$PAPER" == "demo-async" ]; then
  # N-Step Q-learning Catch demo
  th main.lua -zoom 4 -async NStepQ -eta 0.00025 -momentum 0.99 -bootstraps 0 -batchSize 5 -hiddenSize 32 -doubleQ false -duel false -optimiser adam -steps 15000000 -tau 4 -memSize 20000 -epsilonSteps 10000 -valFreq 10000 -valSteps 6000 -bootstraps 0 -PALpha 0 "$@"
elif [ "$PAPER" == "demo-async-a3c" ]; then
  # A3C Catch demo
  th main.lua -zoom 4 -async A3C -entropyBeta 0.001 -eta 0.0007 -momentum 0.99 -bootstraps 0 -batchSize 5 -hiddenSize 32 -doubleQ false -duel false -optimiser adam -steps 15000000 -tau 4 -memSize 20000 -epsilonSteps 10000 -valFreq 10000 -valSteps 6000 -bootstraps 0 -PALpha 0 "$@"
elif [ "$PAPER" == "async-nstep" ]; then
  # Steps for "1 day" = 80 * 1e6; for "4 days" = 1e9
  th main.lua -env rlenvs.Atari -modelBody models.Atari2013 -hiddenSize 256 -game $GAME -height 84 -width 84 -colorSpace y -async NStepQ -bootstraps 0 -batchSize 5 -momentum 0.99 -rmsEpsilon 0.1 -steps 80000000 -duel false -tau 40000 -optimiser sharedRmsProp -epsilonSteps 4000000 -doubleQ false -PALpha 0 -eta 0.0007 -gradClip 0 "$@"
elif [ "$PAPER" == "async-a3c" ]; then
  th main.lua -env rlenvs.Atari -modelBody models.Atari2013 -hiddenSize 256 -game $GAME -height 84 -width 84 -colorSpace y -async A3C -bootstraps 0 -batchSize 5 -momentum 0.99 -rmsEpsilon 0.1 -steps 80000000 -duel false -tau 40000 -optimiser sharedRmsProp -epsilonSteps 4000000 -doubleQ false -PALpha 0 -eta 0.0007 -gradClip 0 "$@"

#Swarm
elif [ "$PAPER" == "demo-async-swarm" ]; then
  pathToSwarmDqn="/home/kai/SwarmbotGazebo-DQN"
	#Parameters
	BUFFER="1"
	MODE=1
	NUM_FOOD=2 #40
	NUM_BOTS=$((2 - 1)) #[Number of bots including number of validation agents] - [Number of validation agents]
  NUM_PRED=$((2 - 1)) #[Number of predators including valitator] - [Number of validation agents]
	#STAT_UPDATE_TIME=320
	args="$NUM_FOOD $NUM_BOTS $NUM_PRED"

	function execute_in_new_window {
		stringToExec=$1
		comment=$2
		echo " --- $2"
		echo " $1"
		gnome-terminal -t "$2" -e "bash -c \"$stringToExec ; exec bash\""
	}
	
		#gnome-terminal -e "bash -c \"roslaunch swarm_simulator soup_plus.launch gui:=false ; exec bash\""

		#gnome-terminal -e "bash -c \"roslaunch swarm_simulator soup_plus_single.launch gui:=false ; exec bash\""

	execute_in_new_window "roslaunch swarm_simulator soup_black.launch gui:=false"	"#Load gazebo with arena world"
	execute_in_new_window "th $pathToSwarmDqn/setup.lua $args" 											"#Load models into the world"
	execute_in_new_window "th $pathToSwarmDqn/positions.lua" 	 											"#Throttle position updates"
	execute_in_new_window "th $pathToSwarmDqn/rewards.lua $MODE $args" 							"#Load program to allocate rewards"
	
		#Load the statistics program
		#	base_command="th async/SwarmbotGazebo-DQN/statistics.lua "
		#	agent_id=0
		#	stats_command="$base_command $STAT_UPDATE_TIME $agent_id"
		#	gnome-terminal -e "bash -c \"$stats_command ; exec bash\""
		#	agent_id=1
		#	stats_command="$base_command $STAT_UPDATE_TIME $agent_id"
		#	gnome-terminal -e "bash -c \"$stats_command ; exec bash\""
	if [ "$BUFFER" == "1" ]; then 
		execute_in_new_window "th $pathToSwarmDqn/command_buffer.lua $NUM_BOTS $NUM_PRED" "#Load command buffer"
	fi

	#Run the Atari code (async/SwarmbotGazebo-DQN/ removed for GazeboEnv and SwarmbotModel)
		function run_environment { 
			# Runs a given number of bots in given environment.
			NumThreads=$1
			GazeboEnv=$2			
			# Shifts used to preserve the original "$@" way of writing the calls.
			shift			
			shift
 			echo "th main.lua -threads $NumThreads -zoom 4 -env $GazeboEnv -modelBody SwarmbotModel -histLen 4 -async A3C -entropyBeta 0 -eta 0.0001 -bootstraps 0 -rewardClip 0 -hiddenSize 512 -doubleQ false -duel false -optimiser sharedRmsProp -steps 6750000 -valFreq 501 -valSteps 12000 -PALpha 0 $@"
		}

		execute_in_new_window "`run_environment $NUM_BOTS GazeboEnv $@`" "#Run GazeboEnv "
		#execute_in_new_window "`run_environment $NUM_PRED GazeboEnvPred $@`" "#Run GazeboEnvPred "

		


# th main.lua -threads 2 -zoom 4 -env GazeboEnvPred -modelBody SwarmbotModel -histLen 4 -async A3C -entropyBeta 0 -eta 0.0001 -bootstraps 0 -rewardClip 0 -hiddenSize 512 -doubleQ false -duel false -optimiser sharedRmsProp -steps 6750000 -valFreq 501 -valSteps 12000 -PALpha 0


	#To load previous weights: -network async/SwarmbotGazebo-DQN/Experiments/GazeboEnv_10-Worked/Weights/last.weights.t7
	# -network GazeboEnv/last.weights.t7 -mode eval -_id GazeboEnv

# Examples
elif [ "$PAPER" == "demo-grid" ]; then
  # GridWorld
	th main.lua -zoom 4 -async A3C -entropyBeta 0.001 -eta 0.0007 -momentum 0.99 -bootstraps 0 -rewardclip 0 -batchSize 5 -hiddenSize 32 -doubleQ false -duel false -optimiser adam -steps 15000000 -tau 4 -memSize 20000 -epsilonSteps 10000 -valFreq 501 -valSteps 500 -bootstraps 0 -PALpha 0 "$@"
else
  echo "Invalid options"
fi
