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
elif [ "$PAPER" == "demo-grid" ]; then
  # GridWorld
	th main.lua -zoom 4 -async A3C -entropyBeta 0.001 -eta 0.0007 -momentum 0.99 -bootstraps 0 -rewardclip 0 -batchSize 5 -hiddenSize 32 -doubleQ false -duel false -optimiser adam -steps 15000000 -tau 4 -memSize 20000 -epsilonSteps 10000 -valFreq 501 -valSteps 500 -bootstraps 0 -PALpha 0 "$@"

#Swarm
elif [ "$PAPER" == "demo-async-swarm" ]; then
    # Tentative automatisation at finding path to SwarmbotDQN folder
    #nameFolder="SwarmbotGazebo-DQN"
    #pathToSwarmDqn=`echo $LUA_PATH | sed "s#\(.*\);\(.*\)$nameFolder/?.lua\(.*\)#\2$nameFolder#"`
    #if [ -z $pathToSwarmDqn ]; then
    #  echo "Atari/run.sh not able to find the path to your $nameFolder. "
    #  echo "Please enter it manually in variable pathToSwarmDqn or update \$LUA_PATH (see README)."
    #  exit 1
    #fi

    pathToAtari="/root/torch-ros-gazebo/Atari"
    case $HOME in
        /Users/pmaal) pathToSwarmDqn="/root/torch-ros-gazebo/SwarmbotGazebo-DQN";;
                                #"/Users/pmaal/Dropbox/Nottingham/repos/phd/torch-ros-gazebo/SwarmbotGazebo-DQN";;
        /home/kai)    pathToSwarmDqn="/home/kai/SwarmbotGazebo-DQN";;
        /root)        pathToSwarmDqn="/root/torch-ros-gazebo/SwarmbotGazebo-DQN";;
        *)            echo "Match HOME!" && exit 1
	  esac

		#Parameters
		BUFFER="1"
		MODE=1
		NUM_FOOD=2 #40
		NUM_BOTS=$((3)) #[Number of bots including number of validation agents]
		NUM_PRED=$((0)) #[Number of predators including valitator]
    ARENA_WIDTH=$((16))
		args="$NUM_FOOD $NUM_BOTS $NUM_PRED"

    function nice_display {
        # Nice display of the arguments
        printf "\n --- $2\n $1\n"
    }

    function exec_in_screen {
        # Execute given string in screen window
        screen_name="$1"
        stringToExecScreen="$2"
        screen -S "$screen_name" -X stuff "${stringToExecScreen}$(echo -ne '\r')"
    }

    function open_in_screen {
        # Launch a SGDQN session if there is none. Use setup_docker.sh to launch container if needed.
        #  To enter a screen:   screen -x SGDQN
        #  To detach from it:   c-a d  # (c- is control key+)
        #  To kill it:          screen -X -S SGDQN quit
        stringToExec=$1
        window_name=$2
        screen_name="SGDQN_${window_name}"
        container_name="levity_torchcontainer"
        if [ -z "`screen -ls | grep "$screen_name"`" ]; then
            screen -d -m -S $screen_name
        fi
        if [ -z "`docker ps | grep "$container_name"`" ]; then
            sh ../setup_docker.sh
        fi
        # In screen window, enter the container. If already in container, error message (add ;exit after second command otherwise)
        echo "screen -x $screen_name # to look at this process"
        exec_in_screen "$screen_name" "docker exec -it $container_name bash"
        # In screen window now in container, execute code
        exec_in_screen "$screen_name" "cd $pathToAtari; $stringToExec"
    }

    function open_in_gnome {
        # Open in new terminal window with gnome-terminal
        gnome-terminal -t "$2" -e "bash -c \"$1 ; exec bash\""
    }

    function exec_in_new_window {
        # Alternative solutions to launch command in a new window, depending on the computer setup
        stringToExec=$1
        comment=$2
        window_name=`echo "$comment" | sed "s/  */_/g"`
        case $HOME in
            /Users/pmaal)   fun="open_in_screen";;
            *)              fun="open_in_gnome";;
        esac
        $fun "$stringToExec" "$window_name"
    }

		function exec_in_win {
		      # Execute code (string $2) in new or current terminal ($1)
		      win=$1
		      stringToExec=$2
		      comment=$3
		      nice_display "$stringToExec" "$comment"
		      # Launch in current terminal or in a new one and script continues running
		      case $win in
		          cur) $stringToExec ;;
		          wai) $stringToExec && echo "sleep 0.5" && sleep 0.5 ;;
		          new) exec_in_new_window "$stringToExec" "$comment" ;; # Depends on the computer
		          *) echo "Option not recognised in exec_in_win" && exit 1;;
		      esac
		}

		function run_environment {
		      # Runs a given number of bots ($1) in given environment ($2)
			NumThreads=$1
			GazeboEnv=$2
			# Shifts used to preserve the original "$@" way of writing the calls.
			shift
			shift
	 		echo "th main.lua -threads $NumThreads -zoom 4 -env $GazeboEnv -modelBody SwarmbotModel -histLen 4 -async A3C -entropyBeta 0 -eta 0.0001 -bootstraps 0 -rewardClip 0 -hiddenSize 512 -doubleQ false -duel false -optimiser sharedRmsProp -steps 6750000 -valFreq 501 -valSteps 12000 -PALpha 0 $@"
		}

    # Instead of 'wai', setup.lua should be in current window (cur) to setup things. Complex if sent to docker by screen, so just sleep 15 for now
		#launch="soup_plus.launch" launch="soup_plus_single.launch" launch="soup_black"
	  exec_in_win  "new"  "roslaunch swarm_simulator soup_black.launch gui:=false"  "Load gazebo with arena world"
		exec_in_win  "wai"  "th $pathToSwarmDqn/setup.lua $args $ARENA_WIDTH"  "Load models into the world"
		exec_in_win  "new"  "th $pathToSwarmDqn/positions.lua"  "Throttle position updates"
		exec_in_win  "new"  "th $pathToSwarmDqn/rewards.lua $MODE $args $ARENA_WIDTH"  "Load program to allocate rewards"

		if [ "$BUFFER" == "1" ]; then
			exec_in_win  "new"  "th $pathToSwarmDqn/buffers/command_buffer.lua $NUM_BOTS $NUM_PRED"  "Load command buffer"
		fi

    NumBotsForEnvironment=$(($NUM_BOTS-1))
		exec_in_win  "new"  "`run_environment $NumBotsForEnvironment GazeboEnv $@`"  "Run GazeboEnv in Atari"
		#exec_in_win  "new"  "`run_environment $NUM_PRED GazeboEnvPred $@`"  "Run GazeboEnvPred in Atari"

		#To load previous weights: -network async/SwarmbotGazebo-DQN/Experiments/GazeboEnv_10-Worked/Weights/last.weights.t7
		# -network GazeboEnv/last.weights.t7 -mode eval -_id GazeboEnv

else
  echo "Invalid options"
fi
