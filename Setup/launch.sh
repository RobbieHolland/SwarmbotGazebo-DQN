    # Tentative automatisation at finding path to SwarmbotDQN folder
    #nameFolder="SwarmbotGazebo-DQN"
    #pathToSwarmDqn=`echo $LUA_PATH | sed "s#\(.*\);\(.*\)$nameFolder/?.lua\(.*\)#\2$nameFolder#"`
    #if [ -z $pathToSwarmDqn ]; then
    #  echo "Atari/run.sh not able to find the path to your $nameFolder. "
    #  echo "Please enter it manually in variable pathToSwarmDqn or update \$LUA_PATH (see README)."
    #  exit 1
    #fi
		echo $0
    pathToAtari="/root/torch-ros-gazebo/Atari"
    case $HOME in
        /Users/pmaal) pathToSwarmDqn="/root/torch-ros-gazebo/SwarmbotGazebo-DQN";;
        /home/rh2515) pathToSwarmDqn="home/rh2515/SwarmbotGazebo-DQN";;
        /root)        pathToSwarmDqn="/root/torch-ros-gazebo/SwarmbotGazebo-DQN";;
        *)            echo "Match HOME!" && exit 1
	  esac

		#Parameters
		BUFFER="1"
		MODE=1 #0: Spawns food inside arena_width, 1: Spawns food subsets around their designated robots
		NUM_FOOD=$((60))
		NUM_BOTS=$((2)) #[Number of bots including number of validation agents]
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
		          wai) $stringToExec && echo "sleep 0" && sleep 0 ;;
		          new) exec_in_new_window "$stringToExec" "$comment" ;; # Depends on the computer
		          *) echo "Option not recognised in exec_in_win" && exit 1;;
		      esac
		}

		function run_environment {
		      # Runs a given number of bots ($1) in given environment ($2)
			NumThreads=$1
			Agent=$2
			Model=NetworkModels/SwarmbotModel

			# Shifts used to preserve the original "$@" way of writing the calls.
			shift
			shift
	 		echo "th main.lua -threads $NumThreads -zoom 4 -env $Agent -modelBody $Model -histLen 4 -async A3C -entropyBeta 0 -eta 0.0001 -bootstraps 0 -rewardClip 0 -hiddenSize 512 -doubleQ false -duel false -optimiser sharedRmsProp -steps 6750000 -valFreq 501 -valSteps 12000 -PALpha 0 $@"
		}

    # Instead of 'wai', setup.lua should be in current window (cur) to setup things. Complex if sent to docker by screen, so just sleep 15 for now
		#launch="soup_plus.launch" launch="soup_plus_single.launch" launch="soup_black"
	  exec_in_win  "new"  "roslaunch swarm_simulator soup_black.launch gui:=false"  "Load Gazebo with arena world"
		exec_in_win  "cur"  "th $pathToSwarmDqn/Setup/setup.lua $args $ARENA_WIDTH"  "Load models into the world"
		exec_in_win  "new"  "th $pathToSwarmDqn/Rewards/positions.lua"  "Throttle position updates"
		sleep 1
		exec_in_win  "new"  "th $pathToSwarmDqn/Rewards/rewards.lua $MODE $args $ARENA_WIDTH"  "Load program to allocate rewards"
		sleep 3

		if [ "$BUFFER" == "1" ]; then
			exec_in_win  "new"  "th $pathToSwarmDqn/Buffers/command_buffer.lua $NUM_BOTS $NUM_PRED"  "Load command buffer"
		fi

    NumBotsForEnvironment=$(($NUM_BOTS-1))
		Environment=Agents/GazeboEnv
		exec_in_win  "new"  "`run_environment $NumBotsForEnvironment $Environment $@`"  "Run GazeboEnv in Atari"
		#exec_in_win  "new"  "`run_environment $NUM_PRED Agents/GazeboEnvPred $@`"  "Run GazeboEnvPred in Atari"

		#To load previous weights: -network async/SwarmbotGazebo-DQN/Experiments/GazeboEnv_10-Worked/Weights/last.weights.t7
		# -network GazeboEnv/last.weights.t7 -mode eval -_id GazeboEnv
