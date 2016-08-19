# SwarmbotGazebo-DQN
Simulating multiple agents of a swarm in Gazebo 2.2.3 (http://gazebosim.org/) on ROS Indigo (http://wiki.ros.org/indigo) using 'rlenvs' and 'Atari' repositories from https://github.com/Kaixhin for the DQN.

./run.sh demo-swarm = th main.lua -env async/SwarmbotGazebo-DQN/GazeboEnv -modelBody async/SwarmbotGazebo-DQN/SwarmbotModel -histLen 4 -duel false -bootstraps 0 -doubleQ false -PALpha 0 -async A3C "$@"
