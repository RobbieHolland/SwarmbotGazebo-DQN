--Constants
number_of_food = arg[1]
number_of_bots = arg[2]

--Create food
foods = {}
for i=1, number_of_food do
	--Create new food
	os.execute('rosrun gazebo_ros spawn_model -x ' .. 0 .. ' -y ' .. 0 .. ' -z ' .. 1 .. 
						 ' -file `rospack find swarm_simulator`/sdf/food.sdf' .. ' -sdf -model food' .. i .. ' -robot_namespace food' .. i)	
end

--Create swarmbots
swarmbots = {}
for i=0, number_of_bots do
	--Create new swarmbot
	os.execute('rosrun gazebo_ros spawn_model -x ' .. 0 .. ' -y ' .. 0 .. ' -z ' .. 1 .. ' -file `rospack find swarm_simulator`/sdf/swarm_robot_v2.sdf' .. 
						 ' -sdf -model swarmbot' .. i .. ' -robot_namespace swarmbot' .. i)
end
