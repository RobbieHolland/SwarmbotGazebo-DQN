--Constants
number_of_food = arg[1]
number_of_bots = arg[2]

--PARAMETERISE THIS LATER
arena_width = 16

--Create food
foods = {}
for i=1, number_of_food do
	position = arena_width * (torch.rand(3) - 0.5)
	position[3] = 1
	--Create new food
	os.execute('rosrun gazebo_ros spawn_model -x ' .. position[1] .. ' -y ' .. position[2] .. ' -z ' .. position[3] .. 
						 ' -file `rospack find swarm_simulator`/sdf/food.sdf' .. ' -sdf -model food' .. i .. ' -robot_namespace food' .. i)	
end

--Create swarmbots
swarmbots = {}
for i=0, number_of_bots do
	position = arena_width * (torch.rand(3) - 0.5)
	position[3] = 1
	--Create new swarmbot
	os.execute('rosrun gazebo_ros spawn_model -x ' .. position[1] .. ' -y ' .. position[2] .. ' -z ' .. position[3] .. 
						 ' -file `rospack find swarm_simulator`/sdf/swarm_robot_v2.sdf' .. ' -sdf -model swarmbot' .. i .. ' -robot_namespace swarmbot' .. i)
end
