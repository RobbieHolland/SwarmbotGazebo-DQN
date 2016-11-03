require("torch")

--Constants
number_of_food = arg[1]
number_of_bots = arg[2]
number_of_pred = arg[3]

-- Declarations
foods = {}
swarmbots = {}


--PARAMETERISE THIS LATER
arena_width = 16

function spawn_model(type_spawn, pos, i)
	-- Spawn model of type type_spawn at position pos with index i. Returns spawn code
	local rosrun_spawn_pos = 'rosrun gazebo_ros spawn_model -x ' .. pos[1] .. ' -y ' .. pos[2] .. ' -z ' .. pos[3]
	local look_up = {
		food 	 = rosrun_spawn_pos .. ' -file `rospack find swarm_simulator`/sdf/food.sdf' ..  					' -sdf -model food' 	.. i .. ' -robot_namespace food' .. i,
		bot 	 = rosrun_spawn_pos .. ' -file `rospack find swarm_simulator`/sdf/swarm_robot_v2.sdf' .. 			' -sdf -model swarmbot' .. i .. ' -robot_namespace swarmbot' .. i,
		predator = rosrun_spawn_pos .. ' -file `rospack find swarm_simulator`/sdf/swarm_robot_v2_predator.sdf' .. 	' -sdf -model predator' .. i .. ' -robot_namespace predator' .. i
	} 
	-- Lookup and check type_spawn is implemented
	local spawn_text = look_up[type_spawn]
	if not spawn_text then error("Type " .. type_spawn .. " not implemented in lookup table of function spawn_model") end
	os.execute(spawn_text)
	return spawn_text
end


function spawn_all(type_spawn, num)
	-- Spawn num models of type type_spawn, save code in lookup table spawn_texts
	local spawn_texts = {}
	-- Check nil num 
	if not num then return spawn_texts end 
	local position
	for i=1, num do
		position = arena_width * (torch.rand(3) - 0.5) -- {[1] = 1, [2] = 2, [3] = 3}--
		position[3] = 1
		--Create new food/bot
		spawn_texts[type_spawn .. i] = spawn_model(type_spawn, position, i)
	end
	return spawn_texts
end


function print_results(res) -- for debugging
	for  key, val in pairs(res) do 
		 print(key, val)
	end
end


-- Spawning all food, bots, predators
local result_food = spawn_all("food", number_of_food) 
local result_bots = spawn_all("bot", number_of_bots) 
local result_pred = spawn_all("predator", number_of_pred) 

