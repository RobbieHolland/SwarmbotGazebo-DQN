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

local function spawn_model(type_spawn, pos, i)
	-- Spawn model of type type_spawn at position pos with index i. Returns spawn code
	local rgsp = 'rosrun gazebo_ros spawn_model -x ' .. pos[1] .. ' -y ' .. pos[2] .. ' -z ' .. pos[3]
  local rgsp_pSdf = rgsp ..' -file `rospack find swarm_simulator`/sdf/'
	local look_up = {
		food     = rgsp_pSdf .. 'food.sdf' .. ' -sdf -model food' .. i .. ' -robot_namespace food' .. i,
		swarmbot = rgsp_pSdf .. 'swarm_robot_v2.sdf' .. ' -sdf -model swarmbot' .. i .. ' -robot_namespace swarmbot' .. i,
		predator = rgsp_pSdf .. 'swarm_robot_v2_predator.sdf' ..  ' -sdf -model predator' .. i .. ' -robot_namespace predator' .. i
	}
	-- Lookup and check type_spawn is implemented
	local spawn_text = assert(look_up[type_spawn], "Type " .. type_spawn .. " not implemented in lookup table")
	os.execute(spawn_text)
	return spawn_text
end


local function spawn_all(type_spawn, num)
	-- Spawn num models of type type_spawn, save code in lookup table spawn_texts
	local spawn_texts = {}
	local init_i = {
 		food = 0,
		swarmbot = 0,
		predator = 0
	}
	-- Check nil num and initial index
	if not num then return spawn_texts end
	local i0 = assert(init_i[type_spawn], "init_i[" .. type_spawn .."] not implemented")
 	--Create in loop
	local position
	for i=i0, num-1 do
		position = arena_width * (torch.rand(3) - 0.5)
		position[3] = 1
		--Create new food/bot
		spawn_texts[type_spawn .. i] = spawn_model(type_spawn, position, i)
	end
	return spawn_texts
end


local function print_results(res) -- for debugging
	print("DEBUGsetup_print_results")
	for  key, val in pairs(res) do
		 print(key, val)
	end
end


-- Spawning all food, bots, predators
local result_food = spawn_all("food",     number_of_food)
local result_bots = spawn_all("swarmbot", number_of_bots)
local result_pred = spawn_all("predator", number_of_pred)
