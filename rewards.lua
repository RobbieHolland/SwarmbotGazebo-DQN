-- Provides services (energy, swarmbot reallocation, speeds) and
-- continulously reallocate food too far from robot

--Setup
ros = require 'ros'
ros.init('GazeboDQN_rewards')
print('Handling Rewards...')

require 'torch'
srvs = require 'srvs/srvs'
msgs = require 'msgs/msgs'
require 'food'
require 'swarmbot'
require 'swarm_util'

--Flags
initialised = false
velocity_updated = false

--Constants
eat_distance = 0.4
sensor_range = 2
mode  = tonumber(arg[1])
number_of_food = arg[2]
number_of_bots = arg[3]
number_of_pred = arg[4]
arena_width    = tonumber(arg[5])

SWARMBOT_GAZEBO_init_i = { -- Different initial value to account for one more swarmbot
	food = 0,
	swarmbot = 0,
  predator = 0
}

--setup ros node and spinner (processes queued send and receive topics)
spinner = ros.AsyncSpinner()
nodehandle = ros.NodeHandle()
service_queue = ros.CallbackQueue()

local function create_models(type_model, num)
	-- Create num models of type type_model if at least one asked
 	if tonumber(num) < 1 then return {} end

	-- Lookup table to create models
	local lookup = {
		food     = function (i) return food.create(i, nodehandle, 0, 0, 1, 50) end,
		swarmbot = function (i) return swarmbot.create(i, nodehandle, 0, 0, 1, -5, "swarmbot") end,
		predator = function (i) return swarmbot.create(i, nodehandle, 0, 0, 1, -20, "predator") end
	}

	-- Test the type_model is implemented in lookup table
	local fun = assert(lookup[type_model], "lookup[ " .. type_model .. " ] not implemented yet")
 	local i0  = assert(SWARMBOT_GAZEBO_init_i[type_model], "init_i[ " .. type_model .. " ] not implemented yet")

	--Create new food/bot/... in loop
	local res = {}
	for i=i0, num-1 do
		res[i] = fun(i)
		ros.Duration(0.05):sleep()
		res[i]:random_relocate(arena_width)
	end
	return res
end

--Create food and swarmbots
foods     = create_models("food",     number_of_food)
swarmbots = create_models("swarmbot", number_of_bots)
predators = create_models("predator", number_of_pred)

--Relocate service
function random_relocate_service_handler(request, response, header)
	swarmbots[request.id]:random_relocate(arena_width)
  return true
end
server_relocate = nodehandle:advertiseService('/random_relocate_request', srvs.data_request_spec, random_relocate_service_handler, service_queue)

--Speed request service
function speed_request_service_handler(request, response, header)
	response.data = swarmbots[request.id].speed
  return true
end
server_speed = nodehandle:advertiseService('/speed_request', srvs.data_request_spec, speed_request_service_handler, service_queue)

--Calculates current energy of swarmbots[id]
function calculate_energy(id)
	--Collisions reward energy if the colliding object is food
	while not swarmbots[id].collision_updated do
		ros.spinOnce()
	end
	swarmbots[id].collision_updated = false
end

--Energy service
function energy_service_handler(request, response, header)
	calculate_energy(request.id)
	response.data = swarmbots[request.id].energy
  return true
end
server_energy = nodehandle:advertiseService('/energy_request', srvs.data_request_spec, energy_service_handler, service_queue)

function table_invert(t)
   local s={}
   for k,v in pairs(t) do
     s[v]=k
   end
   return s
end

-- Functions to make the initialisation easier to generalise
function SGDQN_add_model_id(res_i, type_model, num, index_lookup)
	-- Refresh model_id's of all elements of type
	local res = {}
	setmetatable(res, {__index = res_i}) -- overloads res_i
	local i0 = assert(SWARMBOT_GAZEBO_init_i[type_model], "Wrong type model:" .. type_model)

	for i=i0, num-1 do
		--print("DEBUGRewards_Refresh, type_model=" .. type_model .. ", i=" .. i .. ", model_name=" .. res[i].model_name)
		res[i].model_id = index_lookup[res[i].model_name]
		--print("DEBUGRewards_Refresh, res[i].model_id=" .. res[i].model_id)
	end
	return res
end


function SGDQN_add_infos_msgs(res, type_model, num, msg)
	local i0 = assert(SWARMBOT_GAZEBO_init_i[type_model], "Wrong type model:" .. type_model)
	for i=i0, num do
	-- print("i=" .. i .. ", type_model=" .. type_model .. ", res[i].model_id=" .. res[i].model_id )
			print("i=" .. i ..",type_model=" .. type_model)
			print("res[i].model_id=")
			print(res[i].model_id)
			print("msg.pose[res[i].model_id].position=")
			print(msg.pose[res[i].model_id].position)
		if type_model == "food" then
			res[i].position[1] = msg.pose[res[i].model_id].position.x
			res[i].position[2] = msg.pose[res[i].model_id].position.y
			res[i].position[3] = msg.pose[res[i].model_id].position.z
		elseif type_model == "swarmbot" or type_model == "predator" then
			res[i].velocity[1] = msg.twist[res[i].model_id].values.linear.x
			res[i].velocity[2] = msg.twist[res[i].model_id].values.linear.y
			res[i].velocity[3] = msg.twist[res[i].model_id].values.linear.z
			res[i].position[1] = msg.pose[res[i].model_id].position.x
			res[i].position[2] = msg.pose[res[i].model_id].position.y
			res[i].position[3] = msg.pose[res[i].model_id].position.z
			res[i].orientation = msg.pose[res[i].model_id].orientation.z
			--Calculate speed for reward
			--res[i].speed = res[i].velocity:norm()
		else
			error("Wrong type model, not implemented in loop:" .. type_model)
		end
	end
	return res
end

model_states_initialised = false
--Updates velocities of models
model_state_subscriber = nodehandle:subscribe("/throttled_model_states", msgs.model_states_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
model_state_subscriber:registerCallback(function(msg, header)

	if not model_states_initialised then
    -- Refresh model_id's by inverting the table
		index_lookup = table_invert(msg.name)
		foods     = SGDQN_add_model_id(foods,     "food",     number_of_food, index_lookup)
		swarmbots = SGDQN_add_model_id(swarmbots, "swarmbot", number_of_bots, index_lookup)
		predators = SGDQN_add_model_id(predators, "predator", number_of_pred, index_lookup)


	end
	model_states_initialised = true

	--foods     = SGDQN_add_infos_msgs(foods,     "food",     number_of_food, msg)
	--swarmbots = SGDQN_add_infos_msgs(swarmbots, "swarmbot", number_of_bots, msg)
	--predators = SGDQN_add_infos_msgs(predators, "predator", number_of_pred, msg)

	velocity_updated = true
end)


if 		 mode == 0 then --Normal mode
	while ros.ok() do
		if not service_queue:isEmpty() then
		  service_queue:callAvailable()
		end
		for i=1, number_of_food do
			if not foods[i].edible then
				foods[i].edible = os.clock() - foods[i].immunity_start_time > foods[i].immunity_duration
			end
		end
		ros.spinOnce()
	end
elseif mode == 1 then --Training mode
	training_range = 4.5

	while ros.ok() do
		if not service_queue:isEmpty() then
		  service_queue:callAvailable()
		end

		for i=0, number_of_food-1 do
			if not foods[i].edible then
				foods[i].edible = os.clock() - foods[i].immunity_start_time > foods[i].immunity_duration
			end
		end

		for i=0, number_of_bots-1 do
			for j=0, number_of_food-1 do
				assigned_bot_id = j % (number_of_bots + 1)
				if torch.dist(swarmbots[assigned_bot_id].position, foods[j].position) > training_range then
					alpha = (torch.uniform() - 0.5) * 2 * math.pi
					r = torch.uniform() * (training_range - sensor_range) + sensor_range
					new_position = swarmbots[assigned_bot_id].position:clone()
					new_position[1] = new_position[1] + r * math.cos(alpha)
					new_position[2] = new_position[2] + r * math.sin(alpha)
					new_position[3] = 1
					foods[j]:relocate(new_position)
				end
			end
		end

		ros.spinOnce()
	end
end

--Shutdown ROS services
server_energy:shutdown()
server_relocate:shutdown()
server_speed:shutdown()
ros.shutdown()
