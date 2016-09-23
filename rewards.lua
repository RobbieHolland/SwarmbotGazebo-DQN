--Setup
ros = require 'ros'
ros.init('GazeboDQN_rewards')
print('Handling Rewards...')

require 'torch'
srvs = require 'async/SwarmbotGazebo-DQN/srvs'
msgs = require 'async/SwarmbotGazebo-DQN/msgs'
require 'async/SwarmbotGazebo-DQN/food'
require 'async/SwarmbotGazebo-DQN/swarmbot'

--Flags
initialised = false
velocity_updated = false

function connect_cb(name, topic)
  print("subscriber connected: " .. name .. " (topic: '" .. topic .. "')")
end

function disconnect_cb(name, topic)
  print("subscriber diconnected: " .. name .. " (topic: '" .. topic .. "')")
end

--Constants
eat_distance = 0.4
sensor_range = 2
number_of_food = arg[1]
number_of_bots = arg[2]
mode = tonumber(arg[3])
arena_width = 16

--setup ros node and spinner (processes queued send and receive topics)
spinner = ros.AsyncSpinner()
nodehandle = ros.NodeHandle()
service_queue = ros.CallbackQueue()

--Create food
foods = {}
for i=1, number_of_food do
	--Create new food
	foods[i] = food.create(i, nodehandle, 0, 0, 1, 1)
	ros.Duration(0.05):sleep()
	foods[i]:random_relocate(arena_width)
end

--Create swarmbots
swarmbots = {}
for i=0, number_of_bots do
	--Create new swarmbot
	swarmbots[i] = swarmbot.create(i, nodehandle, 0, 0, 1)
	ros.Duration(0.05):sleep()
	swarmbots[i]:random_relocate(arena_width)
end

--Relocate service
function random_relocate_service_handler(request, response, header)
	swarmbots[request.id]:random_relocate(arena_width)
  return true
end
server_energy = nodehandle:advertiseService('/random_relocate_request', srvs.data_request_spec, random_relocate_service_handler, service_queue)

--Speed request service
function speed_request_service_handler(request, response, header)
	response.data = swarmbots[request.id].speed
  return true
end
server_speed = nodehandle:advertiseService('/speed_request', srvs.data_request_spec, speed_request_service_handler, service_queue)

--Calculates current energy of swarmbots[id]
function calculate_energy(id)
	--[[
	while not velocity_updated do
		ros.spinOnce()
	end
	velocity_updated = false
	

	--Movement reward
	swarmbots[id]:add_energy(swarmbots[id].speed)
	--]]

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
server_relocate = nodehandle:advertiseService('/energy_request', srvs.data_request_spec, energy_service_handler, service_queue)

function table_invert(t)
   local s={}
   for k,v in pairs(t) do
     s[v]=k
   end
   return s
end

model_states_initialised = false
--Updates velocities of models
model_state_subscriber = nodehandle:subscribe("/throttled_model_states", msgs.model_states_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
model_state_subscriber:registerCallback(function(msg, header)

	if not model_states_initialised then
		index_lookup = table_invert(msg.name)
		print('ho')
		for i=0, number_of_bots do
			swarmbots[i].model_id = index_lookup[swarmbots[i].model_name]
		end

		for i=1, number_of_food do
			foods[i].model_id = index_lookup[foods[i].model_name]
		end
	end
	model_states_initialised = true

	for i=0, number_of_bots do
		swarmbots[i].velocity[1] = msg.twist[swarmbots[i].model_id].values.linear.x
		swarmbots[i].velocity[2] = msg.twist[swarmbots[i].model_id].values.linear.y
		swarmbots[i].velocity[3] = msg.twist[swarmbots[i].model_id].values.linear.z

		swarmbots[i].position[1] = msg.pose[swarmbots[i].model_id].position.x
		swarmbots[i].position[2] = msg.pose[swarmbots[i].model_id].position.y
		swarmbots[i].position[3] = msg.pose[swarmbots[i].model_id].position.z
		swarmbots[i].orientation = msg.pose[swarmbots[i].model_id].orientation.z

		--Calculate speed for reward
		--swarmbots[i].speed = swarmbots[i].velocity:norm()
	end

	for i=1, number_of_food do
		foods[i].position[1] = msg.pose[foods[i].model_id].position.x
		foods[i].position[2] = msg.pose[foods[i].model_id].position.y
		foods[i].position[3] = msg.pose[foods[i].model_id].position.z
	end

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

		for i=1, number_of_food do
			if not foods[i].edible then
				foods[i].edible = os.clock() - foods[i].immunity_start_time > foods[i].immunity_duration
			end
		end

		for i=0, number_of_bots do
			for j=1, number_of_food do
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

server_energy:shutdown()
server_relocate:shutdown()
server_speed:shutdown()
ros.shutdown()
