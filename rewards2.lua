ros = require 'ros'
ros.init('GazeboDQN_rewards')

require 'torch'
msgs = require 'async/SwarmbotGazebo-DQN/msgs'
require 'async/SwarmbotGazebo-DQN/food'
require 'async/SwarmbotGazebo-DQN/swarmbot'
resp_ready = false
initialised = false
updated = false

function connect_cb(name, topic)
  --print("subscriber connected: " .. name .. " (topic: '" .. topic .. "')")
end

function disconnect_cb(name, topic)
  print("subscriber diconnected: " .. name .. " (topic: '" .. topic .. "')")
end

print('Handling Rewards...')

--Constants
frequency = 10
eat_distance = 0.4
number_of_food = arg[1]
number_of_bots = arg[2]
arena_width = 16

--setup ros node and spinner (processes queued send and receive topics)
spinner = ros.AsyncSpinner()
nodehandle = ros.NodeHandle()

--Create food
foods = {}
for i=1, number_of_food do
	--Create new food
	foods[i] = food.create(i, nodehandle, 0, 0, 1, 100)
end

--Create swarmbots
swarmbots = {}
for i=0, number_of_bots do
	--Create new swarmbot
	swarmbots[i] = swarmbot.create(i, nodehandle, frequency, 0, 0, 1)
end

--Relocate food
for i=1, number_of_food do
	foods[i]:random_relocate(arena_width)
	ros.Duration(0.05):sleep()
end

--Relocate swarmbots
for i=0, number_of_bots do
	swarmbots[i]:random_relocate(arena_width)
	ros.Duration(0.05):sleep()
end

--Setup subscriber for end of episode
episode_end_subscriber = nodehandle:subscribe("/episode_end", msgs.bool_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
episode_end_subscriber:registerCallback(function(msg, header)
	for i=0, number_of_bots do
		swarmbots[i]:random_relocate(arena_width)
		ros.Duration(0.05):sleep()
	end
end)

function table_invert(t)
   local s={}
   for k,v in pairs(t) do
     s[v]=k
   end
   return s
end

--Updates velocities of models
model_state_subscriber = nodehandle:subscribe("/gazebo/model_states", msgs.model_states_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
model_state_subscriber:registerCallback(function(msg, header)
	if not initialised then
		index_lookup = table_invert(msg.name)

		for i=0, number_of_bots do
			swarmbots[i].model_id = index_lookup[swarmbots[i].model_name]
		end

		for i=1, number_of_food do
			foods[i].model_id = index_lookup[foods[i].model_name]
		end
	end
	initialised = true

	for i=0, number_of_bots do
		swarmbots[i].velocity[1] = msg.twist[swarmbots[i].model_id].values.linear.x
		swarmbots[i].velocity[2] = msg.twist[swarmbots[i].model_id].values.linear.y
		swarmbots[i].velocity[3] = msg.twist[swarmbots[i].model_id].values.linear.z

		swarmbots[i].position[1] = msg.pose[swarmbots[i].model_id].position.x
		swarmbots[i].position[2] = msg.pose[swarmbots[i].model_id].position.y
		swarmbots[i].position[3] = msg.pose[swarmbots[i].model_id].position.z

		--Flag if over speed limit
		swarmbots[i].previous_speed = swarmbots[i].speed
		swarmbots[i].speed = swarmbots[i].velocity:norm()
		swarmbots[i].speed_limit_message.data = swarmbots[i].speed > swarmbots[i].speed_limit
		swarmbots[i].speed_limit_publisher:publish(swarmbots[i].speed_limit_message)
	end

	for i=1, number_of_food do
		foods[i].position[1] = msg.pose[foods[i].model_id].position.x
		foods[i].position[2] = msg.pose[foods[i].model_id].position.y
		foods[i].position[3] = msg.pose[foods[i].model_id].position.z
	end

	updated = true
end)



while not ros.isShuttingDown() do

	--Check if all positions have been updated
	while not updated do
		ros.spinOnce()
	end
	updated = false

	--Reset update flags
	for i=0, number_of_bots do
		swarmbots[i].position_updated = false
	end

	--Check if any food is eaten
	for i=0, number_of_bots do
		for j=1, number_of_food do
			if (torch.abs(torch.dist(swarmbots[i].position, foods[j].position)) < eat_distance) then
				foods[j]:random_relocate(arena_width)
				swarmbots[i]:consume(foods[j])
			end
		end
	end

	--Reward for moving forwards
	for i=0, number_of_bots do
		swarmbots[i]:add_energy(swarmbots[i].speed)

		if swarmbots[i].speed / swarmbots[i].previous_speed < 0.2 and swarmbots[i].previous_speed > swarmbots[i].speed_limit / 2 then
			if i == 1 then
				print('Crash')
			end
		end
	end
ros.Duration(1/100):sleep()
	ros.spinOnce()
end
