ros = require 'ros'
ros.init('GazeboDQN_rewards')

require 'torch'
msgs = require 'async/SwarmbotGazebo-DQN/msgs'
require 'async/SwarmbotGazebo-DQN/food'
require 'async/SwarmbotGazebo-DQN/swarmbot'
resp_ready = false

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

while not ros.isShuttingDown() do
	--Check if any food is eaten
	for i=0, number_of_bots do
		for j=1, number_of_food do
			if (torch.abs(torch.dist(swarmbots[i].position, foods[j].position)) < eat_distance) then
				--print(torch.abs(torch.dist(swarmbots[i].position, foods[j].position)))	
				foods[j]:random_relocate(arena_width)
				swarmbots[i]:consume(foods[j])
			end
		end
	end

	--Constant health depletion
	for i=0, number_of_bots do

		--Reward for moving forwards
		swarmbots[i]:update_energy(swarmbots[i].average_velocity:norm())
		
	end

	ros.spinOnce()
	ros.Duration(1/frequency):sleep()
end
