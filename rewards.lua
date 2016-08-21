ros = require 'ros'
require 'torch'
msgs = require 'async/SwarmbotGazebo-DQN/msgs'
require 'async/SwarmbotGazebo-DQN/food'
require 'async/SwarmbotGazebo-DQN/swarmbot'

--Constants
arena_width = 20
frequency = 10
eat_distance = 0.4
number_of_food = 5
number_of_bots = 2

--setup ros node and spinner (processes queued send and receive topics)
ros.init('GazeboDQN_rewards')
spinner = ros.AsyncSpinner()
nodehandle = ros.NodeHandle()

--Create food
foods = {}
for i=1, number_of_food do
	--Create new food
	foods[i] = food.create(i, nodehandle, 0, 0, 1, 10)
	foods[i]:random_relocate(arena_width)
end

--Create swarmbots
swarmbots = {}
for i=1, number_of_bots do
	--Create new swarmbot
	swarmbots[i] = swarmbot.create(i, nodehandle, 0, 0, 1, 10)
	swarmbots[i]:random_relocate(arena_width)
end

while not ros.isShuttingDown() do
	--Check if any food is eaten
	for i=1, number_of_bots do
		for j=1, number_of_food do
			if (torch.abs(torch.dist(swarmbots[i].position, foods[j].position)) < eat_distance) then
				--print(torch.abs(torch.dist(swarmbots[i].position, foods[j].position)))	
				foods[j]:random_relocate(arena_width)
				swarmbots[i]:consume(foods[j])
			end
		end
		swarmbots[i]:publish_energy()
	end

	ros.spinOnce()
	ros.Duration(1/frequency):sleep()
end
