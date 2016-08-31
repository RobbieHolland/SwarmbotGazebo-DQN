ros = require 'ros'
require 'torch'
msgs = require 'async/SwarmbotGazebo-DQN/msgs'
require 'async/SwarmbotGazebo-DQN/food'
require 'async/SwarmbotGazebo-DQN/swarmbot'

print('Handling Rewards...')

--Constants
frequency = 10
eat_distance = 0.4
number_of_food = arg[1]
number_of_bots = arg[2]
arena_width = 16

--setup ros node and spinner (processes queued send and receive topics)
ros.init('GazeboDQN_rewards')
spinner = ros.AsyncSpinner()
nodehandle = ros.NodeHandle()

--Create food
foods = {}
for i=1, number_of_food do
	--Create new food
	foods[i] = food.create(i, nodehandle, 0, 0, 1, 10)
end

--Create swarmbots
swarmbots = {}
for i=0, number_of_bots do
	--Create new swarmbot
	swarmbots[i] = swarmbot.create(i, nodehandle, 0, 0, 1, 10)
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
		--if torch.abs(swarmbots[i].position[1]) > arena_width / 2 or torch.abs(swarmbots[i].position[2]) > arena_width / 2 then
			swarmbots[i]:update_energy(-1/frequency)
		--end
	end

	ros.spinOnce()
	ros.Duration(1/frequency):sleep()
end
