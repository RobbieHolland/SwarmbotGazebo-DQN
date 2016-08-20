ros = require 'ros'
msgs = require 'async/SwarmbotGazebo-DQN/msgs'
require 'async/SwarmbotGazebo-DQN/food'

--Constants
arena_width = 20

--setup ros node and spinner (processes queued send and receive topics)
ros.init('GazeboDQN_rewards')
spinner = ros.AsyncSpinner()
nodehandle = ros.NodeHandle()

number_of_food = 10
--Create food
foods = {}
for i=1, number_of_food do
	--Create new food
	foods[i] = food.create(i, nodehandle, 0, 0, 1, 10)
	foods[i]:random_relocate(arena_width)
end
