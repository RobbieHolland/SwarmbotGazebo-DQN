print('Handling command buffering...')

--ROS connections
ros = require 'ros'
ros.init('command_buffer')
resp_ready = false

--Imports
msgs = require 'msgs'
util = require 'swarm_util'
buffer = require 'buffers/buffer'

--Setup ros node and spinner (processes queued send and receive topics)
spinner = ros.AsyncSpinner()
nodehandle = ros.NodeHandle()

--Variables
number_of_bots = tonumber(arg[1])
number_of_pred = tonumber(arg[2])

--Creating subscribers and publishers for buffering
swarmbot_buffer = buffer.create('swarmbot', number_of_bots)
predator_buffer = buffer.create('predator', number_of_pred)

while not ros.isShuttingDown() do
	--Keep spinning until all messages have been received
	while not (swarmbot_buffer:check_commands_received() and predator_buffer:check_commands_received()) do
		--Check again
		ros.spinOnce()
	end

	--Once all network comamnds have been received send them all of to
  --virtual robots simulatenously
  swarmbot_buffer:publish_commands()
  predator_buffer:publish_commands()

	--Publish to networks that their commands have been sent
  swarmbot_buffer:notify_agents_commands_sent();
  predator_buffer:notify_agents_commands_sent();
end
