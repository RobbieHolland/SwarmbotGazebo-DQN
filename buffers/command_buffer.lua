print('Handling command buffering...')

--ROS connections
ros = require 'ros'
ros.init('command_buffer')
msgs = require 'msgs'
util = require 'swarm_util'
buffer = require 'buffers/buffer'
resp_ready = false

--Setup ros node and spinner (processes queued send and receive topics)
spinner = ros.AsyncSpinner()
nodehandle = ros.NodeHandle()

function connect_cb(name, topic)
  print("subscriber connected: " .. name .. " (topic: '" .. topic .. "')")
end

function disconnect_cb(name, topic)
  print("subscriber diconnected: " .. name .. " (topic: '" .. topic .. "')")
end

--Variables
number_of_bots = tonumber(arg[1])
number_of_pred = tonumber(arg[2])

--Creating subscribers and publishers for buffering
swarmbot_buffer = buffer.create('swarmbot', number_of_bots)
predator_buffer = buffer.create('predator', number_of_pred)

--Publish to topic that signifies if messages have been sent to robots
commands_sent_msg = ros.Message(msgs.bool_spec)

function publish_commands(publishers, commands, received)
  if #publishers > 0 then
    publishers[0]:publish(commands[0])
  end

  for i=1, #publishers do
		publishers[i]:publish(commands[i])
		received[i] = false
	end
end

function notify_agents_commands_sent(publishers, number_of_agents)
  --Publish that commands have been sent
	commands_sent_msg.data = true

  --Must notify validation agent that commands have been sent so that it
  --is synchronised with other robots
  --However the other robots should not wait for validation agent to give
  --commands since it becomes dormant between validation sessions
  if number_of_agents > 0 then
    publishers[0]:publish(commands_sent_msg)
  end

	for i=1, #publishers do
		publishers[i]:publish(commands_sent_msg)
	end
end

while not ros.isShuttingDown() do
	--Keep spinning until all messages have been received
	while not (util.check_received(swarmbot_buffer.network_commands_received, #swarmbot_buffer.network_commands_received) and
             util.check_received(predator_buffer.network_commands_received, #predator_buffer.network_commands_received)) do
		--Check again
		ros.spinOnce()
	end

	--Send off all commands to robots
  publish_commands(swarmbot_buffer.command_publishers, swarmbot_buffer.commands, swarmbot_buffer.network_commands_received)
  publish_commands(predator_buffer.command_publishers, predator_buffer.commands, predator_buffer.network_commands_received)

	--Publish that commands have been sent
	notify_agents_commands_sent(swarmbot_buffer.command_sent_publishers, number_of_bots);
  notify_agents_commands_sent(predator_buffer.command_sent_publishers, number_of_pred);
end
