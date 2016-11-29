print('Handling command buffering...')

--ROS connections
ros = require 'ros'
ros.init('command_buffer')
msgs = require 'msgs'
util = require 'swarm_util'
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
number_of_bots = arg[1]
number_of_pred = arg[2]

--Subscribe to command topics advertised by environments
local function subsc_publish(type_agent, command_publishers, commands, network_commands_received, n)
	for i=1, n-1 do
    --Where to look for commands
		command_subscriber
      = nodehandle:subscribe("/" .. type_agent .. i .. "/network_command", msgs.twist_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
    --Where to send commands
    command_publishers[i]
      = nodehandle:advertise("/" .. type_agent .. i .. "/cmd_vel", msgs.twist_spec, 100, false, connect_cb, disconnect_cb)

		command_subscriber:registerCallback(function(msg, header)
			--Extract command action chosen by network
			commands[i] = msg
			network_commands_received[i] = true
		end)
	end
end

--Creating subscribers and publishers for buffering
swarmbot_buffer = {
  command_publishers = {},
  command_sent_publishers = {},
  commands = {},
  network_commands_received = {}
}
predator_buffer = {
  command_publishers = {},
  command_sent_publishers = {},
  commands = {},
  network_commands_received = {}
}

swarmbot_buffer.network_commands_received = {}
for i=1, number_of_bots-1 do
	swarmbot_buffer.network_commands_received[i] = false
end

predator_buffer.network_commands_received = {}
for i=1, number_of_pred-1 do
	predator_buffer.network_commands_received[i] = false
end

subsc_publish('swarmbot', swarmbot_buffer.command_publishers, swarmbot_buffer.commands, swarmbot_buffer.network_commands_received, number_of_bots)
subsc_publish('predator', predator_buffer.command_publishers, predator_buffer.commands, predator_buffer.network_commands_received, number_of_pred)

--Publish to topic that signifies if messages have been sent to robots
function create_command_sent_publishers(type_agent, command_sent_publishers, n)
  commands_sent_msg = ros.Message(msgs.bool_spec)
  for i=0, n-1 do
  	command_sent_publishers[i] = nodehandle:advertise("/swarmbot" .. i .. "/commands_sent", msgs.bool_spec, 100, false, connect_cb, disconnect_cb)
  end
end
create_command_sent_publishers('swarmbot', swarmbot_buffer.command_sent_publishers, number_of_bots)
create_command_sent_publishers('predator', predator_buffer.command_sent_publishers, number_of_pred)

----------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------

function publish_commands(publishers, commands, received)
  for i=1, #publishers do
		publishers[i]:publish(commands[i])
		received[i] = false
	end
end

function notify_agents_commands_sent(publishers)
  --Publish that commands have been sent
	commands_sent_msg.data = true
	for i=0, #publishers do
		publishers[i]:publish(commands_sent_msg)
	end
end

while not ros.isShuttingDown() do
	--Keep spinning until all messages have been received
	while not (util.check_received(swarmbot_buffer.network_commands_received, #swarmbot_buffer.network_commands_received) and
             util.check_received(predator_buffer.network_commands_received, #predator_buffer.network_commands_received)) do
		--Check again
		ros.spinOnce()
		commands_sent_msg.data = false
	end

	--Send off all commands to robots
  publish_commands(swarmbot_buffer.command_publishers, swarmbot_buffer.commands, swarmbot_buffer.network_commands_received)
  publish_commands(predator_buffer.command_publishers, predator_buffer.commands, predator_buffer.network_commands_received)

	--Publish that commands have been sent
	notify_agents_commands_sent(swarmbot_buffer.command_sent_publishers);

end
