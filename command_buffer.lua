--ROS conenctions
ros = require 'ros'
ros.init('command_buffer')
msgs = require 'msgs'
util = require 'swarm_util'
resp_ready = false

function connect_cb(name, topic)
  --print("subscriber connected: " .. name .. " (topic: '" .. topic .. "')")
end

function disconnect_cb(name, topic)
  print("subscriber diconnected: " .. name .. " (topic: '" .. topic .. "')")
end

--Variables
print('Handling command buffering...')
number_of_bots = arg[1]
bots_received = {}
for i=1, number_of_bots do
	bots_received[i] = false
end
bots_commands = {}
command_subscribers = {}
command_publishers = {}

--Setup ros node and spinner (processes queued send and receive topics)
spinner = ros.AsyncSpinner()
nodehandle = ros.NodeHandle()

--Subscribe to command topics advertised by environments
for i=1, number_of_bots do
	command_subscribers[i]
				= nodehandle:subscribe("/swarmbot" .. i .. "/network_command", msgs.twist_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
	command_publishers[i] = nodehandle:advertise("/swarmbot" .. i .. "/cmd_vel", msgs.twist_spec, 100, false, connect_cb, disconnect_cb)

	command_subscribers[i]:registerCallback(function(msg, header)
		--Extract command action
		bots_commands[i] = msg
		bots_received[i] = true
	end)
end


--Publish to topic that signifies if messages have been sent to robots
commands_sent_msg = ros.Message(msgs.bool_spec)
command_sent_publishers = {}
for i=0, number_of_bots do
	command_sent_publishers[i] = nodehandle:advertise("/commands_sent" .. i, msgs.bool_spec, 100, false, connect_cb, disconnect_cb)
end


while not ros.isShuttingDown() do
	--Keep spinning until all messages have been received
	while not util.check_received(bots_received, number_of_bots) do
		--Check again
		ros.spinOnce()
		commands_sent_msg.data = false
	end

	--Send off all commands to robots
	for i=1, number_of_bots do
		command_publishers[i]:publish(bots_commands[i])
		bots_received[i] = false
	end

	--Publish that commands have been sent
	commands_sent_msg.data = true
	for i=0, number_of_bots do
		command_sent_publishers[i]:publish(commands_sent_msg)
	end
	
end



--Send message to topic signifying that commands have been sent
