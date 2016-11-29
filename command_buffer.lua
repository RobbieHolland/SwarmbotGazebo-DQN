--ROS connections
ros = require 'ros'
ros.init('command_buffer')
msgs = require 'msgs'
util = require 'swarm_util'
resp_ready = false

print('Handling command buffering...')

function connect_cb(name, topic)
  print("subscriber connected: " .. name .. " (topic: '" .. topic .. "')")
end

function disconnect_cb(name, topic)
  print("subscriber diconnected: " .. name .. " (topic: '" .. topic .. "')")
end

--Variables
number_of_bots = arg[1]
number_of_pred = arg[2]
total_number_agents = number_of_bots + number_of_pred

--Setup ros node and spinner (processes queued send and receive topics)
spinner = ros.AsyncSpinner()
nodehandle = ros.NodeHandle()

local function append(table, element)
	table[#table + 1] = element
	return table
end

--Subscribe to command topics advertised by environments
local function subsc_publish(type_agent, command_subscribers, command_publishers, n)
	for i=1, n do
    --Where to look for commands
		command_subscribers[i]
      = nodehandle:subscribe("/" .. type_agent .. i .. "/network_command", msgs.twist_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
    --Where to send commands
    command_publishers[i]
      = nodehandle:advertise("/" .. type_agent .. i .. "/cmd_vel", msgs.twist_spec, 100, false, connect_cb, disconnect_cb)

		command_subscribers[i]:registerCallback(function(msg, header)
			--Extract command action chosen by network
			bots_commands[i] = msg
			bots_received[i] = true
		end)
	end
end

--Creating subscribers and publishers for buffering
command_publishers = { -- Different initial value to account for one more swarmbot
	swarmbot = {},
  predator = {}
}
command_subscribers = {}
bots_commands = {}
subsc_publish('swarmbot', command_subscribers, command_publishers.swarmbot, number_of_bots-1)
--subsc_publish('predator', command_subscribers, command_publishers, number_of_pred)

bots_received = {}
for i=1, number_of_bots-1 do
	bots_received[i] = false
end

preds_received = {}
for i=1, number_of_pred-1 do
	preds_received[i] = false
end

--Publish to topic that signifies if messages have been sent to robots
commands_sent_msg = ros.Message(msgs.bool_spec)
command_sent_publishers = {}
for i=0, number_of_bots-1 do
	command_sent_publishers[i] = nodehandle:advertise("/swarmbot" .. i .. "/commands_sent", msgs.bool_spec, 100, false, connect_cb, disconnect_cb)
end

while not ros.isShuttingDown() do
	--Keep spinning until all messages have been received
	while not (util.check_received(bots_received, #bots_received) and
             util.check_received(preds_received, #preds_received)) do
		--Check again
		ros.spinOnce()
		commands_sent_msg.data = false
	end

	--Send off all commands to robots
	for i=1, #command_publishers.swarmbot do
		command_publishers.swarmbot[i]:publish(bots_commands[i])
		bots_received[i] = false
	end

	--Publish that commands have been sent
	commands_sent_msg.data = true
	for i=0, #command_publishers.swarmbot do
		command_sent_publishers[i]:publish(commands_sent_msg)
	end

end
