--ROS conenctions
ros = require 'ros'
ros.init('command_buffer')
msgs = require 'async/SwarmbotGazebo-DQN/msgs'
srvs = require 'async/SwarmbotGazebo-DQN/srvs'
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
reward_calculation_time = tonumber(arg[2])

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

--Services to play and pause simulation
play_physics_service = nodehandle:serviceClient('/gazebo/unpause_physics', srvs.empty_spec)
play_physics_message = play_physics_service:createRequest()

pause_physics_service = nodehandle:serviceClient('/gazebo/pause_physics', srvs.empty_spec)
pause_physics_message = pause_physics_service:createRequest()

current_time = 0
clock_subscriber = nodehandle:subscribe("/clock", msgs.clock_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
clock_subscriber:registerCallback(function(msg, header)
	current_time = msg.clock:toSec()
end)

--Publish to topic that signifies if messages have been sent to robots
commands_sent_msg = ros.Message(msgs.bool_spec)
time_stepped_msg = ros.Message(msgs.bool_spec)
command_sent_publishers = {}
time_step_publishers = {}
for i=0, number_of_bots do
	command_sent_publishers[i] = nodehandle:advertise("/commands_sent" .. i, msgs.bool_spec, 100, false, connect_cb, disconnect_cb)
	time_step_publishers[i] = nodehandle:advertise("/time_stepped" .. i, msgs.bool_spec, 100, false, connect_cb, disconnect_cb)
end


function check_received(bots_received)
	for i=1, number_of_bots do
		if not bots_received[i] then
			return false
		end
	end
	return true
end

while not ros.isShuttingDown() do
	--Keep spinning until all messages have been received
	time_stepped_msg.data = true
	while not check_received(bots_received) do
		--Check again
		ros.spinOnce()

		--Send that simulation has been paused
		for i=0, number_of_bots do
			time_step_publishers[i]:publish(time_stepped_msg)
		end
	end

	--Send off all commands to robots
	for i=1, number_of_bots do
		command_publishers[i]:publish(bots_commands[i])
		bots_received[i] = false
	end

	--Play simulation
	play_physics_service:call(play_physics_message)

	--Publish that commands have been sent
	commands_sent_msg.data = true
	for i=0, number_of_bots do
		command_sent_publishers[i]:publish(commands_sent_msg)
	end

	--Wait some time for rewards to occur
	start_time = current_time
	while current_time - start_time < reward_calculation_time do
		ros.spinOnce()
	end

	--Pause simulation
	pause_physics_service:call(pause_physics_message)
end
