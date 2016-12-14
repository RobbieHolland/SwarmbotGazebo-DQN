ros = require 'ros'
require 'torch'
msgs = require 'ROS_specifications/msgs'
local gnuplot = require 'gnuplot'

agent_id = tonumber(arg[1])
energy = 0
received = false

--setup ros node and spinner (processes queued send and receive topics)
ros.init('GazeboDQN_statistics' .. agent_id)
spinner = ros.AsyncSpinner()
nodehandle = ros.NodeHandle()
print('Looking at action distribution for agent ' .. agent_id .. ' ...')

--Configure subscriber to receive robots current energy
chosen_message = 0
command_subscriber = nodehandle:subscribe("/swarmbot" .. agent_id .. "/cmd_vel", msgs.twist_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
command_subscriber:registerCallback(function(msg, header)
		chosen_message = msg.angular.z

		received = true
end)

--For now hard coded to only one agent
epoch_count = 0
sample_size = 200
chosen_commands = torch.Tensor(sample_size):zero()
while not ros.isShuttingDown() and epoch_count < sample_size do
	epoch_count = epoch_count + 1

	while not received do
		ros.spinOnce()
	end
	received = false

	chosen_commands[epoch_count] = chosen_message

end

gnuplot.pngfigure('SwarmbotGazebo-DQN/Experiments/Action_Distribution_Tests/agent_ ' .. agent_id .. '_chosen_commands.png')
gnuplot.hist(chosen_commands,3)
gnuplot.plotflush()
