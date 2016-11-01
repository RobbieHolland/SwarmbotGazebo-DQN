ros = require 'ros'
require 'torch'
msgs = require 'msgs'
local gnuplot = require 'gnuplot'

update_time = tonumber(arg[1])
agent_id = tonumber(arg[2])
energy = 0

--setup ros node and spinner (processes queued send and receive topics)
ros.init('GazeboDQN_statistics' .. agent_id)
spinner = ros.AsyncSpinner()
nodehandle = ros.NodeHandle()
print('Handling Statistics for agent ' .. agent_id .. ' ...')

--Configure subscriber to receive robots current energy
energy_subscriber = nodehandle:subscribe("/swarmbot" .. agent_id .. "/energy_level", msgs.float_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
energy_subscriber:registerCallback(function(msg, header)
		--Update current energy level
		energy = msg.data
end)

--For now hard coded to only one agent
epoch_count = 0
energy_chart = {}
while not ros.isShuttingDown() do
	epoch_count = epoch_count + 1
	ros.Duration(update_time):sleep()
	--Save old energy levels
	old_energy = energy
	--Update to new energy levels
	ros.spinOnce()
	--Find the difference
	energy_change = energy - old_energy
	--Save difference
	energy_chart[epoch_count] = energy_change

	epoch_indices = torch.linspace(1, epoch_count, epoch_count)

  gnuplot.pngfigure(paths.concat('experiments', 'GazeboEnv', 'agent_ ' .. agent_id .. '_scores_.png'))
	gnuplot.plot('Energy', epoch_indices, torch.Tensor(energy_chart), '-')
	gnuplot.xlabel('Epoch')
	gnuplot.ylabel('Energy')
	gnuplot.movelegend('left', 'top')
	gnuplot.plotflush()
	
end

