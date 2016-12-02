--Setup
ros = require 'ros'
ros.init('GazeboDQN_positions')
print('Handling Positions...')

require 'torch'
msgs = require 'msgs'
require 'swarm_util'

--setup ros node and spinner (processes queued send and receive topics)
spinner = ros.AsyncSpinner()
nodehandle = ros.NodeHandle()

--Publish model states
position_publisher
		= nodehandle:advertise("/throttled_model_states", msgs.model_states_spec, 100, false, connect_cb, disconnect_cb)

latest_message = msgs.model_states_spec

--Updates model states
model_state_subscriber = nodehandle:subscribe("/gazebo/model_states", msgs.model_states_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
model_state_subscriber:registerCallback(function(msg, header)
	initialised = true
	latest_message = msg
end)

frequency = 2
while ros.ok() do
	ros.spinOnce()
	if initialised then position_publisher:publish(latest_message) end

	ros.Duration(1/frequency):sleep()
end

ros.shutdown()
