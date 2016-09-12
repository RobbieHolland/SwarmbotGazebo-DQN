ros = require 'ros'
ros.init('GazeboDQN_d')


require 'torch'
srvs = require 'async/SwarmbotGazebo-DQN/srvs'
msgs = require 'async/SwarmbotGazebo-DQN/msgs'
require 'async/SwarmbotGazebo-DQN/food'
require 'async/SwarmbotGazebo-DQN/swarmbot'
resp_ready = false

function connect_cb(name, topic)
  print("subscriber connected: " .. name .. " (topic: '" .. topic .. "')")
end

function disconnect_cb(name, topic)
  print("subscriber diconnected: " .. name .. " (topic: '" .. topic .. "')")
end

--Flags
initialised = false
updated = false

function connect_cb(name, topic)
  print("subscriber connected: " .. name .. " (topic: '" .. topic .. "')")
end

function disconnect_cb(name, topic)
  print("subscriber diconnected: " .. name .. " (topic: '" .. topic .. "')")
end

function table_invert(t)
   local s={}
   for k,v in pairs(t) do
     s[v]=k
   end
   return s
end

spinner = ros.AsyncSpinner()
nodehandle = ros.NodeHandle()

--Updates velocities of models
model_state_subscriber = nodehandle:subscribe("/gazebo/model_states", msgs.model_states_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
model_state_subscriber:registerCallback(function(msg, header)
	velocity = torch.Tensor(3):zero()

	velocity[1] = msg.twist[4].values.angular.x
	velocity[2] = msg.twist[4].values.angular.y
	velocity[3] = msg.twist[4].values.angular.z

	print(velocity)
end)

while not ros.isShuttingDown() do
	ros.spinOnce()
end
