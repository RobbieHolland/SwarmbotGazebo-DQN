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

spinner = ros.AsyncSpinner()
nodehandle = ros.NodeHandle()

sv = nodehandle:serviceClient('/demo_service', srvs.energy_request_spec)
x = sv:createRequest()
x.id = 4
response = sv:call(x)
print(response.data)

while not ros.isShuttingDown() do

	
	ros.spinOnce()
	ros.Duration(1/1):sleep()
end
