ros = require 'ros'
ros.init('GazeboDQN_de')


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

a = 0
b = 0
c = 0
d = 0
e = 0
updated = false

model_estate_subscriber = nodehandle:subscribe("/swarmbot1/cmd_vel", msgs.twist_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
model_estate_subscriber:registerCallback(function(msg, header)
	if msg.angular.z > 0 then
		a = a + 1
	end
	if msg.angular.z < 0 then
		b = b + 1
	end
	if msg.angular.z == 0 then
		c = c + 1
	end
	d = d + 1
updated = true
end)

while not ros.isShuttingDown() do
	ros.spinOnce()
	if updated then
	print('a: ' .. a .. ', b: ' .. b .. ', c: ' .. c)
	end
	updated = false
end
