--[[
Advertise a service from torch-ros.
The existing roscpp/GetLoggers srv file ist used.
]]

ros = require 'ros'

ros.init('advertiseService_demo')
nh = ros.NodeHandle()
srvs = require 'async/SwarmbotGazebo-DQN/srvs'
msgs = require 'async/SwarmbotGazebo-DQN/msgs'

service_queue = ros.CallbackQueue()
print(srvs.energy_request_spec)
function myServiceHandler(request, response, header)
	response.data = 3
	response.success = true
	print(request.id)
  return true
end

server = nh:advertiseService('/demo_service', srvs.energy_request_spec, myServiceHandler, service_queue)
print('name: ' .. server:getService())
print('service server running, call "rosservice call /demo_service" to send a request to the service.')

local s = ros.Duration(0.001)
while ros.ok() do
  s:sleep()
  if not service_queue:isEmpty() then
    print('[!] incoming service call')
    service_queue:callAvailable()
  end
  ros.spinOnce()
end

server:shutdown()
ros.shutdown()
