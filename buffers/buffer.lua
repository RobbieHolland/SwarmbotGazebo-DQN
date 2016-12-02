buffer = {}
buffer.__index = buffer

util = require 'swarm_util'

function buffer.create(type_agent, n)
  local bffr = {}
  setmetatable(bffr, buffer)

  bffr.type_agent = type_agent
  bffr.n = n
  bffr.commands_sent_msg = ros.Message(msgs.bool_spec)

  bffr.command_publishers = {}
  bffr.command_sent_publishers = {}
  bffr.commands = {}
  bffr.network_commands_received = {}

  for i=1, n-1 do
  	bffr.network_commands_received[i] = false
  end

  bffr:subsc_publish()
  bffr:create_command_sent_publishers()

  return bffr
end

--Subscribe to command topics advertised by environments
function buffer:subsc_publish()
	for i=0, self.n-1 do
    --Where to look for commands
		command_subscriber
      = nodehandle:subscribe("/" .. self.type_agent .. i .. "/network_command", msgs.twist_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
    --Where to send commands
    self.command_publishers[i]
      = nodehandle:advertise("/" .. self.type_agent .. i .. "/cmd_vel", msgs.twist_spec, 100, false, connect_cb, disconnect_cb)

    self.commands[i] = ros.Message(msgs.twist_spec)
		command_subscriber:registerCallback(function(msg, header)
			--Extract command action chosen by network
			self.commands[i] = msg
			self.network_commands_received[i] = true
		end)
	end
end

--Publishes to topic that signifies if messages have been sent to robots
function buffer:create_command_sent_publishers()
  for i=0, self.n-1 do
  	self.command_sent_publishers[i] = nodehandle:advertise("/swarmbot" .. i .. "/commands_sent", msgs.bool_spec, 100, false, connect_cb, disconnect_cb)
  end
end

--Check if all robots using buffer have sent their commands_sent
function buffer:check_commands_received()
  return util.check_received(self.network_commands_received, #self.network_commands_received)
end

--Send off all commands received from networks simulatenously to physical robots
function buffer:publish_commands()
  if #self.command_publishers > 0 then
    self.command_publishers[0]:publish(self.commands[0])
  end

  for i=1, #self.command_publishers do
		self.command_publishers[i]:publish(self.commands[i])
		self.network_commands_received[i] = false
	end
end

--Publish to topic that signifies if messages have been sent to robots
function buffer:notify_agents_commands_sent()
  --Publish that commands have been sent
	self.commands_sent_msg.data = true

  --Must notify validation agent that commands have been sent so that it
  --is synchronised with other robots
  --However the other robots should not wait for validation agent to give
  --commands since it becomes dormant between validation sessions
  if self.n > 0 then
    self.command_sent_publishers[0]:publish(self.commands_sent_msg)
  end

	for i=1, #self.command_publishers do
		self.command_sent_publishers[i]:publish(self.commands_sent_msg)
	end
end

return buffer
