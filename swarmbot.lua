swarmbot = {}
swarmbot.__index = swarmbot

function swarmbot.create(id, nodehandle, x, y, z)
	--Create object
  local sbot = {}
  setmetatable(sbot,swarmbot)

	--Assign variables
	sbot.id = id
	sbot.model_id = 0
	sbot.energy = 0
	sbot.nodehandle = nodehandle
	sbot.position = torch.Tensor(3):zero()
	sbot.speed = 1
	sbot.previous_speed = 1
	sbot.velocity = torch.Tensor(3):zero()
	sbot.model_name = 'swarmbot' .. sbot.id
	sbot.position_updated = false

	--Publisher to publish position updates
	sbot.relocation_publisher = sbot.nodehandle:advertise("/gazebo/set_model_state", msgs.model_state_spec, 100, false, connect_cb, disconnect_cb)

	--Create messages
	sbot.relocation_message = ros.Message(msgs.model_state_spec)

  return sbot
end

function swarmbot.random_relocate(self, distance)
	new_position = distance * (torch.rand(3) - 0.5)
	new_position[3] = 1
	
	--To avoid spawning inside walls
	if torch.abs(new_position[1]) < 1 then
		new_position[1] = new_position[1] + 2
	end
	if torch.abs(new_position[2]) < 1 then
		new_position[2] = new_position[1] + 2
	end

	new_orientation = 2 * (torch.rand(1) - 0.5)
	self:relocate(new_position, new_orientation)
end

function swarmbot:upright()
	self:relocate(self.position)
end

function swarmbot.relocate(self, new_position, new_orientation)
	m = self.relocation_message
	m.model_name = self.model_name
	m.pose.position.x = new_position[1]
	m.pose.position.y = new_position[2]
	m.pose.position.z = new_position[3]

	--Only randomly rotate on z axis (but with Quaternions)
	m.pose.orientation.z = new_orientation[1]
	m.pose.orientation.w = 1
	
	self.relocation_publisher:publish(m)
	self.position = new_position
end

function swarmbot:consume(food)
	self:add_energy(food.value)
end

function swarmbot:add_energy(value)
	self.energy = self.energy + value
end
