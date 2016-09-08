food = {}
food.__index = food

function connect_cb(name, topic)
  --print("subscriber connected: " .. name .. " (topic: '" .. topic .. "')")
end

function disconnect_cb(name, topic)
  print("subscriber diconnected: " .. name .. " (topic: '" .. topic .. "')")
end

function food.create(id, nodehandle, x, y, z, value)
  local fd = {}
  setmetatable(fd,food)
	fd.id = id
	fd.model_name = 'food' .. fd.id
	fd.nodehandle = nodehandle
	fd.value = value
	fd.position = torch.Tensor(3):zero()
	fd.position[1] = x
	fd.position[2] = y
	fd.position[3] = z
	fd.relocation_message = ros.Message(msgs.model_state_spec)

	--Publisher to publish position updates
	fd.relocation_publisher = fd.nodehandle:advertise("/gazebo/set_model_state", msgs.model_state_spec, 100, false, connect_cb, disconnect_cb)

  return fd
end

--Calculates a new random position for the object within a square boundary
function food.random_relocate(self, distance)
	new_position = distance * (torch.rand(3) - 0.5)
	--Set spawn height to 1
	new_position[3] = 1
	self:relocate(new_position)
end

--Relocates the object to a new position
function food:relocate(new_position)
	m = self.relocation_message
	m.model_name = self.model_name
	m.pose.position.x = new_position[1]
	m.pose.position.y = new_position[2]
	m.pose.position.z = new_position[3]
	self.relocation_publisher:publish(m)

	--Spin required here, food position in Gazebo was not updating quickly enough so could be consumed multiple times
	ros.spinOnce()
	self.position = new_position
end
