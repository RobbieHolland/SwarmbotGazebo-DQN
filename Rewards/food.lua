food = {}
food.__index = food

require 'swarm_util'

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
	fd.immunity_duration = 0.03
	fd.immunity_start_time = 0
	fd.edible = true

	--Publisher to publish position updates
	fd.relocation_publisher = fd.nodehandle:advertise("/gazebo/set_model_state", msgs.model_state_spec, 100, false, connect_cb, disconnect_cb)

	--Configure service caller to relocate robot
	fd.relocation_service = fd.nodehandle:serviceClient('/gazebo/set_model_state', srvs.model_state_spec)
	fd.relocation_message = fd.relocation_service:createRequest()
	fd.relocation_message.model_state.model_name = fd.model_name

  return fd
end

--Calculates a new random position for the object within a square boundary
function food.random_relocate(self, distance)
	new_position = distance * (torch.rand(3) - 0.5)

	--To avoid spawning inside walls
	if torch.abs(new_position[1]) < 1 then
		new_position[1] = new_position[1] + 2
	end
	if torch.abs(new_position[2]) < 1 then
		new_position[2] = new_position[1] + 2
	end

	--Set spawn height to 1
	new_position[3] = 1
	self:relocate(new_position)
end

--Relocates the object to a new position
function food:relocate(new_position)
	m = self.relocation_message
	self.edible = false
	self.immunity_start_time = os.clock()

	m.model_state.pose.position.x = new_position[1]
	m.model_state.pose.position.y = new_position[2]
	m.model_state.pose.position.z = new_position[3]
	response = self.relocation_service:call(m)

	--Spin required here, food position in Gazebo was not updating quickly enough so could be consumed multiple times
	ros.spinOnce()
	self.position = new_position
end
