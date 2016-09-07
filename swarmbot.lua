swarmbot = {}
swarmbot.__index = swarmbot

function swarmbot.create(id, nodehandle, frequency, x, y, z)
	--Create object
  local sbot = {}
  setmetatable(sbot,swarmbot)

	--Assign variables
	sbot.id = id
	sbot.speed_limit = 1.5
	sbot.energy = 0
	sbot.nodehandle = nodehandle
	sbot.position = torch.Tensor(3):zero()
	sbot.average_velocity = torch.Tensor(3):zero()
	sbot.model_name = 'swarmbot' .. sbot.id
	sbot.max_drive = 1.0
	sbot.max_turn = 1.0
	sbot.frequency = frequency

	--Spawn swarmbot.sdf in gazebo
	--os.execute('rosrun gazebo_ros spawn_model -x ' .. sbot.position[1] .. ' -y ' .. sbot.position[2] .. ' -z ' .. sbot.position[3] .. ' -file `rospack find 		      	--	          swarm_simulator`/sdf/swarm_robot_v2.sdf' .. ' -sdf -model swarmbot' .. sbot.id .. ' -robot_namespace swarmbot' .. sbot.id)
	--To suppress output: .. ' > /dev/null 2>&1'

	--Publisher to publish position updates
	sbot.relocation_publisher = sbot.nodehandle:advertise("/gazebo/set_model_state", msgs.model_state_spec, 100, false, connect_cb, disconnect_cb)
	--Publisher to publish robot's current energy level
	sbot.energy_publisher = sbot.nodehandle:advertise("/swarmbot" .. sbot.id .. "/energy_level", msgs.float_spec, 100, false, connect_cb, disconnect_cb)
	--Publisher to receive position updatess
	sbot.odom_subscriber 
		= sbot.nodehandle:subscribe("/swarmbot" .. sbot.id .. "/base_pose", msgs.odom_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
	--Publisher to indicate if over speed limit
	sbot.speed_limit_publisher 
		= sbot.nodehandle:advertise("/swarmbot" .. sbot.id .. "/speed_limit_indicator", msgs.bool_spec, 100, false, connect_cb, disconnect_cb)

	--Create messages
	sbot.relocation_message = ros.Message(msgs.model_state_spec)
	sbot.energy_level_message = ros.Message(msgs.float_spec)
	sbot.speed_limit_message = ros.Message(msgs.bool_spec)

  sbot.odom_subscriber:registerCallback(function(msg, header)
		--Save old position
		old_position = sbot.position:clone()

		--Position published by robot is recorded
		sbot.position[1] = msg.pose.pose.position.x
		sbot.position[2] = msg.pose.pose.position.y
		sbot.position[3] = msg.pose.pose.position.z

		--Calculate average_velocity
		sbot.average_velocity = frequency * (sbot.position - old_position)
		sbot.speed_limit_message.data = sbot.average_velocity:norm() > sbot.speed_limit
		sbot.speed_limit_publisher:publish(sbot.speed_limit_message)
  end)

  return sbot
end

function swarmbot.random_relocate(self, distance)
	new_position = distance * (torch.rand(3) - 0.5)
	new_position[3] = 0
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
	self:update_energy(food.value)
end

function swarmbot:update_energy(value)
	self.energy = self.energy + value
	self:publish_energy()
end

function swarmbot:publish_energy()
	--Publish swarmbot's energy for environment
	self.energy_level_message.data = self.energy
	self.energy_publisher:publish(self.energy_level_message)
end
