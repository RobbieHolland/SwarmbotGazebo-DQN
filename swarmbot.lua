swarmbot = {}
swarmbot.__index = swarmbot

function swarmbot.create(id, nodehandle, HUs, x, y, z)
	--Create object
  local sbot = {}
  setmetatable(sbot,swarmbot)

	--Assign variables
	sbot.id = id
	sbot.energy = 0
	sbot.nodehandle = nodehandle
	sbot.position = torch.Tensor(3):zero()
	sbot.model_name = 'swarmbot' .. sbot.id
	sbot.max_drive = 1.0
	sbot.max_turn = 1.0

	--Spawn swarmbot.sdf in gazebo
	os.execute('rosrun gazebo_ros spawn_model -x ' .. sbot.position[1] .. ' -y ' .. sbot.position[2] .. ' -z ' .. sbot.position[3] .. ' -file `rospack find 		      		          swarm_simulator`/sdf/test_model.sdf' .. ' -sdf -model swarmbot' .. sbot.id .. ' -robot_namespace swarmbot' .. sbot.id)
	--To suppress output: .. ' > /dev/null 2>&1'

	--Publisher to publish position updates
	sbot.relocation_publisher = sbot.nodehandle:advertise("/gazebo/set_model_state", msgs.model_state_spec, 100, false, connect_cb, disconnect_cb)
	--Publisher to publish robot's current energy level
	sbot.energy_publisher = sbot.nodehandle:advertise("/swarmbot" .. sbot.id .. "/energy_level", msgs.float_spec, 100, false, connect_cb, disconnect_cb)
	--Publisher to receive position updates
	sbot.odom_subscriber = sbot.nodehandle:subscribe("/swarmbot" .. sbot.id .. "/base_pose", msgs.odom_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })

	--Create messages
	sbot.relocation_message = ros.Message(msgs.model_state_spec)
	sbot.energy_level_message = ros.Message(msgs.float_spec)

  sbot.odom_subscriber:registerCallback(function(msg, header)
		--Position published by robot is recorded
		sbot.position[1] = msg.pose.pose.position.x
		sbot.position[2] = msg.pose.pose.position.y
		sbot.position[3] = msg.pose.pose.position.z
  end)

  return sbot
end

function swarmbot.random_relocate(self, distance)
	new_position = distance * (torch.rand(3) - 0.5)
	new_position[3] = 0
	self:relocate(new_position)
end

function swarmbot.relocate(self, new_position)
	m = self.relocation_message
	m.model_name = self.model_name
	m.pose.position.x = new_position[1]
	m.pose.position.y = new_position[2]
	m.pose.position.z = new_position[3]

	self.relocation_publisher:publish(m)
	self.position = new_position
end

function swarmbot.consume(self, food)
	self.energy = self.energy + food.value
end

function swarmbot:publish_energy()
	--Publish swarmbot's energy for environment
	self.energy_level_message.data = self.energy
	self.energy_publisher:publish(self.energy_level_message)
end
