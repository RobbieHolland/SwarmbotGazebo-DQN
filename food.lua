food = {}
food.__index = food

function food.create(id, nodehandle, x, y, z, value)
	--Spawn food in gazebo
	os.execute('rosrun gazebo_ros spawn_model -x ' .. x .. ' -y ' .. y .. ' -z ' .. z .. ' -file `rospack find swarm_simulator`/sdf/food.sdf' .. 
						 ' -sdf -model food' .. id .. ' -robot_namespace food' .. id)	
	-- .. ' > /dev/null 2>&1'

  local fd = {}
  setmetatable(fd,food)
	fd.id = id
	fd.nodehandle = nodehandle
	fd.value = value
	fd.position = torch.Tensor(3):zero()
	fd.position[1] = x
	fd.position[2] = y
	fd.position[3] = z
	fd.relocation_message = ros.Message(msgs.model_state_spec)

	--Connect subscribers to topics that food publishes to
  fd.odom_subscriber = fd.nodehandle:subscribe("/food" .. id .. "/base_pose", msgs.odom_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
	--Connect publishers to topics that food subscribes to
	fd.relocation_publisher = fd.nodehandle:advertise("/gazebo/set_model_state", msgs.model_state_spec, 100, false, connect_cb, disconnect_cb)

  fd.odom_subscriber:registerCallback(function(msg, header)
		--position published by robot
		fd.position[1] = msg.pose.pose.position.x
		fd.position[2] = msg.pose.pose.position.y
		fd.position[3] = msg.pose.pose.position.z
  end)

  return fd
end

function food.random_relocate(self, distance)
	new_position = distance * (torch.rand(3) - 0.5)
	new_position[3] = 1
	self:relocate(new_position)
end

function food.relocate(self, new_position)
	--os.execute('rosservice call /gazebo/delete_model "model_name: \'food' .. self.id .. '\'" ')	
	m = self.relocation_message
	m.model_name = 'food' .. self.id
	m.pose.position.x = new_position[1]
	m.pose.position.y = new_position[2]
	m.pose.position.z = new_position[3]
	self.relocation_publisher:publish(m)

	--Spin required here, food position in Gazebo was not updating quickly enough so could be consumed multiple times
	ros.spinOnce()
	self.position = new_position
end
