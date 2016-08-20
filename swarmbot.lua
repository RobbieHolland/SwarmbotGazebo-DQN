swarmbot = {}
swarmbot.__index = swarmbot

function swarmbot.create(id, nodehandle, HUs, twist_spec, image_spec, odom_spec, link_spec, model_state_spec, x, y, z)
	--Spawn swarmbot.sdf in gazebo
	os.execute('rosrun gazebo_ros spawn_model -x ' .. x .. ' -y ' .. y .. ' -z ' .. z .. ' -file `rospack find swarm_simulator`/sdf/test_model.sdf' .. 
							 ' -sdf -model swarmbot' .. id .. ' -robot_namespace swarmbot' .. id)	
-- .. ' > /dev/null 2>&1'
	--Create object
  local sbot = {}
  setmetatable(sbot,swarmbot)

	--Assign variables
	sbot.id = id
	sbot.energy = 0
	sbot.nodehandle = nodehandle
	sbot.position = torch.Tensor(3):zero()
	sbot.model_name = 'swarmbot' .. sbot.id
	number_of_colour_channels = 3;
	number_of_cameras = 2;
	number_of_colour_sensors_per_camera = 15;
	number_of_output_channels = 2;
	sbot.current_input = torch.Tensor(number_of_colour_sensors_per_camera * number_of_cameras)
	sbot.max_drive = 0.0
	sbot.max_turn = 0.0


	--Create swarmbot's neural net
	sbot.neural_net = nn.Sequential();
	sbot.neural_net:add(nn.Linear(number_of_colour_sensors_per_camera * number_of_cameras, number_of_output_channels, true))
	--sbot.neural_net:add(nn.Linear(number_of_colour_sensors, HUs, true))
	--sbot.neural_net:add(nn.Tanh())
	--sbot.neural_net:add(nn.Linear(HUs, number_of_output_channels, true))

	--Connect publisher to topic that swarmbot subscribes to
  sbot.publisher = sbot.nodehandle:advertise("/swarmbot" .. id .. "/cmd_vel", twist_spec, 100, false, connect_cb, disconnect_cb)
	sbot.relocation_publisher = sbot.nodehandle:advertise("/gazebo/set_model_state", model_state_spec, 100, false, connect_cb, disconnect_cb)
	--Connect subscribers to topics that swarmbot publishes to
	sbot.input_subscribers = {}

	--Configure sensors
	for i=1, number_of_cameras do
		sbot.input_subscribers[i] 
				= sbot.nodehandle:subscribe("/swarmbot" .. id .. "/front_colour_sensor_" .. i .. "/image_raw", image_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })

	  sbot.input_subscribers[i]:registerCallback(function(msg, header)
			--input is taken from msg published by swarmbot
			sensor_input = torch.reshape(msg.data,msg.height*number_of_colour_channels*msg.width)
			--print(sensor_input)
			--Is there a way for torch.reshape to return a DoubleTensor?
			sensor_input = (1/255) * sensor_input:double()
			
			--Green channel
			for j=1, number_of_colour_sensors_per_camera do
				sbot.current_input[j + (i - 1) * number_of_colour_sensors_per_camera] = sensor_input[2 + number_of_colour_channels * (j - 1)]
			end
			
			--robot is commanded to move every time it receives input
			sbot:move()
  	end)
	end
	
  sbot.odom_subscriber = sbot.nodehandle:subscribe("/swarmbot" .. id .. "/base_pose", odom_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
	--Create command message
	sbot.command_message = ros.Message(twist_spec)
	sbot.relocation_message = ros.Message(model_state_spec)

  sbot.odom_subscriber:registerCallback(function(msg, header)
		--position published by robot
		sbot.position[1] = msg.pose.pose.position.x
		sbot.position[2] = msg.pose.pose.position.y
		sbot.position[3] = msg.pose.pose.position.z

  end)

  return sbot
end

function swarmbot.move(self)
		--output is result of forwarding real world input through neural network
		output = self.neural_net:forward(self.current_input)

	--Following are attempts to fix physics bugs, and are not in the spirit of this project
--[[
		output[1] = math.abs(output[1])
		if output[1] > self.max_drive then
			output[1] = self.max_drive
		end
		if output[2] > self.max_turn then
			output[2] = self.max_turn
		end
		if output[2] < -self.max_turn then
			output[2] = -self.max_turn
		end

		a = output[1] .. '0'
		b = output[2] .. '0'
print(a)
print(b)
--]]
		if output[1] ~= output[1] then
			output[1] = 0
			print(self.current_input)
		end
		if output[2] ~= output[2] then
			output[2] = 0
			print(self.current_input)
		end

		self.command_message.linear.x = output[1];
		self.command_message.angular.z = output[2];


		self.publisher:publish(self.command_message)
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

function swarmbot.update_neural_net(self, new_neural_net)
	self.neural_net = new_neural_net
end

function swarmbot.consume(self, food)
	self.energy = self.energy + food.value
end

--Returns the fittest of two swarmbots
function swarmbot.fittest(sbot1, sbot2)
	return sbot1.energy > sbot2.energy
end
