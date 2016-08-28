--Initialise
ros = require 'ros'
ros.init('GazeboDQN_Env')
local classic = require 'classic'
msgs = require 'async/SwarmbotGazebo-DQN/msgs'
local GazeboEnv, super = classic.class('GazeboEnv', Env)
resp_ready = false

function connect_cb(name, topic)
  --print("subscriber connected: " .. name .. " (topic: '" .. topic .. "')")
end


function disconnect_cb(name, topic)
  print("subscriber diconnected: " .. name .. " (topic: '" .. topic .. "')")
end

--Constructor
function GazeboEnv:_init(opts)

  opts = opts or {}

  --Constants
	self.arena_width = 15
	self.number_of_colour_channels = 3
	self.number_of_used_colour_channels = 3
	self.number_of_cameras = 2
	self.camera_size = 30
	self.min_reward = -1
	self.max_reward = 100
	self.energy_level = 0
	self.action_magnitude = 5
	self.brake_coefficient = 1
	self.turning_coefficient = 0.2
	self.command_message = ros.Message(msgs.twist_spec)
	self.current_observation = torch.Tensor(self.number_of_used_colour_channels, self.camera_size, self.number_of_cameras):zero()
	self.frequency = opts.threads
	self.step_count = 0
	self.number_steps_in_episode = opts.valSteps --assuming 150 step/s for 180 seconds
	self.updated = false
	self.initialised = false

	--Setup ros node and spinner (processes queued send and receive topics)
	self.spinner = ros.AsyncSpinner()
	self.nodehandle = ros.NodeHandle()
end

--Swarmbot cameras form greyscale Vector with values between 0 and 1
function GazeboEnv:getStateSpec()
	return {'real', {self.number_of_used_colour_channels, self.camera_size, self.number_of_cameras}, {0, 1}}
end

--4 actions required 0:[Both wheels forwards] 1:[Right Wheel forward only] 2:[Left wheel forward only] 3:[Both wheels backwards]
function GazeboEnv:getActionSpec()
  return {'int', 1, {0, 3}}
end

--Min and max reward - Apparently not used for A3C
function GazeboEnv:getRewardSpec()
  return self.min_reward, self.max_reward 
end

--Starts GazeboEnv and spawns all models? Or is that done by individual agents?
function GazeboEnv:start()

	if not self.initialised then 
		self.initialised = true

		--Setup agent's ID (identical to ID of thread)
		self.id = __threadid
		self.model_name = 'swarmbot' .. self.id

		--Configure robot control
		self.command_publisher = self.nodehandle:advertise("/swarmbot" .. self.id .. "/cmd_vel", msgs.twist_spec, 100, false, connect_cb, disconnect_cb)

		--Configure subscriber to receive robots current energy
		self.energy_subscriber = self.nodehandle:subscribe("/swarmbot" .. self.id .. "/energy_level", msgs.float_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
		self.energy_subscriber:registerCallback(function(msg, header)
				--Update current energy level
				self.energy_level = msg.data
		end)

		--Configure sensors
		self.input_subscribers = {}
		for i=1, self.number_of_cameras do
			self.input_subscribers[i] 
			= self.nodehandle:subscribe("/swarmbot" .. self.id .. "/front_colour_sensor_" .. i .. "/image_raw", 
																		msgs.image_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
			self.input_subscribers[i]:registerCallback(function(msg, header)
				--input is taken from msg published by swarmbot
				sensor_input = torch.reshape(msg.data,msg.height*self.number_of_colour_channels*msg.width)
				--Is there a way for torch.reshape to return a DoubleTensor?
				sensor_input = (1/255) * sensor_input:double()
				self.updated = true

				--Colour channels
				for c=1, self.number_of_used_colour_channels do
					for j=1, self.camera_size do
						self.current_observation[c][j][i] = sensor_input[c + self.number_of_colour_channels * (j - 1)]
					end
				end
			end)
		end

		--Publisher to publish position updates
		self.relocation_publisher = self.nodehandle:advertise("/gazebo/set_model_state", msgs.model_state_spec, 100, false, connect_cb, disconnect_cb)
		self.relocation_message = ros.Message(msgs.model_state_spec)

		if __threadid == 0 then
			--Only need to start the spinner once
			self.spinner:start()
		end

	end

	print('[Robot ' .. self.id .. ' finished episode with ' .. self.energy_level .. ' energy]')
	self:random_relocate(self.arena_width)

	--Return first observation
  return self.current_observation
end

function GazeboEnv:random_relocate(distance)

	new_position = distance * (torch.rand(3) - 0.5)
	new_position[3] = 0
	self:relocate(new_position)

end

function GazeboEnv:relocate(new_position)
	m = self.relocation_message
	m.model_name = self.model_name
	m.pose.position.x = new_position[1]
	m.pose.position.y = new_position[2]
	m.pose.position.z = new_position[3]

	self.relocation_publisher:publish(m)
end

function GazeboEnv:step(action)
	--Increment step counter
	self.step_count = self.step_count + 1
	terminal = false

	--Calculate reward
	old_energy = self.energy_level
	ros.spinOnce()
	reward = self.energy_level - old_energy

	--Wait for Gazebo
	while self.id == 1 and not self.updated do
		ros.spinOnce()
	end
	self.updated = false

	action_taken = torch.Tensor(2):zero()
	if 		 action == 0 then
		action_taken[1] = self.action_magnitude
		action_taken[2] = 0
	elseif action == 1 then
		action_taken[1] = 0
		action_taken[2] = self.turning_coefficient * self.action_magnitude
	elseif action == 2 then
		action_taken[1] = 0
		action_taken[2] = -self.turning_coefficient * self.action_magnitude
	elseif action == 3 then
		action_taken[1] = -self.action_magnitude
		action_taken[2] = 0
	end

	--Parse action taken into ROS message and send to robot
	self.command_message.linear.x = action_taken[1];
	self.command_message.angular.z = action_taken[2];
	self.command_publisher:publish(self.command_message)

	--Check if end of episode
	if self.step_count >= self.number_steps_in_episode then
		terminal = true
		self.step_count = 0
	end

  return reward, self.current_observation, terminal
end

return GazeboEnv
