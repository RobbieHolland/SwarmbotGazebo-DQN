--Initialise
ros = require 'ros'
ros.init('GazeboDQN_Env')
local classic = require 'classic'
msgs = require 'msgs'
srvs = require 'srvs'
util = require 'swarm_util'
local GazeboEnv, super = classic.class('GazeboEnv', Env)

function connect_cb(name, topic)
  print("subscriber connected: " .. name .. " (topic: '" .. topic .. "')")
end


function disconnect_cb(name, topic)
  print("subscriber diconnected: " .. name .. " (topic: '" .. topic .. "')")
end

--Constructor
function GazeboEnv:_init(opts)

  opts = opts or {}

  --Constants and variables
	self.episode_time = 300
	self.old_energy = 0
	self.arena_width = 16
	self.number_colour_channels = 3
	self.number_channels = 4
	self.laser_scan_range = 2
	self.number_of_cameras = 1
	self.camera_size = 30
	self.min_reward = 0
	self.max_reward = 3
	self.energy = 0
	self.forward_speed = 0.5
	self.angular_speed = 0.6
	self.command_message = ros.Message(msgs.twist_spec)
	self.current_observation = torch.Tensor(self.number_channels, self.camera_size, self.number_of_cameras):zero()
	self.step_count = 0
	self.number_steps_in_episode = opts.valSteps --assuming 150 step/s for 180 seconds
	self.number_of_sensors = 2
	self.updated = {false, false, false}
	self.initialised = false
	self.command_sent = false
	self.current_time = 0
	self.latest_sensor_updates = {0, 0, 0}
	self.scan_sensor_index = 2

	--Setup ros node and spinner (processes queued send and receive topics)
	self.nodehandle = ros.NodeHandle()
end

--Swarmbot cameras form greyscale Vector with values between 0 and 1
function GazeboEnv:getStateSpec()
	return {'real', {self.number_channels, self.camera_size, self.number_of_cameras}, {0, 1}}
end

--4 actions required 0:[Both wheels forwards] 1:[Right Wheel forward only] 2:[Left wheel forward only] 3:[Both wheels backwards]
function GazeboEnv:getActionSpec()
  return {'int', 1, {0, 2}}
end

--Min and max reward - Apparently not used for A3C
function GazeboEnv:getRewardSpec()
  return self.min_reward, self.max_reward 
end

--Starts GazeboEnv and spawns all models? Or is that done by individual agents?
function GazeboEnv:start()

	if not self.initialised then
		self.initialised = true
		--Wait for services to be setup or crash
		ros.Duration(4):sleep()
		--Setup agent's ID (identical to ID of thread)
		self.id = __threadid or 0
		self.model_name = 'swarmbot' .. self.id

		--Configure robot control
		self.command_publisher 
				= self.nodehandle:advertise("/swarmbot" .. self.id .. "/network_command", msgs.twist_spec, 100, false, connect_cb, disconnect_cb)
		if self:isValidationAgent() then
			self.command_publisher 
				= self.nodehandle:advertise("/swarmbot" .. self.id .. "/cmd_vel", msgs.twist_spec, 100, false, connect_cb, disconnect_cb)
		end

		--Setup timer for epoch terminal timing
		self.clock_subscriber = self.nodehandle:subscribe("/clock", msgs.clock_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
		self.clock_subscriber:registerCallback(function(msg, header)
			self.current_time = msg.clock:toSec()
		end)

		--Configure service caller to relocate at end of episode
		random_relocate_request_service = self.nodehandle:serviceClient('/random_relocate_request', srvs.data_request_spec)
		random_relocate_message = random_relocate_request_service:createRequest()
		random_relocate_message.id = self.id

		--Configure service caller to get energy of robot
		energy_request_service = self.nodehandle:serviceClient('/energy_request', srvs.data_request_spec)
		energy_request_message = energy_request_service:createRequest()
		energy_request_message.id = self.id

		--Configure service caller to get speed of robot
		speed_request_service = self.nodehandle:serviceClient('/speed_request', srvs.data_request_spec)
		speed_request_message = speed_request_service:createRequest()
		speed_request_message.id = self.id

		--Configure subscriber to check if command has been sent by buffer
		self.command_sent_subscriber 
				= self.nodehandle:subscribe("/commands_sent" .. self.id, msgs.bool_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
		self.command_sent_subscriber:registerCallback(function(msg, header)
			self.command_sent = msg.data
		end)

		--Configure sensors
			--Colour sensors
			self.camera_input_subscribers = {}
			for i=1, self.number_of_cameras do
				self.camera_input_subscribers[i] 
				= self.nodehandle:subscribe("/swarmbot" .. self.id .. "/front_colour_sensor_" .. i .. "/image_raw", 
																			msgs.image_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
				self.camera_input_subscribers[i]:registerCallback(function(msg, header)
					--input is taken from msg published by swarmbot
					sensor_input = torch.reshape(msg.data,msg.height*self.number_colour_channels*msg.width)
					--Is there a way for torch.reshape to return a DoubleTensor?
					sensor_input = (1/255) * sensor_input:double()
					self.updated[i] = true
					self.latest_sensor_updates[i] = os.clock()

					--Colour channels
					for c=1, self.number_colour_channels do
						for j=1, self.camera_size do
							self.current_observation[c][j][i] = sensor_input[c + self.number_colour_channels * (j - 1)]
						end
					end
				end)
			end

			--Range sensor
			self.laser_input_subscriber 
				= self.nodehandle:subscribe("/swarmbot" .. self.id .. "/scan_sensor", msgs.laser_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
			self.laser_input_subscriber:registerCallback(function(msg, header)
				--input is taken from msg published by swarmbot
				count = 1
				for i=1, self.camera_size * self.number_of_cameras do
					j = torch.ceil(i / self.camera_size)
					self.current_observation[4][count][j] = msg.ranges[1 + self.camera_size * self.number_of_cameras - i] / self.laser_scan_range

					count = i % self.camera_size
					count = count + 1
				end

				self.updated[self.scan_sensor_index] = true
				self.latest_sensor_updates[self.scan_sensor_index] = os.clock()
			end)

		if not ros.isStarted() then
			self.spinner = ros.AsyncSpinner()
			self.spinner:start()
		end
	end

	print('[Robot ' .. self.id .. ' finished episode with ' .. self.energy .. ' energy]')
	random_relocate_request_service:call(energy_request_message)
	self.start_time = self.current_time

	--Return first observation
  return self.current_observation
end

function GazeboEnv:isValidationAgent()
	return self.id == 0
end

function GazeboEnv:parse_action(action)
	--self.current_speed = speed_request_service:call(speed_request_message).data

	action_taken = torch.Tensor(2):zero()
	if 		 action == 0 then
		action_taken[1] = self.forward_speed
		action_taken[2] = 0
	elseif action == 1 then
		action_taken[1] = self.forward_speed
		action_taken[2] = self.angular_speed
	elseif action == 2 then
		action_taken[1] = self.forward_speed
		action_taken[2] = -self.angular_speed
	end
	--[[
	elseif action == 3 then
		action_taken[1] = 0
		action_taken[2] = self.angular_speed
	elseif action == 4 then
		action_taken[1] = 0
		action_taken[2] = -self.angular_speed
	end
	--]]
	return action_taken
end

function GazeboEnv:step(action)
	--Increment step counter
	self.step_count = self.step_count + 1
	terminal = false

	--Wait for Gazebo sensors to update (Ensures a meaningfull history)
	while not util.check_received(self.updated, self.number_of_sensors) do
		ros.spinOnce()
	end
	for i=1, self.number_of_sensors do
		self.updated[i] = false
	end

	--[[
	if self.id == 1 then
		print('Sensor updates - 1: ' .. self.latest_sensor_updates[1] .. ', 2: ' .. self.latest_sensor_updates[2] .. ', 3: ' .. self.latest_sensor_updates[3])
	end
	--]]

	--Parse action given by DQN
	action_taken = self:parse_action(action)
	self.command_message.linear.x = action_taken[1];
	self.command_message.angular.z = action_taken[2];

	--Check if end of episode
	terminal = self.current_time - self.start_time > self.episode_time

	--Wait for command buffer to send command
	while not self.command_sent do
		self.command_publisher:publish(self.command_message)
		ros.spinOnce()
	end
	self.command_sent = false

	ros.Duration(0.01):sleep()
	--[[
	if self.id == 1 then
		print('Command was sent at: ' .. os.clock())
	end
	-]]

	--Calculate reward as result of action
	self.old_energy = self.energy
	response = energy_request_service:call(energy_request_message)
	self.energy = response.data
	reward = self.energy - self.old_energy
		--Add green pixel reward
		reward = reward + self.current_observation[2]:sum() / self.camera_size

  return reward, self.current_observation, terminal
end

return GazeboEnv
