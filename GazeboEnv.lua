--Initialise
ros = require 'ros'
ros.init('GazeboDQN_Env')
local classic = require 'classic'
msgs = require 'async/SwarmbotGazebo-DQN/msgs'
local GazeboEnv, super = classic.class('GazeboEnv', Env)
resp_ready = false

function connect_cb(name, topic)
  print("subscriber connected: " .. name .. " (topic: '" .. topic .. "')")
end


function disconnect_cb(name, topic)
  print("subscriber diconnected: " .. name .. " (topic: '" .. topic .. "')")
end

--Constructor
function GazeboEnv:_init(opts)

  opts = opts or {}
	
  --Constants
	self.number_of_colour_channels = 3
	self.number_of_cameras = 2
	self.camera_size = 15
	self.min_reward = 0
	self.max_reward = 1e5
	self.energy_level = 0
	self.action_magnitude = 0.8
	self.command_message = ros.Message(msgs.twist_spec)
	self.current_observation = torch.Tensor(1, self.camera_size, self.number_of_cameras):zero()
	--self.current_position = torch.Tensor(3):zero()

	--Setup ros node and spinner (processes queued send and receive topics)
	self.spinner = ros.AsyncSpinner()
	self.nodehandle = ros.NodeHandle()
end

--Swarmbot cameras form greyscale Vector with values between 0 and 1
function GazeboEnv:getStateSpec()
	return {'real', {1, self.camera_size, self.number_of_cameras}, {0, 1}}
end

--4 actions required 0:[Neither wheel] 1:[Right Wheel only] 2:[Left wheel only] 3:[Both wheels]
function GazeboEnv:getActionSpec()
  return {'int', 1, {0, 3}}
end

--Min and max reward - Apparently not used for A3C
function GazeboEnv:getRewardSpec()
  return self.min_reward, self.max_reward 
end

--Starts GazeboEnv and spawns all models? Or is that done by individual agents?
function GazeboEnv:start()
	--if not validation agent
	if __threadid ~= 0 then
		--Setup agent's ID (identical to ID of thread)
		self.id = __threadid

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
			
				--Green channel
				for j=1, self.camera_size do
					self.current_observation[1][j][i] = sensor_input[2 + self.number_of_colour_channels * (j - 1)]
				end
			end)
		end
	else
		--Only need to start the spinner once
		self.spinner:start()
	end

	--Return first observation
  return self.current_observation
end

function GazeboEnv:step(action)
	--Calculate reward
	old_energy = self.energy_level
	ros.spinOnce()
	reward = self.energy_level - old_energy

	--Find corresponding action (change this to binary version?)
	action_taken = torch.Tensor(2):zero()
	if 		 action == 0 then
		action_taken[1] = 0
		action_taken[2] = 0
	elseif action == 1 then
		action_taken[1] = 0
		action_taken[2] = self.action_magnitude
	elseif action == 2 then
		action_taken[1] = 0
		action_taken[2] = -self.action_magnitude
	elseif action == 3 then
		action_taken[1] = self.action_magnitude
		action_taken[2] = 0
	end

	--Parse action taken into ROS message and send to robot
	self.command_message.linear.x = action_taken[1];
	self.command_message.angular.z = action_taken[2];
	self.command_publisher:publish(self.command_message)

	--Return reward, observation, terminal flag
	terminal = false
  return reward, self.current_observation, terminal
end

return GazeboEnv
