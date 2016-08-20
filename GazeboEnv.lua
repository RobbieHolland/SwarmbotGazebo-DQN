ros = require 'ros'
ros.init('GazeboDQN_Env')
local classic = require 'classic'
msgs = require 'async/SwarmbotGazebo-DQN/msgs'
local GazeboEnv, super = classic.class('GazeboEnv', Env)
resp_ready = false

-- Constructor
function GazeboEnv:_init(opts)

  opts = opts or {}
	
  -- Constants
	self.number_of_colour_channels = 3
	self.number_of_cameras = 2
	self.camera_size = 15
	self.min_reward = 0
	self.max_reward = 1e5
	self.energy = 0
	self.current_observation = torch.Tensor(1, self.camera_size, self.number_of_cameras):zero()
	--self.current_position = torch.Tensor(3):zero()

	--setup ros node and spinner (processes queued send and receive topics)
	self.spinner = ros.AsyncSpinner()
	self.nodehandle = ros.NodeHandle()
end

--Swarmbot cameras form greyscale Vector with values between 0 and 1
function GazeboEnv:getStateSpec()
	return {'real', {1, self.camera_size, self.number_of_cameras}, {0, 1}}
end

-- 4 actions required 0:[Neither wheel] 1:[Right Wheel only] 2:[Left wheel only] 3:[Both wheels]
function GazeboEnv:getActionSpec()
  return {'int', 1, {0, 3}}
end

-- Min and max reward - Apparently not used for A3C
function GazeboEnv:getRewardSpec()
  return self.min_reward, self.max_reward 
end

-- Starts GazeboEnv and spawns all models? Or is that done by individual agents?
function GazeboEnv:start()
	--if not validation agent
	if __threadid ~= 0 then
		self.id = __threadid
		--os.execute('rosrun gazebo_ros spawn_model -x ' .. 0 .. ' -y ' .. 0 .. ' -z ' .. 1 .. ' -file `rospack find swarm_simulator`/sdf/test_model.sdf' .. 
  	--					 ' -sdf -model swarmbot' .. self.id .. ' -robot_namespace swarmbot' .. self.id)
	
		--Configure robot control
	  self.command_publisher = self.nodehandle:advertise("/swarmbot" .. self.id .. "/cmd_vel", msgs.twist_spec, 100, false, connect_cb, disconnect_cb)

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
--[[
		--Configure position sensor
		self.odom_subscriber = self.nodehandle:subscribe("/swarmbot" .. self.id .. "/base_pose", msgs.odom_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
	  self.odom_subscriber:registerCallback(function(msg, header)
			--position published by robot
			self.current_position[1] = msg.pose.pose.position.x
			self.current_position[2] = msg.pose.pose.position.y
			self.current_position[3] = msg.pose.pose.position.z
  	end)
--]]
	else
		--Only need to start the spinner once
		self.spinner:start()
	end

	--Return first observation
  return self.current_observation
end

function GazeboEnv:step(action)
	ros.spinOnce()
	--Return reward, observation, terminal flag
	reward = 0
	terminal = false
  return reward, self.current_observation, terminal
end

return GazeboEnv
