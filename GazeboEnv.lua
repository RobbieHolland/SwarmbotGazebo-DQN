local classic = require 'classic'

local GazeboEnv, super = classic.class('GazeboEnv', Env)

-- Constructor
function GazeboEnv:_init(opts)
  opts = opts or {}
  
  -- Constants
	self.camera_size = 15
	self.number_of_cameras = 2
	self.min_reward = 0
	self.max_reward = 1e5
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
	print(__threadid)

	--Return first observation
	first_observation = torch.Tensor(30)
  return first_observation
end


function GazeboEnv:step(action)

	--Return reward, observation, terminal flag
	reward = 0
	observation = torch.Tensor(30)
	terminal = false
  return reward, observation, terminal
end

return GazeboEnv
