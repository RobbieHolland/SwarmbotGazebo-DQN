-- Extends GazeboEnv to predator
local GazeboEnv = require 'GazeboEnv'
local GazeboEnvPred =  classic.class('GazeboEnvPred', GazeboEnv)

-- Type of agents (over-written by different environments, such as GazeboEnvPred)
function GazeboEnvPred:type_agent()
	return "predator"
end
