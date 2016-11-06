-- Extends GazeboEnv to predator
local GazeboEnv = class.class("GazeboEnv", Env)
local GazeboEnvPred =  classic.class('GazeboEnvPred', GazeboEnv)

-- Type of agents (over-written by different environments, such as GazeboEnvPred)
function GazeboEnvPred:type_agent()
	return "predator"
end
