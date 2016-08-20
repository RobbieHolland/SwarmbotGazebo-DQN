local nn = require 'nn'
require 'classic.torch' -- Enables serialisation

local Body = classic.class('Body')

-- Constructor
function Body:_init(opts)
	print('INIT')
  opts = opts or {}

  self.recurrent = opts.recurrent
  self.histLen = opts.histLen
  self.stateSpec = opts.stateSpec

end

function Body:createBody()
  local histLen = self.recurrent and 1 or self.histLen
	print('CREATE BODY')
	--os.execute('rosrun gazebo_ros spawn_model -x ' .. 0 .. ' -y ' .. 0 .. ' -z ' .. 1 .. ' -file `rospack find swarm_simulator`/sdf/test_model.sdf' .. 
--						 ' -sdf -model swarmbot' .. __threadid .. ' -robot_namespace swarmbot' .. __threadid)

  local net = nn.Sequential()
  net:add(nn.View(self.histLen, 1, 15, 2)) -- Concatenate history in channel dimension
  --net:add(nn.SpatialConvolution(4, 32, 8, 8, 4, 4, 1, 1))
  --net:add(nn.SpatialConvolution(32, 4, 4, 4, 2, 2))

	--For testing purposes
	net:add(nn.View(120))
	net:add(nn.Linear(120, 4))

  return net
	
end

return Body
