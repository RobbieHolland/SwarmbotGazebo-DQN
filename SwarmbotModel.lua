local nn = require 'nn'
require 'classic.torch' -- Enables serialisation

local Body = classic.class('Body')

-- Constructor
function Body:_init(opts)
	
  opts = opts or {}

  self.recurrent = opts.recurrent
  self.histLen = opts.histLen
  self.stateSpec = opts.stateSpec
end

function Body:createBody()
  local histLen = self.recurrent and 1 or self.histLen

  local net = nn.Sequential()
  net:add(nn.View(4, 1, 15, 2)) -- Concatenate history in channel dimension
  --net:add(nn.SpatialConvolution(4, 32, 8, 8, 4, 4, 1, 1))
  --net:add(nn.SpatialConvolution(32, 4, 4, 4, 2, 2))

	--For testing purposes
	net:add(nn.View(120))
	net:add(nn.Linear(120, 4))

  return net
	
end

return Body
