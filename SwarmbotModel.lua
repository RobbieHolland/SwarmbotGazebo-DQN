local nn = require 'nn'
require 'classic.torch' -- Enables serialisation

local Body = classic.class('Body')

-- Constructor
function Body:_init(opts)
  opts = opts or {}

  self.recurrent = opts.recurrent
  self.histLen = opts.histLen
  self.stateSpec = opts.stateSpec
	self.hiddenSize = opts.hiddenSize

end

function Body:createBody()
  local histLen = self.recurrent and 1 or self.histLen

  local net = nn.Sequential()
  net:add(nn.View(self.histLen, 1, 15*2))
  net:add(nn.SpatialConvolution(self.histLen, 16, 3, 1))
  net:add(nn.ReLU(true))

  return net
	
end

return Body
