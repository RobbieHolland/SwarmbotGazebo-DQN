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
	--Concatenate history in channel dimension
	--2D input tensor for temporal convolution
  net:add(nn.View(self.histLen, 1*15*2))
	--Each frame has size 1*15*2, outputFrameSize is 4 (is this the number of outputs of the network?), kernel size is 4, kernel step is 1
  net:add(nn.TemporalConvolution(1*15*2, 4, 4, 1))
  net:add(nn.ReLU(true))

  return net
	
end

return Body
