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
	self.number_of_colour_channels_used = 3
	self.number_of_cameras = 2
	self.pixels_per_camera = 30

end

function Body:createBody()
  local histLen = self.recurrent and 1 or self.histLen

  local net = nn.Sequential()
	--4x3x30
  net:add(nn.View(self.histLen * self.number_of_colour_channels_used, 1, self.pixels_per_camera*self.number_of_cameras))
	--16x4x3x1
  net:add(nn.SpatialConvolution(self.histLen * self.number_of_colour_channels_used, 16, 3, 1))
  net:add(nn.ReLU(true))

  return net
	
end

return Body
