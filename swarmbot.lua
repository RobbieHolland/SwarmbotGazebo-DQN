swarmbot = {}
swarmbot.__index = swarmbot

function swarmbot.create(id, nodehandle, x, y, z, value, typebot)
	--Create object with positions. typebot is optional argument for model name
	-- Value refer to a cost of contact. Can be 0 for swarmbot, should be negative for predators
  local sbot = {}
  setmetatable(sbot,swarmbot)

	-- Default typebot: swarmbot. Replacements: predator prey. Short circuit logic notations
	local typebot = typebot or 'swarmbot' 

	-- Default value: -10 for predators, 0 for swarmbots (check below for activations)
	local value = value or (typebot == 'predator' and -10 or 0)

	--Assign variables
	sbot.id = id
	sbot.model_id = 0
	sbot.energy = 0
	sbot.nodehandle = nodehandle
	sbot.position = torch.Tensor(3):zero()
	sbot.speed = 1
	sbot.previous_speed = 1
	sbot.velocity = torch.Tensor(3):zero()
	sbot.typebot = typebot
	sbot.model_name = typebot .. sbot.id
	sbot.orientation = 0
	sbot.position_updated = false
	sbot.collision_updated = false
	sbot.value = value

	--Publisher to publish position updates
	sbot.relocation_publisher = sbot.nodehandle:advertise("/gazebo/set_model_state", msgs.model_state_spec, 100, false, connect_cb, disconnect_cb)

	--Updates whether robot is colliding with object
	model_state_subscriber 
		= nodehandle:subscribe("/" .. typebot .. sbot.id .. "/collision_indicator", msgs.contacts_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
	model_state_subscriber:registerCallback(function(msg, header)
		--For each collision check the names of both objects
		--If either of the names matches a 'food' name then swarmbot eats the food
		local colliding_object = {}
		local link_name = {}
		local type_obj = {}
		local id_obj = {}
		for key,value in pairs(msg.states) do
			local string = msg.states[key].collision2_name
			colliding_object[1], link_name[1] = msg.states[key].collision1_name:match("([^,]+)::([^,]+)::([^,]+)")
			colliding_object[2], link_name[2] = msg.states[key].collision2_name:match("([^,]+)::([^,]+)::([^,]+)")
			for i=1, 2 do
				type_obj[i], id_obj[i] = colliding_object[i]:match("([a-zA-Z]*)([0-9]*)")

				-- Consume food when touching it
				if type_obj[i] == 'food'     then sbot:consume(foods[tonumber(id_obj[i])]) end
				
				-- Lose energy when touching a predator
				if type_obj[i] == 'predator' then sbot:touched_predator(predators[tonumber(id_obj[i])]) end
				
			end
		end
		sbot.collision_updated = true
	end)

	--Create messages
	sbot.relocation_message = ros.Message(msgs.model_state_spec)

  return sbot
end

function swarmbot.random_relocate(self, distance)
	new_position = distance * (torch.rand(3) - 0.5)
	new_position[3] = 1
	
	--To avoid spawning inside walls
	if torch.abs(new_position[1]) < 1 then
		new_position[1] = new_position[1] + 2
	end
	if torch.abs(new_position[2]) < 1 then
		new_position[2] = new_position[1] + 2
	end

	new_orientation = 2 * (torch.rand(1) - 0.5)
	self:relocate(new_position, new_orientation)
end

function swarmbot:upright()
	self:relocate(self.position)
end

function swarmbot.relocate(self, new_position, new_orientation)
	m = self.relocation_message
	m.model_name = self.model_name
	m.pose.position.x = math.pow(-1, self.id) * 10--new_position[1]
	m.pose.position.y = math.pow(-1, self.id) * 10--new_position[2]
	m.pose.position.z = new_position[3]

	--Only randomly rotate on z axis (but with Quaternions)
	m.pose.orientation.z = new_orientation[1]
	m.pose.orientation.w = 1
	
	self.relocation_publisher:publish(m)
	self.position = new_position
end

function swarmbot:touched_predator(predator)
	self:add_energy(predator.value)
end

function swarmbot:consume(food)
	if food.edible then
		food:random_relocate(arena_width)
		self:add_energy(food.value)
	end
end

function swarmbot:add_energy(value)
	self.energy = self.energy + value
end
