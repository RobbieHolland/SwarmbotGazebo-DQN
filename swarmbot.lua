swarmbot = {}
swarmbot.__index = swarmbot

function swarmbot.create(id, nodehandle, x, y, z)
	--Create object
  local sbot = {}
  setmetatable(sbot,swarmbot)

	--Assign variables
	sbot.id = id
	sbot.model_id = 0
	sbot.energy = 0
	sbot.nodehandle = nodehandle
	sbot.position = torch.Tensor(3):zero()
	sbot.speed = 1
	sbot.previous_speed = 1
	sbot.velocity = torch.Tensor(3):zero()
	sbot.model_name = 'swarmbot' .. sbot.id
	sbot.position_updated = false
	sbot.collision_updated = false

	--Publisher to publish position updates
	sbot.relocation_publisher = sbot.nodehandle:advertise("/gazebo/set_model_state", msgs.model_state_spec, 100, false, connect_cb, disconnect_cb)

	--Updates whether robot is colliding with object
	model_state_subscriber 
		= nodehandle:subscribe("/swarmbot" .. sbot.id .. "/collision_indicator", msgs.contacts_spec, 100, { 'udp', 'tcp' }, { tcp_nodelay = true })
	model_state_subscriber:registerCallback(function(msg, header)
		--For each collision check the names of both objects
		--If either of the names matches a 'food' name then swarmbot eats the food
		for key,value in pairs(msg.states) do
			string = msg.states[key].collision2_name
			colliding_object_1, link1_name = msg.states[key].collision1_name:match("([^,]+)::([^,]+)::([^,]+)")
			colliding_object_2, link2_name = msg.states[key].collision2_name:match("([^,]+)::([^,]+)::([^,]+)")
			type_1, id_1 = colliding_object_1:match("([a-zA-Z]*)([0-9]*)")
			type_2, id_2 = colliding_object_2:match("([a-zA-Z]*)([0-9]*)")
			if type_1 == 'food' then
				sbot:consume(foods[tonumber(id_1)])
			end
			if type_2 == 'food' then
				sbot:consume(foods[tonumber(id_2)])
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
	m.pose.position.x = new_position[1]
	m.pose.position.y = new_position[2]
	m.pose.position.z = new_position[3]

	--Only randomly rotate on z axis (but with Quaternions)
	m.pose.orientation.z = new_orientation[1]
	m.pose.orientation.w = 1
	
	self.relocation_publisher:publish(m)
	self.position = new_position
end

function swarmbot:consume(food)
	food:random_relocate(arena_width)
	self:add_energy(food.value)
end

function swarmbot:add_energy(value)
	self.energy = self.energy + value
end
