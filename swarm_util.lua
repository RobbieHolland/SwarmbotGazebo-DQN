local util = {}

function util.check_received(received, n)
	for i=1, n do
		if not received[i] then
			return false
		end
	end
	return true
end

function append(table, element)
	table[#table + 1] = element
	return table
end

return util
