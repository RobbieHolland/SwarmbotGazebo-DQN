require 'torch'
sample_size = 12
space = 2

data = torch.linspace(5, 50, sample_size)
print(data)

function linspace(sample_size, space)
	x = torch.Tensor(torch.floor(sample_size / space) + 1)
	for i=1, torch.floor(sample_size / space) do
		x[i+1] = space * i
	end
	x[1] = 0
	return x
end

function spaced_data(linspace, sample_size, space, data)
	y = torch.Tensor(torch.floor(sample_size / space) + 1):zero()
	total = 0
	count = 0
	data_count = 1

	for i=1, sample_size do
		total = total + data[i]
		count = count + 1
		if i % space == 0 then
			y[data_count + 1] = total / count
			total = 0
			count = 0
			data_count = data_count + 1
		end

	end

	y[1] = 0

	return y
end

x = linspace(sample_size, space)
y = spaced_data(x, sample_size, space, data)
print(x)
print(y)
