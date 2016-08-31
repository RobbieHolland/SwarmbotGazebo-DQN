number_of_sensors = 1
for i=1, number_of_sensors do
	os.execute('gnome-terminal -e \"bash -c \"roslaunch swarm_simulator soup.launch gui:=false ; exec bash\"\"')
	--os.execute('gnome-terminal -e \"bash -c \"rosrun image_view image_view image:=/swarmbot' .. arg[1] .. '/front_colour_sensor_' .. i .. '/image_raw ; exec bash\"\"')
end
