local srvs = {}

--Service Formats
srvs.data_request_spec = ros.SrvSpec('std_srvs/Data_request')
srvs.model_state_spec = ros.SrvSpec('gazebo_msgs/SetModelState')
srvs.empty_spec = ros.SrvSpec('std_srvs/Empty')

return srvs
