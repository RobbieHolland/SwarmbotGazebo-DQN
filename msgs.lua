local msgs = {}

--Message Formats
msgs.string_spec = ros.MsgSpec('std_msgs/String')
msgs.image_spec = ros.MsgSpec('sensor_msgs/Image')
msgs.jointstate_spec = ros.MsgSpec('sensor_msgs/JointState')
msgs.log_spec = ros.MsgSpec('rosgraph_msgs/Log')
msgs.clock_spec = ros.MsgSpec('rosgraph_msgs/Clock')
msgs.int_spec = ros.MsgSpec('std_msgs/Int16')
msgs.float_spec = ros.MsgSpec('std_msgs/Float32')
msgs.link_spec = ros.MsgSpec('gazebo_msgs/LinkStates')
msgs.twist_spec = ros.MsgSpec('geometry_msgs/Twist')
msgs.odom_spec = ros.MsgSpec('nav_msgs/Odometry')
msgs.model_state_spec = ros.MsgSpec('gazebo_msgs/ModelState')
msgs.model_states_spec = ros.MsgSpec('gazebo_msgs/ModelStates')
msgs.bool_spec = ros.MsgSpec('std_msgs/Bool')
msgs.laser_spec = ros.MsgSpec('sensor_msgs/LaserScan')


return msgs
