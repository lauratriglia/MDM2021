"""
.. module:: go_to_point
    	:platform: Unix
	:synopsis: Node implementing the go_to_point behavior

 This node controls the behavior of the robot via an action server.

 Publishes to:
 	/cmd_vel (geometry_msgs.msg.Twist)
 
 ServiceServer:
 	/set_vel (rt2_assignment1.srv.SetVel)
 
 ActionServer:
 	/go_to_point(rt2_assignment1.action.PoseAction


"""
