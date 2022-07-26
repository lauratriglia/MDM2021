"""
.. module:: go_to_point
    :platform: Unix
    :synopsis: Node for making the robot move toward the goal
    
.. moduleauthor:: Carmine Recchiuto <carmine.recchiuto@dibris.unige.unige.it>, Laura Triglia <4494106@studenti.unige.it>
  
This node implements a state machine to make the robot reach the goal position.
To go deeper in the description of this script, firstly the robot is oriented
in the direction of the goal and starts moving towards the goal itself. To consider
reached a goal, the robot not only reaches the x and y coordinates, but it rotates 
to reach the correct orientation. 
The user can interact with the robot throught the jupyter notebook that has the function
of an user interface. The user can change both linear and angular speed, that are updated
each time a request for the /set_vel is received

Publisher:
    /cmd_vel

Subscriber:
    /odom

Action Client:
    /go_to_point
"""
#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import numpy as np
import math
import actionlib
import actionlib.msg
import rt2_assignment1.msg

from rt2_assignment1.srv import SetVel, SetVelResponse, Command

# robot state variables
position_ = Point()
yaw_ = 0
position_ = 0
state_ = 0

#publisher
pub_ = None

#action server
act_s = None

#velocity server
vel_s = None

# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

def srv_set_vel(req):
    """
    /set_vel server
    It receives the maximum and angular speed 
    from the SetVek message 
    Args:
    	req(SetVel): set_vel request
    	
    """
    global ub_a, ub_d
    
    ub_a = req.angular
    ub_d = req.linear
    rospy.logdebug("RECEIVED lin: {} ang:{}".format(req.linear, req.angular))
    return SetVelResponse()

def clbk_odom(msg):
    """
    This function is dedicated to the Odometry callback.
    It retrieves x,y, and theta from the odom message
    
    Args:
    	msg (Odometry): odometry message
    """
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):
    """
    This function is dedicated to change the global state to the most recent one
    
    Args:
    	state (int): new current state
    """
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


def normalize_angle(angle):
    """
    This function is dedicated to normalize an angle between pi and -pi
    
    Args:
    	angle (float): not-normalized angle
    
    Returns:
    	angle (float): normalized angle
    """
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos):
    """
    This function has multiple purposies. However the main one is to orient
    the robot in the desidered way. 
    The other important aspect which the function is dedicated is the changing
    of the state based on the current one. 
    
    Args:
    	des_pos (float): desidered position
    """
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if np.abs(twist_msg.angular.z) > ub_a:
            twist_msg.angular.z = np.sign(twist_msg.angular.z)*ub_a
            
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(1)


def go_straight_ahead(des_pos):
    """
    This function has the aim to drive the robot toward the goal.
    It also regulates the linear and angular speed depending on the
    distance to the goal pose.
    
    Args:
    	des_pos (Point): desidered position
    """
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub_.publish(twist_msg)
    else: # state change conditions
        #print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

def fix_final_yaw(des_yaw):
    """
    This function is dedicated to calculate the error between
    the current orientation and the desidered one. 
    If the errror is below a given threshold the state is changed
    to done
    
    Args:
    	des_yaw (float): desidered orientation
    	
    """
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)
        
def done():
    """
    This function stops the robot, setting the linear and 
    angular velocities to zero
    """
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)
    
def go_to_point(goal):

    """ 
    Depending on the different state of the robot, go_to_point set the correct behaviour
    to reach the goal
    The robot will stop only when the goal is reached or when is preempted.
    
    Args:
    	goal (PoseActionGoal): x,y,theta goal pose
        
    """
    global act_s
    desired_position = Point()
    desired_position.x = goal.x
    desired_position.y = goal.y
    des_yaw = goal.theta
    
    success = True
    rate = rospy.Rate(20)
    change_state(0)
    
    feedback = rt2_assignment1.msg.PoseFeedback()
    result = rt2_assignment1.msg.PoseResult()
    
    while not rospy.is_shutdown():
        if act_s.is_preempt_requested():
            rospy.loginfo('Goal was preempted')
            act_s.set_preempted()
            success = False
            break
        else:
            if state_ == 0:            
                feedback.stat = "Fixing the yaw"
                act_s.publish_feedback(feedback)
                fix_yaw(desired_position)
            elif state_ == 1:
                feedback.stat = "Angle aligned"
                act_s.publish_feedback(feedback)
                go_straight_ahead(desired_position)
            elif state_ == 2:
                feedback.stat = "Yaw fixed"
                act_s.publish_feedback(feedback)
                fix_final_yaw(des_yaw)
            elif state_ == 3:	    
                feedback.stat = "Target reached!"
                act_s.publish_feedback(feedback)
                done()
            else:
                rospy.logerr('Unknown state!')
    
        rate.sleep()  
    if success:
        result.effect = success
        rospy.loginfo('Goal: Succeeded!')
        act_s.set_succeeded(result) 
    
def main():
    global pub_, act_s
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    act_s = actionlib.SimpleActionServer(
        '/go_to_point', rt2_assignment1.msg.PoseAction, go_to_point, auto_start=False)
    act_s.start()
    
    vel_s = rospy.Service('/set_vel', SetVel, srv_set_vel)
    rospy.spin()

if __name__ == '__main__':
    main()
