/**
 *   author: Laura Triglia
 * 
 *   release date: 17/05/2021
 *
 */

#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"
#include "rt2_assignment1/PoseAction.h"

#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

#include <geometry_msgs/Twist.h>

bool start = false;

/**
 *  The user_interfaces is a service callback that set the robot state on start or stop
 *
 *  \param req a rt2_assignment1::Command::Request & : is a service request, that contains the command
 *  \param res a rt2_assignment1::Command::Request & : is a service response, that contains the value of the state of the robot (bool start)
 *
 */
bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}

/**
 * In the main function a while checks if the goal is reached or is preempted.
 * If the goal is preempted, all goals are deleted and the robot stops. 
 * If the user want to start again the robot, a new random goal is given. 
 * If the user does not want to stop the robot, it continues to move to the goal.   
 */
int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   ros::Publisher pub_vel= n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
   
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;

   actionlib::SimpleActionClient <rt2_assignment1::PoseAction> ac("/go_to_point", true);
   rt2_assignment1::PoseGoal goal;
   geometry_msgs::Twist velocity;

   bool goal_reached=false;

   
   while(ros::ok()){
   	ros::spinOnce();
   	if (start){
   		client_rp.call(rp);
		goal.x=rp.response.x;
		goal.y=rp.response.y;
		goal.theta=rp.response.theta;

		ac.sendGoal(goal);

        while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
           ros::spinOnce();
           if(start==false)
           {
              /* the goal is preempted, so we need to cancel all the goals, in order to stop the robot*/
              ac.cancelAllGoals();
              goal_reached=false;

              break;

           }
            goal_reached=true;
        }
	
    }

    else
    {
      /* just a quick check that the robot actually stopped, so all the velocity are set to zero*/
            velocity.linear.x=0;
            velocity.angular.z=0;
            pub_vel.publish(velocity);

    }

    /*In the case the user decide to not stop the robot, the user is notified of the achievemente of the goal*/
    if(goal_reached)
    {
         std::cout << "\nPosition reached" << std::endl;
         goal_reached=false;
    }
           
   	}
   return 0;
}
