#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"


/**
 *  Generate a random number 
 *
 *  \param M a double: is the lower bound
 *  \param N a double: is the upper bound
 *
 *  \retval randMToN a double: return a random number, linearly distruibuted between M and N
 */
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

/**
 *  myrandom is a service callback taht generate a random (x,y,theta) pose
 *
 *  \param req a rt2_assignment1::RandomPosition::Request &: is a service request, that contains the (x,y) ranges.
 *  \param res a rt2_assignment1::RandomPosition::Response &: is a service response, that contains (x,y,theta).
 */

bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}

/**
 *  The main function is a server that generate a random position
 */
int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
