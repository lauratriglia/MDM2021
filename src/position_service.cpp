/** 
 *
 * author: Laura Triglia
 * 
 * release date: 17/05/2021
 *
 */

#include "rt2_assignment1/srv/random_position.hpp"
#include <memory>
#include <inttypes.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using RandomPosition = rt2_assignment1::srv::RandomPosition;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1{

/**
 * This component generates a random position 
 */

class PositionServer : public rclcpp::Node
{

public:

/**
 * The constructor instanciates the server
 * \param option a const rclcpp::NodeOptions &: is used to run this node as a component
 */
  PositionServer(const rclcpp::NodeOptions &options) : Node("random_position_server", options)
  {
    /* Initialize the service */
    service_c = this->create_service<RandomPosition>( "/position_server", std::bind(&PositionServer::myrandom, this, _1, _2, _3));
  }
  
private:

 /**
  * Generate a random numer
  * \param M a double: is the lower bound
  * \param N a double: is the upper bound
  * \retval randMToN a double: return a random number linearly distributed between M and N
  */
  double randMToN(double M, double N)
  {     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

 /**
  * This is the service callback that generate a random (x,y,theta) pose
  * \param request_id a const std::shared_ptr<rmw_request_id_t> : is a service call header
  * \param request a const std::shared_ptr<RandomPosition::Request> : is a service request, that contains the (x,y) ranges
  * \param response a const std::shared_ptr<RandomPosition::Response> : is a service response, that contain (x,y,theta)
  */
  void myrandom (
      const std::shared_ptr<rmw_request_id_t> request_id,
      const std::shared_ptr<RandomPosition::Request> request,
      const std::shared_ptr<RandomPosition::Response> response)
  {
    (void)request_id;
    response->x = randMToN(request->x_min, request->x_max);
    response->y = randMToN(request->y_min, request->y_max);
    response->theta = randMToN(-3.14, 3.14);
    
  }
  rclcpp::Service<RandomPosition>::SharedPtr service_c;

}; /* class ended*/

} /* namespace ended*/

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::PositionServer)

