/** 
 *
 *  author: Laura Triglia
 * 
 *  release date: 17/05/2021
 *
 */

#include <chrono>
#include <cinttypes>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "rt2_assignment1/srv/command.hpp"
#include "rt2_assignment1/srv/position.hpp"
#include "rt2_assignment1/srv/random_position.hpp"

using namespace std::chrono_literals;

using Command = rt2_assignment1::srv::Command;
using Position = rt2_assignment1::srv::Position;
using RandomPosition = rt2_assignment1::srv::RandomPosition;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1
{
/** 
 *  This component listens to the user's requests: 
 *  when the robot is moving, it asks for a random
 *  goal pose to "/position_server" that it is send
 *  to "go_to_point" service.
 */

class StateMachine : public rclcpp::Node
{
public:

/**
 * \param options a const rclcpp::NodeOptions &: is used to run this node as a component
 */
  StateMachine(const rclcpp::NodeOptions & options)
  : Node("state_machine",options)
  {
    start = false;
    goal_reached = true;
    /* Initialize the service */
    service_c = this->create_service<Command>( "/user_interface", std::bind(&StateMachine::user_interface, this, _1, _2, _3));
    
    /* Initialize the clients */
    client_rp = this->create_client<RandomPosition>("/position_server");
    while (!client_rp->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }
    
    client_p = this->create_client<Position>("/go_to_point");
    while (!client_p->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }
    
    request_rp = std::make_shared<RandomPosition::Request>();
    response_rp = std::make_shared<RandomPosition::Response>();
    request_p = std::make_shared<Position::Request>();
    
    request_rp->x_max = 5.0;
    request_rp->x_min = -5.0;
    request_rp->y_max = 5.0;
    request_rp->y_min = -5.0; 
    
    /*Check for the goToPoint callback*/
    timer_ = this-> create_wall_timer( 500ms, std::bind(&StateMachine::state_goal,this));
  }
  
  
 private:
/** service callback that set the start/stop robot state
  * 
  * \param request_id a const std::shared_ptr<rmw_request_id_t> : service call header
  * \param request a const std::shared_ptr<Command::Request> : service request, that contains the command
  * \param response a const std::shared_ptr<Command::Response>: service response, that contains the state of the robot (bool start)
  */
   void user_interface(
         const std::shared_ptr<rmw_request_id_t> request_id,
         const std::shared_ptr<Command::Request> request,
         const std::shared_ptr<Command::Response> response)
   {
     (void) request_id;
     if(request->command == "start"){
       
       start = true;
       
       }  
     else{start = false;}
     response->ok = start;
     RCLCPP_INFO(this->get_logger(), "Request received %s", request->command.c_str());
   }
/**
 * Periodically check if the goal is reached
 */   
  void state_goal(){
    
     if(!goal_reached) return;
     
     if(!start) return;
     
     goToPoint_call();
   
   }
   /** 
    *  Set a new goal 
    */
   void goToPoint_call(){
   
     randomPosition_call();
     
     goal_reached = false;
     
     request_p->x = response_rp->x;
     request_p->y = response_rp->y;
     request_p->theta = response_rp->theta;
     
     RCLCPP_INFO(this->get_logger(), "Going to position: x = %f, y = %f, theta = %f", request_p->x, request_p->y, request_p->theta); 
     
     auto goal_reached_callback = [this](rclcpp::Client<rt2_assignment1::srv::Position>::SharedFuture future){(void)future; goal_reached = true;
                                  RCLCPP_INFO(this->get_logger(), "Goal is reached!!!");};
     auto fResult = client_p -> async_send_request(request_p, goal_reached_callback);
   }
   /**
    * Call the random positon server to retrieve the goal pose
    */
   void randomPosition_call(){
   
     auto response_rp_callback = [this](rclcpp::Client<rt2_assignment1::srv::RandomPosition>::SharedFuture future){response_rp = future.get();};
     
     auto fResult = client_rp-> async_send_request(request_rp, response_rp_callback);
   }

   
   rclcpp::Service<Command>::SharedPtr service_c;
   rclcpp::Client<RandomPosition>::SharedPtr client_rp;
   rclcpp::Client<Position>::SharedPtr client_p;
   
   std::shared_ptr<RandomPosition::Request> request_rp;
   std::shared_ptr<RandomPosition::Response> response_rp;
   std::shared_ptr<Position::Request> request_p;
   
   bool start;
   bool goal_reached;
   
   rclcpp::TimerBase::SharedPtr timer_;
 }; /* class closed */

} /* namespace closed*/

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::StateMachine)
