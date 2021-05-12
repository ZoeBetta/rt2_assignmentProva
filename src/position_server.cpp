
#include <inttypes.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rt2_assignment1/srv/random_position.hpp"
#include <iostream>

using namespace std;
using RandomPositionSRV = rt2_assignment1::srv::RandomPosition;
rclcpp::Node::SharedPtr g_node = nullptr;

double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<RandomPositionSRV::Request> request,
  const std::shared_ptr<RandomPositionSRV::Response> response)
{
  (void)request_header;
  response->x= randMToN(request->x_min, request->x_max);
  response->y= randMToN(request->y_min, request->y_max);
  response->theta = randMToN(-3.14, 3.14);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("position_server");
  auto server = g_node->create_service<RandomPositionSRV>("/position_server", handle_service);
  
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}
