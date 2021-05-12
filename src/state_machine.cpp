
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rt2_assignment1/srv/command.hpp"
#include "rt2_assignment1/srv/position.hpp"
#include "rt2_assignment1/srv/random_position.hpp"
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
using namespace std;
using UserInt = rt2_assignment1::srv::Command;
using GoalPos = rt2_assignment1::srv::Position;
using RndPos = rt2_assignment1::srv::RandomPosition;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
bool start = false;

namespace my_composition_package {

class GoalPositionClient : public rclcpp::Node
	{
		public:
			GoalPositionClient() : Node("Goal_client")
				{
						client_ = this->create_client<GoalPos>("/go_to_point");
						while(!client_ -> wait_for_service(std::chrono::seconds(1)))
							{
								if (!rclcpp::ok())
									{
										RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
										return;
									}
								//RCLCPP_INFO(this->get_logger()), "aiting for service /go_to_point to appear.");
							}
						this->request_=std::make_shared<GoalPos::Request>();
						this->response_=std::make_shared<GoalPos::Response>();	
				}
				
				void call_serverGTP()
					{
						auto result_future= client_->async_send_request(request_);
						if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
							{
								RCLCPP_ERROR(this->get_logger(), "service call failed");
							}
						this->response_=result_future.get();
					}
					
				std::shared_ptr<GoalPos::Request> request_;
				std::shared_ptr<GoalPos::Response> response_;
	
		private:
			rclcpp::Client<GoalPos>::SharedPtr client_;
	
	};	
	
class RandomPosClient : public rclcpp::Node
	{
		public:
			RandomPosClient() : Node("Random_pos")
				{
					client = this->create_client<RndPos>("/position_server");
					while(!client -> wait_for_service(std::chrono::seconds(1)))
						{
							if (!rclcpp::ok())
								{
									RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
									return;
								}
						}
					this->request=std::make_shared<RndPos::Request>();
					this->response=std::make_shared<RndPos::Response>();	
				}
				
			void call_serverRP()
				{
					auto result_future= client->async_send_request(request);
					if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
						{
							RCLCPP_ERROR(this->get_logger(), "service call failed");
						}
					this->response=result_future.get();
				}
				
			std::shared_ptr<RndPos::Request> request;
			std::shared_ptr<RndPos::Response> response;
		
		private:
			rclcpp::Client<RndPos>::SharedPtr client;		
	};
	
class StateMachine : public rclcpp::Node
	{
		public:
			StateMachine(const rclcpp::NodeOptions & options) : Node("State_machine", options)
				{
					timer_ =this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&StateMachine::timer_callback, this));
	
					service_ = this->create_service<UserInt>("/user_interface", std::bind(&StateMachine::handle_service, this, _1, _2, _3));
				}
	
		private:
		
			void handle_service( 
			const std::shared_ptr<rmw_request_id_t> request_header,
			const std::shared_ptr<UserInt::Request> request,
			const std::shared_ptr<UserInt::Response> response)
			{
				(void) request_header;

				if (request->command == "stop")
					{
						std::cout << "switch" << std::endl;
						start= false;
						response->ok = true;
							
					}
				else if ( request->command == "start")
					{
						std::cout << "switch" << std::endl;
						start= true;
						response->ok=true;
						
					}
				response->ok=true;
			}
			
			
			void timer_callback()
				{
					
					auto goal_position = std::make_shared<GoalPositionClient>();
					auto random_position = std::make_shared<RandomPosClient>();
					if (start == false)
						{

						}
					else if ( start == true)
						{
							random_position->request->x_max = 5.0;
							random_position->request->x_min = -5.0;
							random_position->request->y_max = 5.0;
							random_position->request->y_min = -5.0;
							random_position->call_serverRP();
							goal_position->request_->x=random_position->response->x;
							goal_position->request_->y=random_position->response->y;
							goal_position->request_->theta=random_position->response->theta;
							std::cout << "\nGoing to the position: x= " << goal_position->request_->x << " y= " <<goal_position->request_->y << " theta = " <<goal_position->request_->theta << std::endl;
							goal_position->call_serverGTP();
							std::cout << "Position reached" << std::endl;
							
						}		
				}
				

			
			rclcpp::Service<UserInt>::SharedPtr service_;
			rclcpp::TimerBase::SharedPtr timer_;
	
	};
	
}

RCLCPP_COMPONENTS_REGISTER_NODE(my_composition_package::StateMachine)	

