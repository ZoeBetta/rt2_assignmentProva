#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/Position.h"
#include "rt2_assignment1/RandomPosition.h"
#include <rt2_assignment1/PlanningAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

bool start = false;
bool notgoing = true;

bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::NodeHandle n1;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n1.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   actionlib::SimpleActionClient<rt2_assignment1::PlanningAction> ac("/go_to_point");
   
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   
   while(ros::ok()){
   	ros::spinOnce();
   	if (start){
		if (notgoing)
		{
   		client_rp.call(rp);
   		rt2_assignment1::PlanningGoal goal;
   		goal.target_pose.header.frame_id = "base_link";
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = rp.response.x;
		goal.target_pose.pose.position.y = rp.response.y;
		goal.target_pose.pose.orientation.z = rp.response.theta;

   		std::cout << "\nGoing to the position: x= " << rp.response.x << " y= " <<rp.response.y << " theta = " <<rp.response.theta << std::endl;
		 ac.sendGoal(goal);
		 std::cout << "Goal sent" << std::endl;
		
		 notgoing = false;
		}
		else {
			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					std::cout << "Goal completed" << std::endl;
					notgoing= true;
				}
   	}
}
   	else {
			if (!notgoing)
			{
				ac.cancelAllGoals();
				std::cout << "Goal cancelled" << std::endl;
				notgoing= true;
			}
	}
  

}
   return 0;
}
