#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;



#define NUM_GOALS_T 3
double goals_triangle[NUM_GOALS_T][4] = {
	{1, -2.0, -0.5, 0.866}, // x y z w
	{-1.0, -2.0, 1, 0}, 
	{0.0, 0.0, 0.5, 0.866},	
};

#define NUM_GOALS 4
double goals_square[NUM_GOALS][4] = {
	{0.0, -1.0, 0.7071, -0.7071},
	{-1.0, -1.0, 1, 0.0}, 
	{-1.0, 0.0, 0.7071, 0.7071},
	{0.0, 0.0, 0.0, 1},	
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "navigation_goals");
	MoveBaseClient ac("move_base", true);

	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

    int a;
        
    ROS_INFO("Choose trajectory:\n1. Triangle\n2. Square");

    std::cin >> a;
    int n;
    switch(a){

        case 1: 
            n=3;
        break;

        case 2:
            n=4;
        break;
    }


	for(int i = 0; i < n; ++i)
	{  	if (n == 3){
        goal.target_pose.pose.position.x = goals_triangle[i][0];
		goal.target_pose.pose.position.y = goals_triangle[i][1];
		goal.target_pose.pose.orientation.z = goals_triangle[i][2];
		goal.target_pose.pose.orientation.w = goals_triangle[i][3];
        ROS_INFO("Sending goal: (%f; %f)", goals_triangle[i][0], goals_triangle[i][1]);        
		ac.sendGoal(goal);
        }
        else{
        goal.target_pose.pose.position.x = goals_square[i][0];
		goal.target_pose.pose.position.y = goals_square[i][1];
		goal.target_pose.pose.orientation.z = goals_square[i][2];
		goal.target_pose.pose.orientation.w = goals_square[i][3];
        ROS_INFO("Sending goal: (%f; %f)", goals_square[i][0], goals_square[i][1]);        
		ac.sendGoal(goal);
        }

		ac.waitForResult();
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Goal achieved!");


		ros::Duration(1.0).sleep();
	}
	ROS_INFO("Done.");
	return 0;
}




