#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include "std_msgs/Bool.h"
#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
#include <stdlib.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include <vector>
#include <string>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Float64MultiArray.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
// start of the subscribe class//
using namespace tf;

int main(int argc, char **argv)
{
	//delcarations

	ros::init(argc, argv, "moveit_expl");
	ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
	ros::Rate loop_rate(1);
	geometry_msgs::Pose temp_pose; //temporary pose to check when the same target is receive

	moveit::planning_interface::MoveGroupInterface::Plan my_plan; // plan containing the trajectory
	static const std::string PLANNING_GROUP = "abb_arm";		  // planning group
	static const std::string PLANNING_GROUP2 = "barrett_hand";
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface; // planning interface
	moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);			 // planning group
	moveit::planning_interface::MoveGroupInterface hand(PLANNING_GROUP2);		 // planning group
	arm.setPlannerId("RRTConnectkConfigDefault");
	//can be modified as desired
	arm.setGoalTolerance(0.001);
	arm.setPlanningTime(20.0);
	//hand.setPlannerId("LBKPIECEkConfigDefault");
	hand.setPlannerId("RRTConnectkConfigDefault");
	//can be modified as desired
	hand.setGoalJointTolerance(0.1);
	hand.setPlanningTime(20.0);
	//arm.setPlanningTime(10.0);
	//arm.setPlanningTime(15.0);

	// end of declarations

	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true); // to display path in Rviz
	moveit_msgs::DisplayTrajectory display_trajectory;

	ROS_INFO("Reference frame: %s", arm.getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", arm.getEndEffectorLink().c_str());

	temp_pose = (arm.getCurrentPose(arm.getEndEffectorLink().c_str())).pose;

    /*pose goal */
    temp_pose.position.x +=0.1;
    arm.setPoseTarget(temp_pose);
    arm.plan(my_plan); 
	arm.move();

    /*name goal */
	arm.setNamedTarget("home");
	arm.plan(my_plan); 
	arm.move();


    /*joint goal */
	arm.setJointValueTarget("abb2_joint_1",-1.57);
	arm.plan(my_plan); 
	arm.move();

    /*name goal */
	arm.setNamedTarget("home");
	arm.plan(my_plan); 
	arm.move();

    ros::shutdown();

	loop_rate.sleep();

	return 0;
}