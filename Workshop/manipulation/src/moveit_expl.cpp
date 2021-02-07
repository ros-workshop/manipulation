#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
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
	static const std::string PLANNING_GROUP = "manipulator";		  // planning group
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface; // planning interface
	moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);			 // planning group
	arm.setPlannerId("RRTConnectkConfigDefault");
	//can be modified as desired
	arm.setGoalTolerance(0.001);
	arm.setPlanningTime(20.0);
	//arm.setPlanningTime(10.0);
	//arm.setPlanningTime(15.0);

	// end of declarations

	ROS_INFO("Reference frame: %s", arm.getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", arm.getEndEffectorLink().c_str());

	temp_pose = (arm.getCurrentPose(arm.getEndEffectorLink().c_str())).pose;

    /*pose goal */
    temp_pose.position.x -=0.1;
    arm.setPoseTarget(temp_pose);
    arm.plan(my_plan); 
	arm.move();

    /*name goal */
	arm.setNamedTarget("home");
	arm.plan(my_plan); 
	arm.move();


    /*joint goal */
	arm.setJointValueTarget("shoulder_pan_joint",-1.57);
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