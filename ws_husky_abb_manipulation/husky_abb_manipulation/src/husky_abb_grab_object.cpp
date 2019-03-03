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
#include <std_srvs/SetBool.h>
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

class Subscribe
{

  public:
	Subscribe();

	void setPoseCallback(const geometry_msgs::Pose::ConstPtr &pose);
	bool serviceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
	void setFalse();
	void setSuccess(bool success);
	bool getFlag();
	geometry_msgs::Pose getTarget();
	

  private:
	geometry_msgs::Pose target_pose;
	ros::NodeHandle n;
	ros::Subscriber pose_sub;;
	bool _flag;
	bool _flag2;
	bool _srv_success;
};






Subscribe::Subscribe()
{

	pose_sub = n.subscribe("/tag_pose", 1, &Subscribe::setPoseCallback, this);
	setFalse();
}


bool Subscribe::serviceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

		ROS_INFO("service called");
		
		_flag = true;
		_flag2 = false;
		while(_flag2==false)
		{sleep(0.1);}
		if(_srv_success){
		res.success = _srv_success;
		res.message = "all motion succeeded";
		}
		else{
		res.success = _srv_success;	
		res.message = "falied to exceute all motion";
		}

		return true;

}

void Subscribe::setPoseCallback(const geometry_msgs::Pose::ConstPtr &pose)
{
	target_pose.position = pose->position;
	target_pose.orientation = createQuaternionMsgFromRollPitchYaw(1.57,0, 0);
	

}

geometry_msgs::Pose Subscribe::getTarget()
{
	ROS_INFO("Retreiving target pose");
	ROS_INFO("Target pose received:\n x:[%f]\n y:[%f]\n z:[%f]\n w:[%f]", target_pose.position.x, target_pose.position.y, target_pose.position.z, target_pose.orientation.w);
	return (target_pose);
}

void Subscribe::setFalse()
{
	_flag = false;
}

void Subscribe::setSuccess(bool success)
{
	_srv_success= success;
	_flag2=true;
}

bool Subscribe::getFlag()
{
	return _flag;
}

int main(int argc, char **argv)
{
	//delcarations

	ros::init(argc, argv, "husky_ur5_grab_object");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(2);
	spinner.start();
	int mode = 0;
	ros::Rate loop_rate(1);
	ros::NodeHandle n;
	bool sim;					   // exceute planned path if we are in simulation
	geometry_msgs::Pose temp_pose; //temporary pose to check when the same target is receive
	Subscribe get_pose;			   // instance of the class Subscribe

	moveit::planning_interface::MoveGroupInterface::Plan my_plan; // plan containing the trajectory
	static const std::string PLANNING_GROUP = "abb_arm";		  // planning group
	static const std::string PLANNING_GROUP2 = "barrett_hand";
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface; // planning interface
	moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);			 // planning group
	moveit::planning_interface::MoveGroupInterface hand(PLANNING_GROUP2);		 // planning group
	arm.setPlannerId("RRTConnectkConfigDefault");
	//can be modified as desired
	arm.setGoalTolerance(0.001);
	arm.setPlanningTime(5.0);
	//hand.setPlannerId("LBKPIECEkConfigDefault");
	hand.setPlannerId("RRTConnectkConfigDefault");
	//can be modified as desired
	hand.setGoalJointTolerance(0.1);
	hand.setPlanningTime(5.0);
	//arm.setPlanningTime(10.0);
	//arm.setPlanningTime(15.0);

	// end of declarations

	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true); // to display path in Rviz
	moveit_msgs::DisplayTrajectory display_trajectory;

	ROS_INFO("Reference frame: %s", arm.getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", arm.getEndEffectorLink().c_str());

	temp_pose = (arm.getCurrentPose(arm.getEndEffectorLink().c_str())).pose;

	arm.setNamedTarget("home");
	arm.plan(my_plan); // check if plan succeded
	arm.move();

	hand.setJointValueTarget("finger_2_med_joint", 0);
	hand.setJointValueTarget("finger_1_med_joint", 0);
	hand.setJointValueTarget("finger_3_med_joint", 0);
	hand.plan(my_plan);
	hand.move();

	sleep(4.0);



	ros::ServiceServer service = n.advertiseService("grasp", &Subscribe::serviceCallback, &get_pose);

	while (true) // keep on running until stoped
	{

		get_pose.setFalse();
		while (!get_pose.getFlag())
		{
		}
		ROS_INFO("starting motion plan");
		temp_pose = get_pose.getTarget();
		temp_pose.position.x -=0.15;
		temp_pose.position.z += 0.03;
		arm.setPoseTarget(temp_pose, arm.getEndEffectorLink().c_str());
		arm.plan(my_plan); // check if plan succeded
		moveit::planning_interface::MoveItErrorCode success = arm.move();

		if (success==moveit_msgs::MoveItErrorCodes::SUCCESS){
		hand.setJointValueTarget("finger_2_med_joint",1.56);
		hand.setJointValueTarget("finger_1_med_joint", 1.56);
		hand.setJointValueTarget("finger_3_med_joint", 1.56);
		hand.plan(my_plan);
		success = hand.move();
		}
	
		sleep(12.0);
		if (success==moveit_msgs::MoveItErrorCodes::SUCCESS){
		temp_pose.position.z += 0.13;
		arm.setPoseTarget(temp_pose, arm.getEndEffectorLink().c_str());
		arm.plan(my_plan); // check if plan succeded
		success = arm.move();
		}

		ROS_INFO("finished motion plan");

		if (success==moveit_msgs::MoveItErrorCodes::SUCCESS){
		get_pose.setSuccess(true);
		}

		else{
			get_pose.setSuccess(false);
		}
		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}