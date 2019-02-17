#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "std_msgs/Bool.h"
#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
#include <random_numbers/random_numbers.h>
#include <stdlib.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <industrial_msgs/CmdJointTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "ros/ros.h"
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit_msgs/ExecuteTrajectoryGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/MoveGroupGoal.h>
#include <moveit_msgs/MoveGroupAction.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"
#include <vector>
#include <string>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Float64MultiArray.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <apriltags2_ros/AprilTagDetectionArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
// start of the subscribe class//

using namespace tf;

class Subscribe
{

  public:
	Subscribe();

	void setPoseCallback(const geometry_msgs::Pose::ConstPtr &pose);
	void setActionCallback(const geometry_msgs::Point::ConstPtr &action);
	void setFalse();
	bool getFlag();
	geometry_msgs::Pose getTarget();
	geometry_msgs::Point getAction();
	void setActive(bool setting);
	bool getActive();

  private:
	geometry_msgs::Pose target_pose;
	ros::NodeHandle n;
	ros::Subscriber pose_sub;
	ros::Subscriber action_sub;
	geometry_msgs::Point m_action;
	bool _active;
	bool _flag;
};

Subscribe::Subscribe()
{

	pose_sub = n.subscribe("/tag_pose", 1, &Subscribe::setPoseCallback, this);
	action_sub = n.subscribe("/action_ddpg", 1, &Subscribe::setActionCallback, this);
	setFalse();
}

void Subscribe::setActionCallback(const geometry_msgs::Point::ConstPtr &action)
{
	m_action = *action;
	
	//ROS_INFO("Action received: %d ",m_action.data);
}

void Subscribe::setPoseCallback(const geometry_msgs::Pose::ConstPtr &pose)
{
	target_pose.position = pose->position;
	//target_pose.position.x = 1;
	//target_pose.position.y = 0.0;
	//target_pose.position.z = 0.7;

	//target_pose.orientation = createQuaternionMsgFromRollPitchYaw(0, 3.14159 / 2, atan2(target_pose.position.y, target_pose.position.x) + 3.14159);
	target_pose.orientation = createQuaternionMsgFromRollPitchYaw(0,3.14159, 0);
	_flag = true;

}

geometry_msgs::Pose Subscribe::getTarget()
{
	ROS_INFO("Retreiving target pose");
	ROS_INFO("Target pose received:\n x:[%f]\n y:[%f]\n z:[%f]\n w:[%f]", target_pose.position.x, target_pose.position.y, target_pose.position.z, target_pose.orientation.w);
	return (target_pose);
}

geometry_msgs::Point Subscribe::getAction()
{
	ROS_INFO("Retreiving action");

	return (m_action);
}

void Subscribe::setFalse()
{
	_flag = false;
}

bool Subscribe::getFlag()
{
	return _flag;
}

int main(int argc, char **argv)
{
	//delcarations

	ros::init(argc, argv, "husky_ur5_grab_ball");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	int mode = 0;
	ros::Rate loop_rate(1);
	ros::NodeHandle n;
	bool sim;					   // exceute planned path if we are in simulation
	geometry_msgs::Pose temp_pose; //temporary pose to check when the same target is receive
	Subscribe get_pose;			   // instance of the class Subscribe

	moveit::planning_interface::MoveGroupInterface::Plan my_plan; // plan containing the trajectory
	static const std::string PLANNING_GROUP = "ur5_arm";		  // planning group
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

	arm.setJointValueTarget("urshoulder_lift_joint", -1.57);
	arm.setJointValueTarget("urshoulder_pan_joint", 0);
	arm.setJointValueTarget("urelbow_joint", 1.57);
	arm.setJointValueTarget("urwrist_1_joint", -3.14159);
	arm.setJointValueTarget("urwrist_2_joint", -1.57);
	arm.setJointValueTarget("urwrist_3_joint", -1.57);
	arm.plan(my_plan); // check if plan succeded
	arm.move();

	sleep(4.0);

	/*ROS_INFO("x: %lf", temp_pose.position.x);
	ROS_INFO("y: %lf", temp_pose.position.y);
	arm.setPoseTarget(temp_pose, arm.getEndEffectorLink().c_str());
	arm.plan(my_plan); // check if plan succeded
	arm.move();
	/*std::vector<geometry_msgs::Pose> waypoints;
	moveit_msgs::RobotTrajectory trajectory;
	float fraction;
	sleep(4.0);
	temp_pose.position.z -= 0.01;
	waypoints.push_back(temp_pose);
	fraction = arm.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
	my_plan.trajectory_ = trajectory;
	waypoints.clear();
	arm.execute(my_plan);*/
	while (true) // keep on running until stoped
	{

		get_pose.setFalse();
		while (!get_pose.getFlag())
		{
		}

		temp_pose = get_pose.getTarget();
		temp_pose.position.z +=0.35;
		arm.setPoseTarget(temp_pose, arm.getEndEffectorLink().c_str());
		arm.plan(my_plan); // check if plan succeded
		arm.move();
		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}