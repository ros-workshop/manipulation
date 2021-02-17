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

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;				 // plan containing the trajectory
	static const std::string PLANNING_GROUP = "manipulator";					 // planning group
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface; // planning interface
	moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);			 // planning group
	arm.setPlannerId("RRTConnectkConfigDefault");
	//can be modified as desired
	arm.setGoalTolerance(0.001);
	arm.setPlanningTime(20.0);
	//arm.setPlanningTime(10.0);
	//arm.setPlanningTime(15.0);

	// end of declarations

	/*First we add a fake object to prevent the robot from hitting the floor */

	moveit_msgs::CollisionObject attached_object;
	/* The header must contain a valid TF frame*/
	attached_object.header.frame_id = "world";
	/* The id of the object */
	attached_object.id = "floor";

	/* A default pose */
	geometry_msgs::Pose pose;
	pose.orientation.w = 1.0;
	pose.position.x = 0.76;
	pose.position.y = 0.2;
	pose.position.z = -0.30;
	
	/* Define a box to be attached */
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 5.0;
	primitive.dimensions[1] = 5.0;
	primitive.dimensions[2] = 0.05;

	attached_object.primitives.push_back(primitive);
	attached_object.primitive_poses.push_back(pose);
	attached_object.operation = attached_object.ADD;

	//**********************Defining first collision object End**********************//

	ROS_INFO("adding floor");
	//moveit_msgs::CollisionObject remove_object;

	
	planning_scene_interface.applyCollisionObject(attached_object);
	sleep(1.0);

	ROS_INFO("Reference frame: %s", arm.getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", arm.getEndEffectorLink().c_str());

	temp_pose = (arm.getCurrentPose(arm.getEndEffectorLink().c_str())).pose;

	/*pose goal */
	ROS_WARN("Moving the arm");
	temp_pose.position.x -= 0.1;
	arm.setPoseTarget(temp_pose);
	arm.plan(my_plan);
	arm.move();

	/*name goal */
	ROS_WARN("Moving the arm");
	arm.setNamedTarget("home");
	arm.plan(my_plan);
	arm.move();

	/*joint goal */
	ROS_WARN("Moving the arm");
	arm.setJointValueTarget("shoulder_pan_joint", -1.57);
	arm.plan(my_plan);
	arm.move();

	/*name goal */
	ROS_WARN("Moving the arm");
	arm.setNamedTarget("home");
	arm.plan(my_plan);
	arm.move();

	ros::shutdown();

	loop_rate.sleep();

	return 0;
}