#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char **argv)
{
	// Declarations
	ros::init(argc, argv, "moveit_expl");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	geometry_msgs::Pose working_pose; // Working pose for calcs and sub goals

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	static const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);

	// Can be modified as desired
	arm.setPlannerId("RRTConnectkConfigDefault");
	arm.setGoalTolerance(0.01);
	arm.setPlanningTime(20.0);

	// First we add a fake object to prevent the robot from hitting the floor
	moveit_msgs::CollisionObject attached_object;
	attached_object.header.frame_id = "world";
	attached_object.id = "floor";

	// Floor centred below robot default pose
	working_pose.orientation.w = 1.0;
	working_pose.position.x = 0.76;
	working_pose.position.y = 0.2;
	working_pose.position.z = -0.30;

	// Floor is a flat box
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 5.0;
	primitive.dimensions[1] = 5.0;
	primitive.dimensions[2] = 0.05;

	// Add the floor
	attached_object.primitives.push_back(primitive);
	attached_object.primitive_poses.push_back(working_pose);
	attached_object.operation = attached_object.ADD;
	ROS_INFO("adding floor");
	planning_scene_interface.applyCollisionObject(attached_object);
	sleep(1.0);

	// Information for workshop action
	ROS_INFO("Reference frame: %s", arm.getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", arm.getEndEffectorLink().c_str());

	// Showing off Pose Goal
	working_pose = (arm.getCurrentPose(arm.getEndEffectorLink().c_str())).pose;
	working_pose.position.x -= 0.1;
	ROS_WARN
	(
		"Moving arm to Pose:\nx:%f\ny:%f\nz:%f\n",
		working_pose.position.x,
		working_pose.position.y,
		working_pose.position.z
	);
	arm.setPoseTarget(working_pose);
	arm.plan(my_plan);
	arm.move();

	// Showing off Named Goal
	ROS_WARN("Moving arm to Named Goal: \"home\"");
	arm.setNamedTarget("home");
	arm.plan(my_plan);
	arm.move();

	// Showing off Joint Goal
	ROS_WARN("Moving arm to Joint Goal: \"shoulder_pan_joint\": -1.57");
	arm.setJointValueTarget("shoulder_pan_joint", -1.57);
	arm.plan(my_plan);
	arm.move();

	// Returning arm Home
	ROS_WARN("Moving arm back to home");
	arm.setNamedTarget("home");
	arm.plan(my_plan);
	arm.move();

	// Fin,
	ros::shutdown();
	return 0;
}
