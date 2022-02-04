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
	ros::Rate loop_rate(1);
	geometry_msgs::Pose temp_pose; // Working pose for calcs and sub goals

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	static const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);

	// Can be modified as desired
	arm.setPlannerId("RRTConnectkConfigDefault");
	arm.setGoalTolerance(0.001);
	arm.setPlanningTime(20.0);

	// First we add a fake object to prevent the robot from hitting the floor
	moveit_msgs::CollisionObject attached_object;
	attached_object.header.frame_id = "world";
	attached_object.id = "floor";

	// Floor centred below robot default pose
	geometry_msgs::Pose pose;
	pose.orientation.w = 1.0;
	pose.position.x = 0.76;
	pose.position.y = 0.2;
	pose.position.z = -0.30;

	// Floor is a flat box
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 5.0;
	primitive.dimensions[1] = 5.0;
	primitive.dimensions[2] = 0.05;

	// Add the floor
	attached_object.primitives.push_back(primitive);
	attached_object.primitive_poses.push_back(pose);
	attached_object.operation = attached_object.ADD;
	ROS_INFO("adding floor");
	planning_scene_interface.applyCollisionObject(attached_object);
	sleep(1.0);

	// Information for workshop action
	ROS_INFO("Reference frame: %s", arm.getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", arm.getEndEffectorLink().c_str());

	// Showing off Pose Goal
	temp_pose = (arm.getCurrentPose(arm.getEndEffectorLink().c_str())).pose;
	ROS_WARN("Moving the arm");
	temp_pose.position.x -= 0.1;
	arm.setPoseTarget(temp_pose);
	arm.plan(my_plan);
	arm.move();

	// Showing off Named Goal
	ROS_WARN("Moving the arm");
	arm.setNamedTarget("home");
	arm.plan(my_plan);
	arm.move();

	// Showing off Joint Goal
	ROS_WARN("Moving the arm");
	arm.setJointValueTarget("shoulder_pan_joint", -1.57);
	arm.plan(my_plan);
	arm.move();

	// Returning arm Home
	ROS_WARN("Moving the arm");
	arm.setNamedTarget("home");
	arm.plan(my_plan);
	arm.move();

	// Fin,
	ros::shutdown();
	loop_rate.sleep();
	return 0;
}
