#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <signal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <gazebo_ros_link_attacher/Attach.h>
// start of the GraspTag class//



/*************ADD SOME CODE HERE (START)**********
|
|
|
|
|
|
************ADD SOME CODE HERE (END)**************/

// The above shows you where code should be added

// Look for spelling mistake and commented lines

// Only read from line 178

using namespace tf;

void SigintHandler(int sig)
{

	ROS_INFO("SHUTING DOWN()");
	ros::shutdown();
}

class GraspTag
{

public:
	GraspTag();

	void setPoseCallback(const geometry_msgs::Pose::ConstPtr &pose);
	bool serviceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
	void DontExecuteGrasp();
	void setSuccess(bool success);
	bool ExecuteGrasp();
	void CloseGripper(float angle);
	void AttachGripper();
	void DetachGripper();
	geometry_msgs::Pose getTarget();

private:
	geometry_msgs::Pose target_pose;
	ros::NodeHandle n;
	ros::Subscriber pose_sub;
	;
	bool _execute_grasp;
	bool _succeeded;
	bool _srv_success;
	ros::Publisher gripper_grasp_pub;
	trajectory_msgs::JointTrajectory _gripper_angle_traj;
	trajectory_msgs::JointTrajectoryPoint _gripper_angle_point;
	gazebo_ros_link_attacher::Attach _attach_srv;
	ros::ServiceClient attach_tag_client;
	ros::ServiceClient detach_tag_client;
};

GraspTag::GraspTag()
{

	signal(SIGINT, SigintHandler);
	pose_sub = n.subscribe("/tag_pose", 1000, &GraspTag::setPoseCallback, this);
	gripper_grasp_pub = n.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 1000);
	attach_tag_client = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
	detach_tag_client = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
	DontExecuteGrasp();
	_attach_srv.request.model_name_1 = "robot";
	_attach_srv.request.link_name_1 = "wrist_3_link";
	_attach_srv.request.model_name_2 = "apriltag";
	_attach_srv.request.link_name_2 = "my_box";
}

bool GraspTag::serviceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{

	ROS_INFO("service called");

	_execute_grasp = true;
	_succeeded = false;
	while (_succeeded == false && !ros::isShuttingDown())
	{
		sleep(0.1);
	}
	if (_srv_success)
	{
		res.success = _srv_success;
		res.message = "all motion succeeded";
	}
	else
	{
		res.success = _srv_success;
		res.message = "falied to exceute all motion";
	}

	return true;
}

void GraspTag::setPoseCallback(const geometry_msgs::Pose::ConstPtr &pose)
{
	target_pose.position = pose->position;
	target_pose.orientation = createQuaternionMsgFromRollPitchYaw(1.57, 0, 0);
}

geometry_msgs::Pose GraspTag::getTarget()
{
	return (target_pose);
}

void GraspTag::DontExecuteGrasp()
{
	_execute_grasp = false;
}

void GraspTag::setSuccess(bool success)
{
	_srv_success = success;
	_succeeded = true;
}

bool GraspTag::ExecuteGrasp()
{
	return _execute_grasp;
}

void GraspTag::CloseGripper(float angle)
{
	ros::Rate _loop_rate(10);
	if (angle < 0.0)
		angle = 0.0;
	else if (angle > 1.5707)
		angle = 1.5707;

	_gripper_angle_traj.joint_names.push_back("simple_gripper_right_driver_joint");
	_gripper_angle_point.positions.push_back(angle);
	_gripper_angle_point.time_from_start.nsec = 5000000;
	_gripper_angle_traj.points.push_back(_gripper_angle_point);

	ros::Time now = ros::Time::now();
	ros::Time then = ros::Time::now();

	while ((then - now).toSec() < 1.0)
	{
		gripper_grasp_pub.publish(_gripper_angle_traj);
		then = ros::Time::now();
		_loop_rate.sleep();
	}
	ROS_INFO("Gripper command completed");
	_gripper_angle_point.positions.clear();
	_gripper_angle_traj.points.clear();
	_gripper_angle_traj.joint_names.clear();
}

void GraspTag::AttachGripper()
{
	attach_tag_client.call(_attach_srv);
}

void GraspTag::DetachGripper()
{
	detach_tag_client.call(_attach_srv);
}

int main(int argc, char **argv)
{
	//delcarations

	signal(SIGINT, SigintHandler);

	ros::init(argc, argv, "object_grasp_server");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::Rate loop_rate(1);
	ros::NodeHandle n;
	geometry_msgs::Pose temp_pose; //temporary pose to check when the same target is receive
	GraspTag grasObj;			   // instance of the class GraspTag

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;				 // plan containing the trajectory
	static const std::string PLANNING_GROUP = "manipulator";					 // planning group
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface; // planning interface
	moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);			 // planning group
	arm.setPlannerId("RRTConnect");
	//can be modified as desired
	arm.setGoalTolerance(0.0001);
	arm.setPlanningTime(5.0);
	// end of declarations

	ROS_INFO("Reference frame: %s", arm.getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", arm.getEndEffectorLink().c_str());

	temp_pose = (arm.getCurrentPose(arm.getEndEffectorLink().c_str())).pose;

	ROS_INFO("Pose:HOME");
	arm.setNamedTarget("home");
	arm.plan(my_plan); // check if plan succeded
	arm.move();

	sleep(4.0);

	ros::ServiceServer service = n.advertiseService("grasp", &GraspTag::serviceCallback, &grasObj); // creating the server called grasp

	while (ros::ok) // keep on running until stoped
	{
		grasObj.DontExecuteGrasp();
		ros::topic::waitForMessage<geometry_msgs::Pose>("/tag_pose");
		grasObj.CloseGripper(0.0);
		ROS_INFO("tag detected");

		ROS_INFO("Ready for Grasp service call");
		while (!grasObj.ExecuteGrasp() && !ros::isShuttingDown())
		{
			//wait
			loop_rate.sleep();
		}
		ROS_INFO("Grasp server active");
		temp_pose = grasObj.getTarget(); //set the desired pose to the position of the cube

		ROS_INFO("Tag location:\n x:%lf \n c:%lf \n z:%lf", temp_pose.position.x, temp_pose.position.y, temp_pose.position.z);

		temp_pose.position.z += 0.25; //offset by .25 meters in the z axis
		temp_pose.position.x -= 0.05;
		temp_pose.position.y += 0.05;

		temp_pose.orientation.w = 0.707;
		temp_pose.orientation.x = 0.0;
		temp_pose.orientation.y = 0.707;
		temp_pose.orientation.z = 0.0; // and pointed straight down

		arm.setPoseTarget(temp_pose, arm.getEndEffectorLink().c_str());

		arm.plan(my_plan); // check if plan succeded

		arm.move();

		//sleep(5.0);
		sleep(1.0);

		/*************ADD SOME CODE HERE (START)**********
		|
		|
		|
		|
		|
		|
		************ADD SOME CODE HERE (END)**************/

		sleep(1.0);
		ROS_INFO("Closing Gripper");

		grasObj.CloseGripper(0.25); // closing gripper

		//sleep(2.0);
		sleep(1.0);
		//grasObj.AttachGripper(); //attaching object
		ROS_INFO("Attaching objects");

		temp_pose.position.z += 0.2; //raising cube in the air

		arm.setPoseTarget(temp_pose, arm.getEndEffectorLink().c_str());
		arm.plan(my_plan); // check if plan succeded
		arm.move();

		// moving object to next stand

		/*************ADD SOME CODE HERE (START)**********
		|
		|
		|
		|
		|
		|
		************ADD SOME CODE HERE (END)**************/

		sleep(1.0);

		arm.setPoseTarget(temp_pose, arm.getEndEffectorLink().c_str());
		arm.plan(my_plan); // check if plan succeded
		arm.move();
		sleep(1.0);

		ROS_INFO("Detaching Object");
		grasObj.DetachGripper(); // Detaching object
		grasObj.CloseGripper(0.0);
		sleep(1);


		ROS_INFO("Moving to Home");
		arm.setNamedTarget("home"); // This is needed so that the robot arm will not block the LIDAR
		arm.plan(my_plan);
		arm.move();

		ROS_INFO("finished motion plan");

		grasObj.setSuccess(true);

		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}