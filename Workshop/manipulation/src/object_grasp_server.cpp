#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <gazebo_ros_link_attacher/Attach.h>

// Look for spelling mistake and commented lines
// Look for the following commented lines in the code to guide you on where to
// add your own code

/*************ADD SOME CODE HERE (START)**********
|
|
|
|
|
|
************ADD SOME CODE HERE (END)**************/


class GraspTag
{

public:
	GraspTag(ros::NodeHandle* nh);

	void setPoseCallback(const geometry_msgs::Pose::ConstPtr &pose);
	bool serviceCallback
	(
		std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res
	);
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
	bool _execute_grasp;
	bool _action_complete;
	bool _srv_success;
	ros::Publisher gripper_grasp_pub;
	trajectory_msgs::JointTrajectory _gripper_angle_traj;
	trajectory_msgs::JointTrajectoryPoint _gripper_angle_point;
	gazebo_ros_link_attacher::Attach _attach_srv;
	ros::ServiceClient attach_tag_client;
	ros::ServiceClient detach_tag_client;
};

GraspTag::GraspTag(ros::NodeHandle* nh):n(*nh)
{
	pose_sub = n.subscribe("/tag_pose", 3, &GraspTag::setPoseCallback, this);
	gripper_grasp_pub = n.advertise<trajectory_msgs::JointTrajectory>
	(
		"/gripper_controller/command", 2
	);
	attach_tag_client = n.serviceClient<gazebo_ros_link_attacher::Attach>
	(
		"/link_attacher_node/attach"
	);
	detach_tag_client = n.serviceClient<gazebo_ros_link_attacher::Attach>
	(
		"/link_attacher_node/detach"
	);
	DontExecuteGrasp();
	_attach_srv.request.model_name_1 = "robot";
	_attach_srv.request.link_name_1 = "wrist_3_link";
	_attach_srv.request.model_name_2 = "apriltag";
	_attach_srv.request.link_name_2 = "my_box";
}

bool GraspTag::serviceCallback
(
	std_srvs::SetBool::Request &req,
	std_srvs::SetBool::Response &res
)
{

	ROS_INFO("service called");

	_execute_grasp = true;
	_action_complete = false;
	while (_action_complete == false && !ros::isShuttingDown())
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
	target_pose = *pose;
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
	_action_complete = true;
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

	_gripper_angle_traj.joint_names.push_back
	(
		"simple_gripper_right_driver_joint"
	);
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
	// Declarations
	ros::init(argc, argv, "object_grasp_server");
	ros::NodeHandle n;
	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::Rate loop_rate(1.0);
	GraspTag graspObj(&n); // Instance of the class GraspTag

	// MoveIt Classes -> Highly recommended to google these when you can!
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	static const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::PlanningSceneInterface
		planning_scene_interface;
	moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);

	// Can be modified as desired
	arm.setPlannerId("RRTConnect");
	arm.setGoalTolerance(0.0001);
	arm.setPlanningTime(5.0);

	ROS_INFO("Reference frame: %s", arm.getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", arm.getEndEffectorLink().c_str());
	ROS_INFO("Pose:HOME");
	arm.setNamedTarget("home");
	if (arm.plan(my_plan) != moveit::core::MoveItErrorCode::SUCCESS)
	{
		// Stop here and try again later if it doesn't work.
		ROS_ERROR(
			"MoveIt cannot make a plan to the \"HOME\" position from here!\n"
			"Reduce the tolerances on the planner, or move the arm to a new "
			"position and try again."
		);
	}
	else if (arm.move() != moveit::core::MoveItErrorCode::SUCCESS)
	{
		// If the execution fails, stop here and wait for new call.
		ROS_ERROR(
			"The arm cannot move to \"HOME\" from here, even though moveit "
			"made a plan successfully!\n"
			"Reduce the colerances on the execution, or move the arm to a new "
			"position and try again."
		);
	}
	else
	{
		// Wait just to make sure it is stopped
		sleep(1.0);

		// creating the server called grasp
		ros::ServiceServer service = n.advertiseService
		(
			"grasp", &GraspTag::serviceCallback, &graspObj
		);

		while (ros::ok()) // keep on running until stopped
		{
			graspObj.DontExecuteGrasp();

			ROS_WARN("waiting for tag positions");
			ros::topic::waitForMessage<geometry_msgs::Pose>("/tag_pose");
			ROS_INFO("Received tag position");
			graspObj.CloseGripper(0.0);
			ROS_INFO("Ready for Grasp service call");

			while (!graspObj.ExecuteGrasp() && !ros::isShuttingDown())
			{
				//wait
				loop_rate.sleep();
			}
			if (ros::isShuttingDown())
			{
				break;
			}

			ROS_INFO("Grasp server active");
			geometry_msgs::Pose tag_pose;
			tag_pose = graspObj.getTarget(); // Target the tag position
			ROS_INFO
			(
				"Tag location:\n x:%lf \n y:%lf \n z:%lf",
				tag_pose.position.x,
				tag_pose.position.y,
				tag_pose.position.z
			);

			tag_pose.position.z += 0.25; //offset by .25 meters in the z axis
			tag_pose.position.x -= 0.05;
			tag_pose.position.y += 0.05;
			tag_pose.orientation.w = 0.707;
			tag_pose.orientation.x = 0.0;
			tag_pose.orientation.y = 0.707;
			tag_pose.orientation.z = 0.0; // and pointed straight down
			arm.setPoseTarget(tag_pose, arm.getEndEffectorLink().c_str());
			if (arm.plan(my_plan) != moveit::core::MoveItErrorCode::SUCCESS)
			{
				// If the plan fails, stop here and wait for new call.
				ROS_ERROR(
					"MoveIt wasn't able to make a plan to the Tag!"
				);
				graspObj.setSuccess(false);
				continue;
			}
			if (arm.move() != moveit::core::MoveItErrorCode::SUCCESS)
			{
				// If the execution fails, stop here and wait for new call.
				ROS_ERROR(
					"The arm wasn't able to move to the Tag!"
				);
				graspObj.setSuccess(false);
				continue;
			}
			sleep(1.0); // Ensure it is definately stopped.

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
			graspObj.CloseGripper(0.25); // closing gripper
			sleep(1.0);

			//graspObj.AttachGripper(); // Attaching object
			ROS_INFO("Attaching objects");

			tag_pose.position.z += 0.2; //raising cube in the air
			arm.setPoseTarget(tag_pose, arm.getEndEffectorLink().c_str());
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
			arm.setPoseTarget(tag_pose, arm.getEndEffectorLink().c_str());
			arm.plan(my_plan); // check if plan succeded
			arm.move();
			sleep(1.0);

			ROS_INFO("Detaching Object");
			graspObj.DetachGripper(); // Detaching object
			graspObj.CloseGripper(0.0);
			sleep(1);

			ROS_INFO("Moving to Home");
			arm.setNamedTarget("home");
			arm.plan(my_plan);
			arm.move();
			ROS_INFO("finished motion plan");
			graspObj.setSuccess(true);

			loop_rate.sleep();
		}
	}
	return 0;
}
