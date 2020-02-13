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
// These messages are used to manually publish cmd_vel
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

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

	pose_sub = n.subscribe("/tag_pose", 1000, &Subscribe::setPoseCallback, this);
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

void Subscribe::setPoseCallback(const geometry_msgs::Pose::ConstPtr& pose)
{
	target_pose.position = pose->position;
	target_pose.orientation = createQuaternionMsgFromRollPitchYaw(1.57,0, 0);
}

geometry_msgs::Pose Subscribe::getTarget()
{
	//ROS_INFO("Retreiving target pose");
	//ROS_INFO("Target pose received:\n x:[%f]\n y:[%f]\n z:[%f]\n w:[%f]", target_pose.position.x, target_pose.position.y, target_pose.position.z, target_pose.orientation.w);
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


class command_vel {
  public:
    void start_publisher () {
      husky_commands =  nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel",1000);
    }

	void stop_husky() {
		velocity_commands.linear.x = 0.0;
        velocity_commands.linear.y = 0.0;
        velocity_commands.linear.z = 0.0;

        velocity_commands.angular.x = 0.0;
        velocity_commands.angular.y = 0.0;
        velocity_commands.angular.z = 0.0;
		husky_commands.publish(velocity_commands);

	}

    bool drive_forwards(){
		ROS_INFO("STARTED DRIVING FORWARDS");
		geometry_msgs::Pose cube_position=get_pose.getTarget();

		stop_husky();

		while(fabsf(cube_position.position.y)>.11){
		  cube_position=get_pose.getTarget();
		  ros::Rate rate(1);//publish at 1 Hz
		  velocity_commands.angular.x = 0.0;
          velocity_commands.angular.y = 0.0;
          velocity_commands.angular.z = cube_position.position.y*1.15;
		  husky_commands.publish(velocity_commands);
		}

		stop_husky();

		while(cube_position.position.x>0.85) {
		  cube_position=get_pose.getTarget();
		  ros::Rate rate(1);//publish at 1 Hz
		  velocity_commands.linear.x=cube_position.position.x*0.1;
		  husky_commands.publish(velocity_commands);
		}

		stop_husky();

		sleep(5.0);


    }
    
	bool drive_backwards(){
		ROS_INFO("STARTED DRIVING BACKWARDS");
		
		int i=0;

		while(i<8) {
		velocity_commands.linear.x = -0.5;
        velocity_commands.linear.y = 0.0;
        velocity_commands.linear.z = 0.0;

        velocity_commands.angular.x = 0.0;
        velocity_commands.angular.y = 0.0;
        velocity_commands.angular.z = 0.0;
		husky_commands.publish(velocity_commands);
		i++;
		sleep(1);
		}

		velocity_commands.linear.x = 0.0;
        velocity_commands.linear.y = 0.0;
        velocity_commands.linear.z = 0.0;

        velocity_commands.angular.x = 0.0;
        velocity_commands.angular.y = 0.0;
        velocity_commands.angular.z = 0.0;
		husky_commands.publish(velocity_commands);


    } 

  private:
    ros::NodeHandle nh;
    ros::Publisher husky_commands;
    float x;
	Subscribe get_pose;
	geometry_msgs::Twist velocity_commands;
	};

int main(int argc, char **argv)
{
	//delcarations

	ros::init(argc, argv, "husky_abb_grab_object");
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
	arm.setGoalTolerance(0.01);
	arm.setPlanningTime(15.0);
	//hand.setPlannerId("LBKPIECEkConfigDefault");
	hand.setPlannerId("RRTConnectkConfigDefault");
	//can be modified as desired
	hand.setGoalJointTolerance(0.1);
	hand.setPlanningTime(5.0);
	//arm.setPlanningTime(10.0);
	//arm.setPlanningTime(15.0);

	command_vel HUSKY;
	HUSKY.start_publisher();

	// end of declarations

	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true); // to display path in Rviz
	moveit_msgs::DisplayTrajectory display_trajectory;

	ROS_INFO("Reference frame: %s", arm.getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", arm.getEndEffectorLink().c_str());

	temp_pose = (arm.getCurrentPose(arm.getEndEffectorLink().c_str())).pose;
	
	ROS_INFO("Pose:HOME");
	arm.setNamedTarget("home");
	arm.plan(my_plan); // check if plan succeded
	arm.move();
	
	/*ROS_INFO("Pose: x to .1");
	temp_pose = (arm.getCurrentPose(arm.getEndEffectorLink().c_str())).pose;
	temp_pose.position.x += .1;
	arm.setPoseTarget(temp_pose);
	arm.plan(my_plan);
	arm.move();*/

	/*ROS_INFO("Pose:HOME");
	arm.setNamedTarget("home");
	arm.plan(my_plan); // check if plan succeded
	arm.move();*/

	hand.setJointValueTarget("finger_2_med_joint", 0);
	hand.setJointValueTarget("finger_1_med_joint", 0);
	hand.setJointValueTarget("finger_3_med_joint", 0);
	hand.plan(my_plan);
	hand.move();
	ROS_INFO("2");
	sleep(4.0);



	ros::ServiceServer service = n.advertiseService("grasp", &Subscribe::serviceCallback, &get_pose);
	ROS_INFO("3");
	while (ros::ok) // keep on running until stoped
	{
		ROS_INFO("4");
		get_pose.setFalse();
		while (!get_pose.getFlag()){ 
		}
		//Moving towards the cube
		HUSKY.drive_forwards();
		ROS_INFO("starting motion plan");
		temp_pose = get_pose.getTarget(); //set the desired pose to the position of the cube
		temp_pose.position.z += 0.183;	  //offset by .19 meters in the z axis
		temp_pose.position.x -= 0.1;
		temp_pose.orientation.w= 0.819;
		temp_pose.orientation.x= 0.0;
		temp_pose.orientation.y= 0.574;
		temp_pose.orientation.z= 0.0;   // and pointed straight down

		ROS_INFO("5");
		arm.setPoseTarget(temp_pose, arm.getEndEffectorLink().c_str());
		ROS_INFO("6");
		arm.plan(my_plan); // check if plan succeded
		ROS_INFO("7");
		arm.move();
		ROS_INFO("8");

		hand.setJointValueTarget("finger_2_med_joint",1.5);//closing hand
		hand.setJointValueTarget("finger_1_med_joint",1.5);
		hand.setJointValueTarget("finger_3_med_joint",1.5);
		hand.plan(my_plan);
		hand.move();
		
		sleep(8);
		ROS_INFO("9");
		
		temp_pose.position.z += 0.13;//raising cube in the air
		arm.setPoseTarget(temp_pose, arm.getEndEffectorLink().c_str());
		arm.plan(my_plan); // check if plan succeded
		arm.move();
		

		arm.setJointValueTarget("abb2_joint_1",-2.88);//moving hand to the back of the husky		
		arm.setJointValueTarget("abb2_joint_2",0.54);	
		arm.setJointValueTarget("abb2_joint_3",-0.51);	
		arm.setJointValueTarget("abb2_joint_4",-0.17);	
		arm.setJointValueTarget("abb2_joint_5",1.34);	
		arm.setJointValueTarget("abb2_joint_6",-6.70);
		arm.plan(my_plan);
		arm.move();
			
		
		hand.setJointValueTarget("finger_2_med_joint", 0);//opening hand
		hand.setJointValueTarget("finger_1_med_joint", 0);
		hand.setJointValueTarget("finger_3_med_joint", 0);
		hand.plan(my_plan);
		hand.move();
		sleep(12);
		arm.setNamedTarget("home");// This is needed so that the robot arm will not block the LIDAR
		arm.plan(my_plan);
		arm.move();				

		ROS_INFO("finished motion plan");
		HUSKY.drive_backwards();

		
		get_pose.setSuccess(true);
		

		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}	