#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>


/****************************

Change the frame names
and publish a geometry pose on the topic "/tag_pose"

*****************************/



int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_tag_location");
	geometry_msgs::Pose msg;
	geometry_msgs::TransformStamped tf_msg;
    ros::NodeHandle node;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    while (ros::ok())
    {
        try
        {
            tf_msg = tfBuffer.lookupTransform("frame", "frame", ros::Time(0), ros::Duration(10.0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        
        msg.position.x=tf_msg.transform.translation.x;
        msg.position.y=tf_msg.transform.translation.y;
        msg.position.z=tf_msg.transform.translation.z;
        msg.orientation.x=tf_msg.transform.rotation.x;
        msg.orientation.y=tf_msg.transform.rotation.y;
        msg.orientation.z=tf_msg.transform.rotation.z;
        msg.orientation.w=tf_msg.transform.rotation.w;
        
        ros::spinOnce();
    }
    return 0;
}

