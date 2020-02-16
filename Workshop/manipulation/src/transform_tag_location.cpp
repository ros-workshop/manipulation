#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
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

    tf::TransformListener listener;
    tf::StampedTransform transform;
    while (ros::ok())
    {
        try
        {
            listener.waitForTransform("/frame_name", "/frame_name", ros::Time(0), ros::Duration(9.0));
            listener.lookupTransform("/frame_name", "/frame_name", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        
        transformStampedTFToMsg(transform,tf_msg);
		
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
