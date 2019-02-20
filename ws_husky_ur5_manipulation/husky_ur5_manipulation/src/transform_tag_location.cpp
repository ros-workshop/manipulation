#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>




int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_tag_location");
	geometry_msgs::TransformStamped tf_msg;
    ros::NodeHandle node;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    while (ros::ok())
    {
        try
        {

            listener.lookupTransform(" *****planning link**** ", "/tag_link", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
		transformStampedTFToMsg(transform,tf_msg);
        
        ros::spinOnce();
    }
    return 0;
}