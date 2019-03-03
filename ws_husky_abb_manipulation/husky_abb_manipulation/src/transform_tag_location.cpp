#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>




int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_tag_location");
	geometry_msgs::Pose msg;
	geometry_msgs::TransformStamped tf_msg;
    ros::NodeHandle node;
	ros::Publisher pub = node.advertise<geometry_msgs::Pose>("tag_pose", 1000);
    tf::TransformListener listener;
    tf::StampedTransform transform;
    while (ros::ok())
    {
        try
        {

            listener.lookupTransform("*****replace planning frame******", "/tag_link", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
		transformStampedTFToMsg(transform,tf_msg);
        
		pub.publish(msg);
        ros::spinOnce();
    }
    return 0;
}