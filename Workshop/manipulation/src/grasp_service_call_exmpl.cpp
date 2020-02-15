#include "ros/ros.h"
#include <cstdlib>
#include <std_srvs/SetBool.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_service_call_exmpl");
  

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("grasp");
  std_srvs::SetBool doesnt_matter;
  if (client.call(doesnt_matter))
  {
    ROS_INFO((doesnt_matter.response.message).c_str());
  }
  else
  {
    ROS_INFO("error with service call");
    
  }

  return 0;
}
