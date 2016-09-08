#include "ros/node_handle.h"
#include "tapi_iiwa_control.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Tapi_iiwa_Control");
  ros::NodeHandle nh;
  Tapi::iiwaControl iiwa(&nh);
  while (ros::ok())
  {
    ros::spin();
  }

  return 0;
}
