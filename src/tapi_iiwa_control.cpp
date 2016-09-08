#include "tapi_iiwa_control.hpp"
#include <algorithm>
#include "geometry_msgs/PoseStamped.h"
#include "tapi_iiwa/OpenIGTLStateService.h"

namespace Tapi
{
iiwaControl::iiwaControl(ros::NodeHandle* nh) : nh(nh)
{
  tclient = new Tapi::ServiceClient(nh, "iiwa Controller");
  tpub = new Tapi::Publisher(nh, "iiwa Controller");
  tsub = new Tapi::Subscriber(nh, "iiwa Controller");
  iiwaModeClient = tclient->AddFeature<tapi_iiwa::OpenIGTLStateService>("Mode");
  iiwaPub = tpub->AddFeature<geometry_msgs::PoseStamped>("Goal Pose", 1);
}
iiwaControl::~iiwaControl()
{
  delete tclient;
  delete tpub;
  delete tsub;
}
}
