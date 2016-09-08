#include "tapi_iiwa_control.hpp"
#include <algorithm>
#include "geometry_msgs/PoseStamped.h"
#include "tapi_iiwa/OpenIGTLStateService.h"

namespace Tapi
{
// Constructor/Destructor
iiwaControl::iiwaControl(ros::NodeHandle* nh) : nh(nh)
{
  std::fill(gotValues, gotValues + sizeof(gotValues), false);
  tclient = new Tapi::ServiceClient(nh, "iiwa Controller");
  tpub = new Tapi::Publisher(nh, "iiwa Controller");
  tsub = new Tapi::Subscriber(nh, "iiwa Controller");
  iiwaModeClient = tclient->AddFeature<tapi_iiwa::OpenIGTLStateService>("Mode");
  iiwaPub = tpub->AddFeature<geometry_msgs::PoseStamped>("Goal Pose", 1);
  coefficients[0] =
      tsub->AddFeature(SubscribeOptionsForTapi(std_msgs::Float64, 1, &iiwaControl::gotAngularX), "Angular X");
  coefficients[1] =
      tsub->AddFeature(SubscribeOptionsForTapi(std_msgs::Float64, 1, &iiwaControl::gotAngularY), "Angular Y");
  coefficients[2] =
      tsub->AddFeature(SubscribeOptionsForTapi(std_msgs::Float64, 1, &iiwaControl::gotAngularZ), "Angular Z");
  coefficients[3] =
      tsub->AddFeature(SubscribeOptionsForTapi(std_msgs::Float64, 1, &iiwaControl::gotLinearX), "Linear X");
  coefficients[4] =
      tsub->AddFeature(SubscribeOptionsForTapi(std_msgs::Float64, 1, &iiwaControl::gotLinearY), "Linear Y");
  coefficients[5] =
      tsub->AddFeature(SubscribeOptionsForTapi(std_msgs::Float64, 1, &iiwaControl::gotLinearZ), "Linear Z");
  tsub->AddFeature(SubscribeOptionsForTapi(sensor_msgs::Joy, 1, &iiwaControl::gotJoy), "[Optional] Full Joy Message");
}

iiwaControl::~iiwaControl()
{
  delete tclient;
  delete tpub;
  delete tsub;
}

// Private member functions
void iiwaControl::gotAngularX(const std_msgs::Float64::ConstPtr& msg)
{
  angular[0] = msg->data * *coefficients[0];
  gotValues[0] = true;
}

void iiwaControl::gotAngularY(const std_msgs::Float64::ConstPtr& msg)
{
  angular[1] = msg->data * *coefficients[1];
  gotValues[1] = true;
}

void iiwaControl::gotAngularZ(const std_msgs::Float64::ConstPtr& msg)
{
  angular[2] = msg->data * *coefficients[2];
  gotValues[2] = true;
}

void iiwaControl::gotJoy(const sensor_msgs::Joy::ConstPtr& msg)
{
  angular[0] = msg->axes[0];
  angular[1] = msg->axes[1];
  angular[2] = msg->axes[2];
  linear[0] = msg->axes[3];
  linear[1] = msg->axes[4];
  linear[2] = msg->axes[5];
  std::fill(gotValues, gotValues + sizeof(gotValues), true);
}

void iiwaControl::gotLinearX(const std_msgs::Float64::ConstPtr& msg)
{
  linear[0] = msg->data * *coefficients[3];
  gotValues[3] = true;
}

void iiwaControl::gotLinearY(const std_msgs::Float64::ConstPtr& msg)
{
  linear[1] = msg->data * *coefficients[4];
  gotValues[4] = true;
}

void iiwaControl::gotLinearZ(const std_msgs::Float64::ConstPtr& msg)
{
  linear[2] = msg->data * *coefficients[5];
  gotValues[5] = true;
}
}
