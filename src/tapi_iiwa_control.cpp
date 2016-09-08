#include "tapi_iiwa_control.hpp"
#include <algorithm>
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
  iiwaPub = tpub->AddFeature<geometry_msgs::PoseStamped>("Goal Pose to Robot", 1);
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
  tsub->AddFeature(SubscribeOptionsForTapi(geometry_msgs::PoseStamped, 1, &iiwaControl::gotCurrentPose),
                   "Get Current Pose from Robot");
  posePub[0] = tpub->AddFeature<std_msgs::Float64>("Current X-Position of Flange", 1);
  posePub[1] = tpub->AddFeature<std_msgs::Float64>("Current Y-Position of Flange", 1);
  posePub[2] = tpub->AddFeature<std_msgs::Float64>("Current Z-Position of Flange", 1);
  posePub[3] = tpub->AddFeature<std_msgs::Float64>("Current X-Rotation of Flange", 1);
  posePub[4] = tpub->AddFeature<std_msgs::Float64>("Current Y-Rotation of Flange", 1);
  posePub[5] = tpub->AddFeature<std_msgs::Float64>("Current Z-Rotation of Flange", 1);
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

void iiwaControl::gotCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  currentPosition[0] = msg->pose.position.x;
  currentPosition[1] = msg->pose.position.y;
  currentPosition[2] = msg->pose.position.z;
  currentRotQuat.setW(msg->pose.orientation.w);
  currentRotQuat.setX(msg->pose.orientation.x);
  currentRotQuat.setY(msg->pose.orientation.y);
  currentRotQuat.setZ(msg->pose.orientation.z);
  tf::Matrix3x3 temp(currentRotQuat);
  temp.getRPY(currentRotEuler[0], currentRotEuler[1], currentRotEuler[2]);
  std_msgs::Float64 msg[6];
  msg[0].data = currentPosition[0];
  msg[1].data = currentPosition[1];
  msg[2].data = currentPosition[2];
  msg[3].data = currentRotEuler[0];
  msg[4].data = currentRotEuler[1];
  msg[5].data = currentRotEuler[2];
  for (int i = 0; i < 6; i++)
    posePub[i]->publish(msg[i]);
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
