#include "tapi_iiwa_control.hpp"
#include <algorithm>
#include <chrono>
#include "tapi_iiwa/OpenIGTLStateService.h"

#define PI 3.14159265358979323846

using namespace std;

namespace Tapi
{
// Constructor/Destructor
iiwaControl::iiwaControl(ros::NodeHandle* nh) : nh(nh)
{
  std::fill(angular, angular + sizeof(angular), 0.0);
  std::fill(linear, linear + sizeof(linear), 0.0);
  gotData = false;
  synchronized = false;
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
  activateThread = new thread(&iiwaControl::activate, this);
  sendThread = new thread(&iiwaControl::sendData, this);
}

iiwaControl::~iiwaControl()
{
  delete tclient;
  delete tpub;
  delete tsub;
  activateThread->join();
  delete activateThread;
  sendThread->join();
  delete sendThread;
}

// Private member functions
void iiwaControl::activate()
{
  while (ros::ok())
  {
    this_thread::sleep_for(chrono::milliseconds(1000));
    if (*iiwaModeClient)
    {
      tapi_iiwa::OpenIGTLStateService msg;
      msg.request.state = "MoveToPose;rob;";
      if (!(*iiwaModeClient)->call(msg) || !msg.response.alive)
        ROS_ERROR("Error when connecting to iiwa or iiwa node");
    }
  }
}

void iiwaControl::gotAngularX(const std_msgs::Float64::ConstPtr& msg)
{
  angular[0] = msg->data * *coefficients[0];
  gotData = true;
}

void iiwaControl::gotAngularY(const std_msgs::Float64::ConstPtr& msg)
{
  angular[1] = msg->data * *coefficients[1];
  gotData = true;
}

void iiwaControl::gotAngularZ(const std_msgs::Float64::ConstPtr& msg)
{
  angular[2] = msg->data * *coefficients[2];
  gotData = true;
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
  if (!synchronized)
    synchronized = true;
  std_msgs::Float64 pubmsg[6];
  pubmsg[0].data = currentPosition[0];
  pubmsg[1].data = currentPosition[1];
  pubmsg[2].data = currentPosition[2];
  pubmsg[3].data = currentRotEuler[0];
  pubmsg[4].data = currentRotEuler[1];
  pubmsg[5].data = currentRotEuler[2];
  for (int i = 0; i < 6; i++)
    posePub[i]->publish(pubmsg[i]);
  currentRotEuler[0] *= 180 / PI;
  currentRotEuler[1] *= 180 / PI;
  currentRotEuler[2] *= 180 / PI;
}

void iiwaControl::gotJoy(const sensor_msgs::Joy::ConstPtr& msg)
{
  if (msg->axes.size() < 6)
    return;
  angular[0] = msg->axes[0];
  angular[1] = msg->axes[1];
  angular[2] = msg->axes[2];
  linear[0] = msg->axes[3];
  linear[1] = msg->axes[4];
  linear[2] = msg->axes[5];
  gotData = true;
}

void iiwaControl::gotLinearX(const std_msgs::Float64::ConstPtr& msg)
{
  linear[0] = msg->data * *coefficients[3];
  gotData = true;
}

void iiwaControl::gotLinearY(const std_msgs::Float64::ConstPtr& msg)
{
  linear[1] = msg->data * *coefficients[4];
  gotData = true;
}

void iiwaControl::gotLinearZ(const std_msgs::Float64::ConstPtr& msg)
{
  linear[2] = msg->data * *coefficients[5];
  gotData = true;
}

void iiwaControl::sendData()
{
  unsigned int runsPerSecond = 10;
  unsigned int waitTime = 1000 / runsPerSecond;
  double speed_cm_per_s = 10.0;
  double speedFactor = speed_cm_per_s / 100 / runsPerSecond;
  double degree_per_s = 45.0;
  double angleFactor = degree_per_s / runsPerSecond;
  while (ros::ok())
  {
    this_thread::sleep_for(chrono::milliseconds(waitTime));
    if (gotData && synchronized)
    {
      double goalPosition[3];
      double goalRotEuler[3];
      goalPosition[0] = currentPosition[0] + linear[0] * speedFactor;
      goalPosition[1] = currentPosition[1] + linear[1] * speedFactor;
      goalPosition[2] = currentPosition[2] + linear[2] * speedFactor;
      goalRotEuler[0] = (currentRotEuler[0] + angular[0] * angleFactor) * PI / 180;
      goalRotEuler[1] = (currentRotEuler[1] + angular[1] * angleFactor) * PI / 180;
      goalRotEuler[2] = (currentRotEuler[2] + angular[2] * angleFactor) * PI / 180;
      header.seq++;
      header.stamp = ros::Time::now();
      geometry_msgs::PoseStamped msg;
      msg.header = header;
      msg.pose.position.x = goalPosition[0];
      msg.pose.position.y = goalPosition[1];
      msg.pose.position.z = goalPosition[2];
      tf::Quaternion quat = tf::createQuaternionFromRPY(goalRotEuler[0], goalRotEuler[1], goalRotEuler[2]);
      msg.pose.orientation.w = quat.getW();
      msg.pose.orientation.x = quat.getX();
      msg.pose.orientation.y = quat.getY();
      msg.pose.orientation.z = quat.getZ();
      gotData = false;
      iiwaPub->publish(msg);
    }
  }
}
}
