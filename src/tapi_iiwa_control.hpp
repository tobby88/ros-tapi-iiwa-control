#ifndef TAPI_IIWA_CONTROL_H
#define TAPI_IIWA_CONTROL_H

#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Header.h"
#include "tapi_lib/tapi_lib.hpp"

namespace Tapi
{
class iiwaControl
{
public:
  // Constructor/Desctructor
  iiwaControl(ros::NodeHandle* nh);
  ~iiwaControl();

private:
  // Private member functions
  void gotAngularX(const std_msgs::Float64::ConstPtr& msg);
  void gotAngularY(const std_msgs::Float64::ConstPtr& msg);
  void gotAngularZ(const std_msgs::Float64::ConstPtr& msg);
  void gotJoy(const sensor_msgs::Joy::ConstPtr& msg);
  void gotLinearX(const std_msgs::Float64::ConstPtr& msg);
  void gotLinearY(const std_msgs::Float64::ConstPtr& msg);
  void gotLinearZ(const std_msgs::Float64::ConstPtr& msg);

  // Private member variables
  double angular[3];
  double* coefficients[6];
  bool gotValues[6];
  std_msgs::Header header;
  ros::ServiceClient** iiwaModeClient;
  ros::Publisher* iiwaPub;
  double linear[3];
  ros::NodeHandle* nh;
  Tapi::ServiceClient* tclient;
  Tapi::Publisher* tpub;
  Tapi::Subscriber* tsub;
};
}

#endif  // TAPI_IIWA_CONTROL_H
