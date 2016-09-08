#ifndef TAPI_IIWA_CONTROL_H
#define TAPI_IIWA_CONTROL_H

#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"
#include "tapi_lib/tapi_lib.hpp"

namespace Tapi
{
class iiwaControl
{
public:
  // Constructor/Desctructor
  iiwaControl(ros::NodeHandle *nh);
  ~iiwaControl();

private:
  // Private member variables
  ros::ServiceClient **iiwaModeClient;
  ros::Publisher *iiwaPub;
  ros::NodeHandle *nh;
  Tapi::ServiceClient *tclient;
  Tapi::Publisher *tpub;
  Tapi::Subscriber *tsub;
};
}

#endif  // TAPI_IIWA_CONTROL_H
