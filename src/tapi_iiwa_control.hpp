#ifndef TAPI_IIWA_CONTROL_H
#define TAPI_IIWA_CONTROL_H

#include "ros/node_handle.h"

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
  ros::NodeHandle *nh;
};
}

#endif  // TAPI_IIWA_CONTROL_H
