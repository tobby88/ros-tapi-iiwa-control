/******************************************************************************
*  Copyright (C) 2016 by Tobias Holst                                         *
*                                                                             *
*  This file is part of tapi_iiwa_control.                                    *
*                                                                             *
*  tapi_iiwa_control is free software: you can redistribute it and/or modify  *
*  it under the terms of the GNU General Public License as published by       *
*  the Free Software Foundation, either version 3 of the License, or          *
*  (at your option) any later version.                                        *
*                                                                             *
*  tapi_iiwa_control is distributed in the hope that it will be useful,       *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of             *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
*  GNU General Public License for more details.                               *
*                                                                             *
*  You should have received a copy of the GNU General Public License          *
*  along with tapi_iiwa_control.  If not, see <http://www.gnu.org/licenses/>. *
*                                                                             *
*  Diese Datei ist Teil von tapi_iiwa_control.                                *
*                                                                             *
*  tapi_iiwa_control ist Freie Software: Sie können es unter den Bedingungen  *
*  der GNU General Public License, wie von der Free Software Foundation,      *
*  Version 3 der Lizenz oder (nach Ihrer Wahl) jeder späteren                 *
*  veröffentlichten Version, weiterverbreiten und/oder modifizieren.          *
*                                                                             *
*  tapi_iiwa_control wird in der Hoffnung, dass es nützlich sein wird, aber   *
*  OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite         *
*  Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK  *
*  Siehe die GNU General Public License für weitere Details.                  *
*                                                                             *
*  Sie sollten eine Kopie der GNU General Public License zusammen mit diesem  *
*  Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>. *
*******************************************************************************/

#ifndef TAPI_IIWA_CONTROL_H
#define TAPI_IIWA_CONTROL_H

#include <thread>
#include "geometry_msgs/PoseStamped.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Header.h"
#include "tapi_lib/tapi_lib.hpp"
#include "tf/tf.h"

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
  void activate();
  void gotAngularX(const std_msgs::Float64::ConstPtr& msg);
  void gotAngularY(const std_msgs::Float64::ConstPtr& msg);
  void gotAngularZ(const std_msgs::Float64::ConstPtr& msg);
  void gotCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void gotJoy(const sensor_msgs::Joy::ConstPtr& msg);
  void gotLinearX(const std_msgs::Float64::ConstPtr& msg);
  void gotLinearY(const std_msgs::Float64::ConstPtr& msg);
  void gotLinearZ(const std_msgs::Float64::ConstPtr& msg);
  void sendData();

  // Private member variables
  std::thread* activateThread;
  double angular[3];
  double* coefficients[6];
  double currentPosition[3];
  double currentRotEuler[3];
  tf::Quaternion currentRotQuat;
  bool gotData;
  std_msgs::Header header;
  ros::ServiceClient** iiwaModeClient;
  ros::Publisher* iiwaPub;
  double linear[3];
  ros::NodeHandle* nh;
  ros::Publisher* posePub[6];
  std::thread* sendThread;
  bool synchronized;
  Tapi::ServiceClient* tclient;
  Tapi::Publisher* tpub;
  Tapi::Subscriber* tsub;
};
}

#endif  // TAPI_IIWA_CONTROL_H
