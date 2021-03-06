/******************************************************************************
 *  Copyright (C) 2016 by Tobias Holst                                        *
 *                                                                            *
 *  This file is part of tapi_iiwa_control.                                   *
 *                                                                            *
 *  tapi_iiwa_control is free software: you can redistribute it and/or modify *
 *  it under the terms of the GNU General Public License as published by      *
 *  the Free Software Foundation, either version 3 of the License, or         *
 *  (at your option) any later version.                                       *
 *                                                                            *
 *  tapi_iiwa_control is distributed in the hope that it will be useful,      *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of            *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
 *  GNU General Public License for more details.                              *
 *                                                                            *
 *  You should have received a copy of the GNU General Public License         *
 *  along with tapi_iiwa_control.  If not, see <http://www.gnu.org/licenses/>.*
 *                                                                            *
 *  Diese Datei ist Teil von tapi_iiwa_control.                               *
 *                                                                            *
 *  tapi_iiwa_control ist Freie Software: Sie können es unter den Bedingungen *
 *  der GNU General Public License, wie von der Free Software Foundation,     *
 *  Version 3 der Lizenz oder (nach Ihrer Wahl) jeder späteren                *
 *  veröffentlichten Version, weiterverbreiten und/oder modifizieren.         *
 *                                                                            *
 *  tapi_iiwa_control wird in der Hoffnung, dass es nützlich sein wird, aber  *
 *  OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite        *
 *  Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK *
 *  Siehe die GNU General Public License für weitere Details.                 *
 *                                                                            *
 *  Sie sollten eine Kopie der GNU General Public License zusammen mit diesem *
 *  Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.*
 ******************************************************************************/

/*!
 * \file tapi_iiwa_control.cpp
 * \ingroup tapi_iiwa_control
 * \author Tobias Holst
 * \date 08 Sep 2016
 * \brief Defintion of the Tapi::iiwaControl-class
 */

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
  // Set default values
  firstPose = true;
  std::fill(angular, angular + sizeof(angular), 0.0);
  std::fill(linear, linear + sizeof(linear), 0.0);

  // Create (Tapi compliant) subscribers, publishers and serviceclients
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

  // Start the thread to activate the robot
  activateThread = new thread(&iiwaControl::activate, this);
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
    this_thread::sleep_for(chrono::milliseconds(50));

    // Check whether the ServiceClient is connected
    if (*iiwaModeClient)
    {
      // Call the service with a specific message to start movement
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
}

void iiwaControl::gotAngularY(const std_msgs::Float64::ConstPtr& msg)
{
  angular[1] = msg->data * *coefficients[1];
}

void iiwaControl::gotAngularZ(const std_msgs::Float64::ConstPtr& msg)
{
  angular[2] = msg->data * *coefficients[2];
}

void iiwaControl::gotCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // Check if the pose is valid. If not: return
  if (msg->pose.position.x == 0 && msg->pose.position.y == 0 && msg->pose.position.z == 0 &&
      msg->pose.orientation.w == 0 && msg->pose.orientation.x == 0 && msg->pose.orientation.y == 0 &&
      msg->pose.orientation.z == 0)
    return;

  // Save the current position
  currentPosition[0] = msg->pose.position.x;
  currentPosition[1] = msg->pose.position.y;
  currentPosition[2] = msg->pose.position.z;
  currentRotQuat.setW(msg->pose.orientation.w);
  currentRotQuat.setX(msg->pose.orientation.x);
  currentRotQuat.setY(msg->pose.orientation.y);
  currentRotQuat.setZ(msg->pose.orientation.z);

  // Calculate the rotation matrice from the quaternion and then get the euler angles in radian
  tf::Matrix3x3 temp(currentRotQuat);
  temp.getRPY(currentRotEuler[0], currentRotEuler[1], currentRotEuler[2]);

  // Publish the current position
  std_msgs::Float64 pubmsg[6];
  pubmsg[0].data = currentPosition[0];
  pubmsg[1].data = currentPosition[1];
  pubmsg[2].data = currentPosition[2];
  pubmsg[3].data = currentRotEuler[0];
  pubmsg[4].data = currentRotEuler[1];
  pubmsg[5].data = currentRotEuler[2];
  for (int i = 0; i < 6; i++)
    posePub[i]->publish(pubmsg[i]);

  // If this is the first time we get a valid position we take this as the base (starting point) for "controlPosition"
  // to do an open loop control and start the sendData-thread
  if (firstPose)
  {
    controlPosition[0] = currentPosition[0];
    controlPosition[1] = currentPosition[1];
    controlPosition[2] = currentPosition[2];
    controlRotEuler[0] = currentRotEuler[0];
    controlRotEuler[1] = currentRotEuler[1];
    controlRotEuler[2] = currentRotEuler[2];
    firstPose = false;
    sendThread = new thread(&iiwaControl::sendData, this);
  }
}

void iiwaControl::gotJoy(const sensor_msgs::Joy::ConstPtr& msg)
{
  // Save the data in the right order (maybe only valid for spacenavs? Other devices better connect to the single
  // "channels"
  if (msg->axes.size() < 6)
    return;
  angular[0] = msg->axes[3];
  angular[1] = msg->axes[4];
  angular[2] = msg->axes[5];
  linear[0] = msg->axes[1];
  linear[1] = msg->axes[0];
  linear[2] = msg->axes[2];
}

void iiwaControl::gotLinearX(const std_msgs::Float64::ConstPtr& msg)
{
  linear[0] = msg->data * *coefficients[3];
}

void iiwaControl::gotLinearY(const std_msgs::Float64::ConstPtr& msg)
{
  linear[1] = msg->data * *coefficients[4];
}

void iiwaControl::gotLinearZ(const std_msgs::Float64::ConstPtr& msg)
{
  linear[2] = msg->data * *coefficients[5];
}

void iiwaControl::sendData()
{
  // Calculate the factors based on the given speeds
  unsigned int runsPerSecond = 20;
  unsigned int waitTime = 1000 / runsPerSecond;
  double speed_cm_per_s = 10.0;
  double speedFactor = speed_cm_per_s / 100 / runsPerSecond;
  double degree_per_s = 45.0 * PI / 180.0;
  double angleFactor = degree_per_s / runsPerSecond;

  while (ros::ok())
  {
    this_thread::sleep_for(chrono::milliseconds(waitTime));

    // Wait for the first valid current robot position to make relative movements to it
    if (!firstPose)
    {
      controlPosition[0] = controlPosition[0] + linear[0] * speedFactor;
      controlPosition[1] = controlPosition[1] + linear[1] * speedFactor;
      controlPosition[2] = controlPosition[2] + linear[2] * speedFactor;
      controlRotEuler[0] = controlRotEuler[0] + angular[0] * angleFactor;
      controlRotEuler[1] = controlRotEuler[1] + angular[1] * angleFactor;
      controlRotEuler[2] = controlRotEuler[2] + angular[2] * angleFactor;

      // Now generate a geometry message from the calculated positions
      header.seq++;
      header.stamp = ros::Time::now();
      geometry_msgs::PoseStamped msg;
      msg.header.seq = header.seq;
      msg.header.stamp = header.stamp;
      msg.pose.position.x = controlPosition[0];
      msg.pose.position.y = controlPosition[1];
      msg.pose.position.z = controlPosition[2];
      // We need quaternions for the robot position, so calculate them from the euler angles
      tf::Quaternion quat = tf::createQuaternionFromRPY(controlRotEuler[0], controlRotEuler[1], controlRotEuler[2]);
      msg.pose.orientation.w = quat.getW();
      msg.pose.orientation.x = quat.getX();
      msg.pose.orientation.y = quat.getY();
      msg.pose.orientation.z = quat.getZ();
      iiwaPub->publish(msg);
    }
  }
}
}
