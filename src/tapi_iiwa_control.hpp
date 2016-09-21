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
 * \file tapi_iiwa_control.hpp
 * \ingroup tapi_iiwa_control
 * \author Tobias Holst
 * \date 08 Sep 2016
 * \brief Declaration of the Tapi::iiwaControl-class and definition of its member variables
 */

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
/*!
 * \brief Connect to tapi_iiwa, generate absolute position messages with a relative control device (e.g. Spacenav) and
 * control a Kuka iiwa with it
 *
 * It uses open loop control, because closed loop control has latency issues and needs a prediction of the actual
 * current position
 * \author Tobias Holst
 * \version 1.0.1
 */
class iiwaControl
{
public:
  // Constructor/Desctructor

  /*!
   * \brief Create an iiwaControl object to control Kuka iiwa
   * \param nh Pointer to a \c ros::NodeHandle created outside of this class
   */
  iiwaControl(ros::NodeHandle* nh);

  /*!
    * \brief Stop publisher/subscriber/services and free memory
    */
  ~iiwaControl();

private:
  // Private member functions

  /*!
   * \brief Do robot movement
   *
   * This function runs in a thread and calls a service at tapi_iiwa. It's a while loop which runs - when started - as
   * long as \c ros::ok() is \c true. Calls the service every 50 ms (20 Hz) with the string \c "MoveToPose;rob;" which
   * will tell tapi_iiwa to start the movement to the coordinates published on the "Goal Pose to Robot" feature/topic.
   * \see Tapi::iiwaControl::activateThread
   * \see Tapi::iiwaControl::sendData
   */
  void activate();

  /*!
   * \brief Called by a ros callback when a new rotation of the x-axis has been got. Multiplies its value with its
   * corresponding coefficient and saves it to angular[0]
   * \param msg The message waiting in the ros message queue where the value of angular x is stored
   * \see Tapi::iiwaControl::angular
   * \see Tapi::iiwaControl::coefficient
   */
  void gotAngularX(const std_msgs::Float64::ConstPtr& msg);

  /*!
   * \brief Called by a ros callback when a new rotation of the y-axis has been got. Multiplies its value with its
   * corresponding coefficient and saves it to angular[1]
   * \param msg The message waiting in the ros message queue where the value of angular y is stored
   * \see Tapi::iiwaControl::angular
   * \see Tapi::iiwaControl::coefficient
   */
  void gotAngularY(const std_msgs::Float64::ConstPtr& msg);

  /*!
   * \brief Called by a ros callback when a new rotation of the z-axis has been got. Multiplies its value with its
   * corresponding coefficient and saves it to angular[2]
   * \param msg The message waiting in the ros message queue where the value of angular z is stored
   * \see Tapi::iiwaControl::angular
   * \see Tapi::iiwaControl::coefficient
   */
  void gotAngularZ(const std_msgs::Float64::ConstPtr& msg);

  /*!
   * \brief Called by a ros callback when a new value of the current robot position has been got.
   *
   * \warning Because of latency issues this position may be a little bit behind the actual current position of the
   * robot. This can be a big issue in closed loop control!
   * \param msg The message waiting in the ros message queue where the position of the robot is stored
   * \see Tapi::iiwaControl::currentPosition
   * \see Tapi::iiwaControl::currentRotQuat
   * \see Tapi::iiwaControl::currentRotEuler
   */
  void gotCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

  /*!
   * \brief Called by a ros callback when a new Joy message has arrived with values for all axes. Multiplies its value
   * with its corresponding coefficients and saves it to the angular and linear arrays.
   * \param msg The message waiting in the ros message queue where the value of all axes (at least six) are stored.
   * \see Tapi::iiwaControl::angular
   * \see Tapi::iiwaControl::linear
   * \see Tapi::iiwaControl::coefficient
   */
  void gotJoy(const sensor_msgs::Joy::ConstPtr& msg);

  /*!
   * \brief Called by a ros callback when a new linear movement of the x-axis has been got. Multiplies its value with
   * its corresponding coefficient and saves it to linear[0]
   * \param msg The message waiting in the ros message queue where the value of the linear x movement is stored
   * \see Tapi::iiwaControl::linear
   * \see Tapi::iiwaControl::coefficient
   */
  void gotLinearX(const std_msgs::Float64::ConstPtr& msg);

  /*!
   * \brief Called by a ros callback when a new linear movement of the y-axis has been got. Multiplies its value with
   * its corresponding coefficient and saves it to linear[1]
   * \param msg The message waiting in the ros message queue where the value of the linear y movement is stored
   * \see Tapi::iiwaControl::linear
   * \see Tapi::iiwaControl::coefficient
   */
  void gotLinearY(const std_msgs::Float64::ConstPtr& msg);

  /*!
   * \brief Called by a ros callback when a new linear movement of the z-axis has been got. Multiplies its value with
   * its corresponding coefficient and saves it to linear[2]
   * \param msg The message waiting in the ros message queue where the value of the linear z movement is stored
   * \see Tapi::iiwaControl::linear
   * \see Tapi::iiwaControl::coefficient
   */
  void gotLinearZ(const std_msgs::Float64::ConstPtr& msg);

  /*!
   * \brief Send the goal position to tapi_iiwa (which sends it to the robot every time its service is called). Runs in
   * its own thread every 50 ms (20 Hz) while \c ros::ok() is \c true
   * \see Tapi::iiwaControl::activate
   * \see Tapi::iiwaControl::sendThread
   */
  void sendData();

  // Private member variables

  /*!
   * \brief Runs the \c activate function
   * \see Tapi::iiwaControl::activate
   */
  std::thread* activateThread;

  /*!
   * \brief Stores the relative control values for rotations
   * \see Tapi::iiwaControl::gotAngularX
   * \see Tapi::iiwaControl::gotAngularY
   * \see Tapi::iiwaControl::gotAngularZ
   * \see Tapi::iiwaControl::gotLinearX
   * \see Tapi::iiwaControl::gotLinearY
   * \see Tapi::iiwaControl::gotLinearZ
   * \see Tapi::iiwaControl::gotJoy
   */
  double angular[3];

  /*!
   * \brief Stores pointers to the coefficients, so angular/linear values can be mulitplied with it (except when got by
   * \c gotJoy)
   */
  double* coefficients[6];

  /*!
   * \brief Goal position of the robot
   * \see Tapi::iiwaControl::sendData
   */
  double controlPosition[3];

  /*!
   * \brief Goal rotation (euler angles in radian) of the robot
   * \see Tapi::iiwaControl::sendData
   */
  double controlRotEuler[3];

  /*!
   * \brief Current position of the robot
   * \see Tapi::iiwaControl::gotCurrentPose
   */
  double currentPosition[3];

  /*!
   * \brief Current rotation (euler angles in radian) of the robot
   * \see Tapi::iiwaControl::gotCurrentPose
   */
  double currentRotEuler[3];

  /*!
   * \brief Current rotation of the robot (quaternion)
   * \see Tapi::iiwaControl::gotCurrentPose
   */
  tf::Quaternion currentRotQuat;

  /*!
   * \brief Wait for the first absoulte position of the robot, so the goal positions can be calculated with the relative
   * control data
   * \see Tapi::iiwaControl::gotCurrentPose
   */
  bool firstPose;

  /*!
   * \brief Header for the goal position messages
   * \see Tapi::iiwaControl::sendData
   */
  std_msgs::Header header;

  /*!
   * \brief Doublepointer to the ServiceClient to call the service on tapi_iiwa
   * \see Tapi::iiwaControl::activate
   */
  ros::ServiceClient** iiwaModeClient;

  /*!
   * \brief Pointer to a publisher to publish the geometry messages for the iiwa (to tapi_iiwa)
   * \see Tapi::iiwaControl::sendData
   */
  ros::Publisher* iiwaPub;

  /*!
   * \brief Stores the relative control values for linear movements
   * \see Tapi::iiwaControl::gotAngularX
   * \see Tapi::iiwaControl::gotAngularY
   * \see Tapi::iiwaControl::gotAngularZ
   * \see Tapi::iiwaControl::gotLinearX
   * \see Tapi::iiwaControl::gotLinearY
   * \see Tapi::iiwaControl::gotLinearZ
   * \see Tapi::iiwaControl::gotJoy
   */
  double linear[3];

  //! NodeHandle-pointer necessary to create subscribers, publishers and services.
  ros::NodeHandle* nh;

  /*!
   * \brief Pointer to publishers to publish the current robot position
   * \see Tapi::iiwaControl::gotCurrentPose
   */
  ros::Publisher* posePub[6];

  /*!
   * \brief Thread to run \c sendData function
   * \see Tapi::iiwaControl::sendData
   */
  std::thread* sendThread;

  //! \c tapi_lib based ServiceClient object to create Tapi compliant ServiceClients
  Tapi::ServiceClient* tclient;

  //! \c tapi_lib based Publisher object to create Tapi compliant Publishers
  Tapi::Publisher* tpub;

  //! \c tapi_lib based Subscriber object to create Tapi compliant Subscribers
  Tapi::Subscriber* tsub;
};
}

#endif  // TAPI_IIWA_CONTROL_H
