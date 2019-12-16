/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  Copyright (c) 2014, Fraunhofer IPA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef motors_custom_controller_H
#define motors_custom_controller_H

#include <map>
//#include <boost/thread/mutex.hpp>
//#include <boost/thread/thread.hpp>

#include <std_msgs/String.h>
#include <controller_interface/controller.h>
#include <qt_motor/qt_motor.h>

#include <motors_custom_controller/StartStop.h>

namespace motors_custom_controller
{

  typedef std::map<std::string, std::vector<double> > JointsPosition;
  typedef std::map<std::string, std::vector<double> >::iterator JointsPosition_itr;

/*
  struct RecordData {
      std::vector<std::string> parts;
      std::vector<ros::Time> stamps;
      JointsPosition positions;
  };
*/

  class QTCustomController : public controller_interface::Controller<QTMotorInterface>
  {
  public:
      /**
       * @brief QTCustomController
       */
      QTCustomController();

      /**
       * @brief ~QTCustomController
       */
      ~QTCustomController();

    bool init(QTMotorInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

    void update(const ros::Time& time, const ros::Duration& period);

    void starting(const ros::Time& time) { ROS_INFO("QTCustomController: starting"); }
    void stopping(const ros::Time& time) { ROS_INFO("QTCustomController: stoping");}

    void changeMainLoopFreq(double freq);

  private:
    bool startStopCB(motors_custom_controller::StartStop::Request  &req,
                            motors_custom_controller::StartStop::Response &res);

    void generateMyTrajectory(void);

  private:
    QTMotorInterface *hw;
    std::vector<hardware_interface::JointStateHandle> jointStateLarm;
    std::vector<hardware_interface::JointStateHandle> jointStateRarm;
    std::vector<hardware_interface::JointStateHandle> jointStateHead;

    ros::ServiceServer serviceStartStop;
    boost::mutex mutexUpdate;
    bool shouldPlay;

    std::vector<double> my_RightShoulderPitch_cmd;
    std::vector<double> my_LeftShoulderPitch_cmd;

  };

}

#endif
