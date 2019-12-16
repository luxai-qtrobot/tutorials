/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, LuxAI S.A.
 *  Author: Ali Paikan
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
