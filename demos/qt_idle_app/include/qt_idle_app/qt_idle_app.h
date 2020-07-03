/*
 * Copyright (C) 2016 Lux Future Robotic
 * Author:  Ali Paikan
 * email:   ali.paikan@luxai.com
 * website: www.luxai.com
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */


#ifndef QT_IDLE_APP
#define QT_IDLE_APP

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>

#include <rfsm.h>
#include "qt_idle_app/qt_robot_app_base.h"

class QTIdleApp : protected QTRobotAppBase{
public:
    QTIdleApp(ros::NodeHandle& nh);

    virtual ~QTIdleApp();

protected:
    virtual bool suspend(bool flag);
    void timerCallback(const ros::TimerEvent& event);

private:
    rfsm::StateMachine rfsm;
    boost::mutex mutexRFSM;
    rfsm::StateCallback* lookAroundStateCB;
    rfsm::StateCallback* boredStateCB;
    ros::Timer timer;

};


#endif //QT_IDLE_APP
