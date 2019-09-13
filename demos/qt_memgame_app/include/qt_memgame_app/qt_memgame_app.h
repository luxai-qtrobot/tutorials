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


#ifndef qt_memgame_app
#define qt_memgame_app


#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

#include "qt_memgame_app/qt_service_helper.h"
//#include "qt_memgame_app/suspend.h"

#include <rfsm.h>

typedef std::map<std::string, std::string> ResultType;

class QTMemGameApp {
public:
    QTMemGameApp(ros::NodeHandle& nh);

    virtual ~QTMemGameApp();
    /*
    bool suspendCB(qt_memgame_app::suspend::Request  &req,
                   qt_memgame_app::suspend::Response &res) {
                        res.status = suspend(req.flag);
                        return true;
                      }*/


    void objectsSubCB(const std_msgs::Float32MultiArray::ConstPtr& objects);

protected:
    virtual bool suspend(bool flag);
    void timerCallback(const ros::TimerEvent& event);

private:
    QTrobotServiceHelper serviceHelper;
    rfsm::StateMachine rfsm;
    boost::mutex mutexRFSM;
    ros::ServiceServer serviceSuspend;
    ros::Subscriber subObjects;

    // rfsm callbacks
    rfsm::StateCallback* waitingStateCB;
    rfsm::StateCallback* playStateCB;
    ros::Timer timer;

};


#endif //qt_memgame_app
