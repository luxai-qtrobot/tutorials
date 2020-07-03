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


#ifndef QT_ROBOT_APP_BASE
#define QT_ROBOT_APP_BASE

#include "ros/ros.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include "std_msgs/String.h"

#include "qt_idle_app/suspend.h"

class QTRobotAppBase {
public:
    QTRobotAppBase(ros::NodeHandle& node, const std::string servicePrefixName) {
      serviceSuspend = node.advertiseService(servicePrefixName+"/suspend", &QTRobotAppBase::suspendCB, this);
    }

    virtual ~QTRobotAppBase() { }

    bool suspendCB(qt_idle_app::suspend::Request  &req,
                   qt_idle_app::suspend::Response &res) {
                        res.status = suspend(req.flag);
                        return true;
                      }
protected:
    virtual bool suspend(bool flag) { };

private:
    ros::ServiceServer serviceSuspend;
};


#endif //QT_ROBOT_APP_BASE
