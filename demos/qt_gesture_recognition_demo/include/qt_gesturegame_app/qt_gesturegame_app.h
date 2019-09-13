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


#ifndef qt_gesturegame_app
#define qt_gesturegame_app


#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

#include "qt_gesturegame_app/qt_service_helper.h"

// temporaty include
//#include "/home/apaikan/catkin_ws/devel/include/qt_nuitrack_app/Gestures.h"
#include "qt_nuitrack_app/Gestures.h"

//#include "qt_gesturegame_app/suspend.h"

#include <rfsm.h>


typedef std::map<std::string, std::string> ResultType;

class QTGestureGameApp {
public:
    QTGestureGameApp(ros::NodeHandle& nh);

    virtual ~QTGestureGameApp();
    /*
    bool suspendCB(qt_gesturegame_app::suspend::Request  &req,
                   qt_gesturegame_app::suspend::Response &res) {
                        res.status = suspend(req.flag);
                        return true;
                      }*/


    void gestureSubCB(const qt_nuitrack_app::Gestures::ConstPtr& gestures);

protected:
    virtual bool suspend(bool flag);
    void fsmTimerCallback(const ros::TimerEvent& event);
    //void nuitrackTimerCallback(const ros::TimerEvent& event);
    //void onNewGestures(const tdv::nuitrack::GestureData::Ptr gesture_data);
    //std::string type2string( const tdv::nuitrack::GestureType gesture_type );

private:
    QTrobotServiceHelper serviceHelper;
    rfsm::StateMachine rfsm;
    boost::mutex mutexRFSM;
    ros::ServiceServer serviceSuspend;
    ros::Subscriber subGestures;

    // rfsm callbacks
    rfsm::StateCallback* waitingStateCB;
    rfsm::StateCallback* playStateCB;
    ros::Timer fsmTimer;

};


#endif //qt_gesturegame_app
