#include <sstream>
#include <std_msgs/Float64MultiArray.h>

#include "qt_idle_app/qt_idle_app.h"
#include "qt_gesture_controller/gesture_play.h"
#include "qt_motors_controller/home.h"
#include <boost/thread/thread.hpp>

/**
 * @brief The LookAroundStateCB class
 */
class LookAroundStateCB : public rfsm::StateCallback {
public:
    LookAroundStateCB(ros::NodeHandle& nh, rfsm::StateMachine& fsm):
        rfsm(fsm), interrupted(false) {

        if(!nh.getParam("/qt_idle_app/idle_time_min", idle_tmin))
            idle_tmin = 120;
        if(!nh.getParam("/qt_idle_app/idle_time_max", idle_tmax))
            idle_tmax = 300;

        head_pub = nh.advertise<std_msgs::Float64MultiArray>("/qt_robot/head_position/command", 5);
        if(!head_pub)
            ROS_WARN_STREAM("cannot advertise to " << "/qt_robot/head_position/command");
    }

    virtual void doo() {
        interrupted = false;
        ros::Time start_time = ros::Time::now();
        ros::Time prev_pub_time = start_time;
        // look around for 10 sec
        int idle_time = rand() % (idle_tmax - idle_tmin) + idle_tmin;
        int pub_time = rand() % (10) + 5;
        while(((ros::Time::now() - start_time).toSec() < idle_time)
              && !ros::isShuttingDown()
              && !interrupted) {

            if(int((ros::Time::now() - prev_pub_time).toSec()) > pub_time) {
                std_msgs::Float64MultiArray pos;
                pos.data.clear();
                pos.data.push_back(rand()%30 - 15);
                pos.data.push_back(0);
                head_pub.publish(pos);
                prev_pub_time = ros::Time::now();
                pub_time = rand() % (10) + 5;
            }
            ros::Duration(1.0).sleep();
            ROS_INFO("idle");
        }

        // send some events to do other things such as bored.
        if(!ros::isShuttingDown() && !interrupted) {
            ROS_INFO("sending e_bored");
            rfsm.sendEvent("e_bored");
        }
    }

    void interrupt() {interrupted = true; }

private:
    bool interrupted;
    int idle_tmin;
    int idle_tmax;
    rfsm::StateMachine& rfsm;
    ros::Publisher head_pub;
};


class BoredStateCB : public rfsm::StateCallback {
public:
    BoredStateCB(ros::NodeHandle& nh, rfsm::StateMachine& fsm):
        rfsm(fsm){
        feeling.push_back("bored");
        feeling.push_back("sleepy");
	      feeling.push_back("sneezing");
	      feeling.push_back("hi");

        emotion_pub = nh.advertise<std_msgs::String>("/qt_robot/emotion/show", 5);
        if(!emotion_pub)
            ROS_WARN_STREAM("cannot advertise to " << "/qt_robot/emotion/show");


        gestureClient = nh.serviceClient<qt_gesture_controller::gesture_play>("/qt_robot/gesture/play");
        if(!gestureClient.exists()) {
            ROS_WARN("Could connect to /qt_robot/gesture/play service");
        }

        homeClient = nh.serviceClient<qt_motors_controller::home>("/qt_robot/motors/home");
        if(!homeClient.exists()) {
            ROS_WARN("Could connect to /qt_robot/motors/home service");
        }


    }

    void homeAll() {
      qt_motors_controller::home srvHome;
	    srvHome.request.parts.push_back("head");
	    srvHome.request.parts.push_back("right_arm");
	    srvHome.request.parts.push_back("left_arm");
	    if(!homeClient.call(srvHome)) {
            ROS_WARN("Could not call service home");
      }
    }

    virtual void entry() {
      homeAll();
    }

    virtual void doo() {
        ROS_INFO("in bored");
        qt_gesture_controller::gesture_play srvGesture;
        srvGesture.request.speed = 0;
	      std::string emotion;
        std::string cur_feeling = feeling[rand() % feeling.size()];
        if(cur_feeling == "bored") {
            emotion = "QT/confused";
	          threadEmotion = new boost::thread(boost::bind(&BoredStateCB::showEmotion, this, emotion, 1.0));
            srvGesture.request.name = "QT/bored";
        }
        else if(cur_feeling == "sleepy") {
            emotion = "QT/yawn";
            srvGesture.request.name = "QT/yawn";
            threadEmotion = new boost::thread(boost::bind(&BoredStateCB::showEmotion, this, emotion, 2.0));
        }
        else if(cur_feeling == "sneezing") {
            emotion = "QT/with_a_cold_sneezing";
            threadEmotion = new boost::thread(boost::bind(&BoredStateCB::showEmotion, this, emotion, 2.0));
            srvGesture.request.name = "QT/sneezing";
        }
        else if(cur_feeling == "hi") {
            emotion = "QT/happy";
            threadEmotion = new boost::thread(boost::bind(&BoredStateCB::showEmotion, this, emotion, 2.0));
            srvGesture.request.name = "QT/hi";
        }

        if(! gestureClient.call(srvGesture)) {
            ROS_WARN("Could not call service gesture_play");
        }
    }

    virtual void exit() {
	     homeAll();
    }

   bool showEmotion(std::string name, double delay_start) {
     std_msgs::String emotion;
	   emotion.data = name;
	   ros::Duration(delay_start).sleep();
	   emotion_pub.publish(emotion);
    return true;
   }
private:
    rfsm::StateMachine& rfsm;
    std::vector<std::string> feeling;
    ros::Publisher emotion_pub;
    ros::ServiceClient gestureClient;
    ros::ServiceClient homeClient;
    boost::thread* threadEmotion;
};


QTIdleApp::QTIdleApp(ros::NodeHandle& nh) : QTRobotAppBase(nh, "qt_idle_app") {

    std::string fsm_file;
    if(!nh.getParam("/qt_idle_app/fsm", fsm_file)) {
        ROS_ERROR_STREAM("Cannot find param  /qt_idle_app/fsm");
        ros::shutdown();
    }

    if(!rfsm.load(fsm_file)) {
        ROS_ERROR_STREAM("Cannot load "<< fsm_file);
        ros::shutdown();
    }

    // Idle.LookAround callback
    lookAroundStateCB = new LookAroundStateCB(nh, rfsm);
    if(!rfsm.setStateCallback("Idle.LookAround", *lookAroundStateCB)) {
        ROS_ERROR_STREAM("Cannot set callback for LookAround from "<< fsm_file);
        ros::shutdown();
    }

    // Idle.Bored callback
    boredStateCB = new BoredStateCB(nh, rfsm);
    if(!rfsm.setStateCallback("Idle.Bored", *boredStateCB)) {
        ROS_ERROR_STREAM("Cannot set callback for Bored from "<< fsm_file);
        ros::shutdown();
    }

    timer = nh.createTimer(ros::Duration(0.3), &QTIdleApp::timerCallback, this);
}

QTIdleApp::~QTIdleApp() {
    timer.stop();
}


void QTIdleApp::timerCallback(const ros::TimerEvent& event) {
    mutexRFSM.lock();
    rfsm.run();
    mutexRFSM.unlock();
}

bool QTIdleApp::suspend(bool flag) {
  if(flag) {
      dynamic_cast<LookAroundStateCB*>(lookAroundStateCB)->interrupt();
      rfsm.sendEvent("e_suspend");
      mutexRFSM.lock();
      rfsm.run(); // to ensure being in suspended state
      mutexRFSM.unlock();
  }
  else {
      rfsm.sendEvent("e_resume");
  }

  return true;
}
