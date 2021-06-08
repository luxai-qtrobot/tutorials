#include <ctime>
#include <cstdlib>
#include <sstream>
#include <std_msgs/Float64MultiArray.h>

#include "qt_memgame_app/qt_memgame_app.h"
#include "qt_idle_app/suspend.h"

#include <boost/thread/thread.hpp>

#define ROBOT_MEMORY_INCREMENT  1
static char GestureTable[] = {'Q', 'T'};


/**
 * @brief The PlayStateCB class
 */
class PlayStateCB : public rfsm::StateCallback {
public:
    PlayStateCB (ros::NodeHandle& nh, rfsm::StateMachine& fsm, QTrobotServiceHelper& srvHelper):
        rfsm(fsm), serviceHelper(srvHelper), with_audio_files(false), node(nh) {
        srand(time(0));

        if(!nh.getParam("/qt_memgame_app/with_audio_files", with_audio_files)) {
            ROS_WARN_STREAM("Cannot find param  /qt_memgame_app/with_audio_files.");
        }
        if(with_audio_files) {
            if(!nh.getParam("/qt_memgame_app/audio_path", audio_path)) {
                ROS_WARN_STREAM("Cannot find param  /qt_memgame_app/audio_path.");
            }
            ROS_INFO_STREAM("Using audios from '"<<audio_path<<"'");
        }
    }

    virtual void entry() {
        ROS_INFO("Play.entry()");
        if(!with_audio_files) {
            // set speech language to English
            std::string lang;
            node.getParam("/qt_memgame_app/language", lang);
            if(!serviceHelper.speechConfig(lang)) {
                ROS_WARN_STREAM("Cannot set speech language to " << lang);
            }
        }

        prevLevel = -1;
        if(with_audio_files)
            serviceHelper.talkAudioPlayGesture("memory_game_001", "QT/challenge", 2.0, true, audio_path);
        else{
            std::string msg;
            node.getParam("/qt_memgame_app/messages/memory_game_001", msg);
            serviceHelper.talkTextPlayGesture(msg, "QT/challenge", 2.0, true);
        }
        ros::Duration(1.0).sleep();

        // doo
        interrupted = false;
        robotMemory.clear();
        playerMemory.clear();
        start_time = ros::Time::now();

        while(!ros::isShuttingDown() && !interrupted) {
            // motivate player
            if(robotMemory.size() == 4)
                talk("memory_game_002");

            // update robot memory
            for(size_t rep=0; rep <ROBOT_MEMORY_INCREMENT; rep++) {
                robotMemory += GestureTable[(rand()%2)];
            }

            //ROS_INFO_STREAM("Show "<<robotMemory);
            if(robotMemory.size() == ROBOT_MEMORY_INCREMENT)
                talk("memory_game_003");
            else
                talk("memory_game_004");

            for(size_t i=0; i<robotMemory.size(); i++) {
                if(robotMemory[i] == 'Q')
                    serviceHelper.playGesture("QT/show_right", 1.0);
                else if(robotMemory[i] == 'T')
                    serviceHelper.playGesture("QT/show_left", 1.0);
                else
                    ROS_WARN_STREAM("Unknown gesture " << robotMemory[i]);
            }

            // clear player memory
            playerMemory.clear();
            ROS_INFO_STREAM("Your turn...");
            talk("memory_game_005");
            // loop until player take enough moves
            size_t robotMemorySize = robotMemory.size();
            while((playerMemory.size() <= robotMemorySize)
                  && !ros::isShuttingDown() && !interrupted) {

                mutexLables.lock();
                shown_lables.clear();
                mutexLables.unlock();

                // wait until user show a lable (read from user shwon image)
                ROS_INFO("Reading...");
                ros::Time start_read_time = ros::Time::now();
                bool read = false;
                while(!read && !ros::isShuttingDown() && !interrupted) {
                    if((ros::Time::now() - start_read_time).toSec() > 10.0) {
                        ROS_INFO("timeout! Okay! it seems you do not want to play anymore!");
                        if(with_audio_files)
                            serviceHelper.talkAudioPlayGesture("memory_game_006", "QT/angry", 1.0, true, audio_path);
                        else{
                            std::string msg;
                            node.getParam("/qt_memgame_app/messages/memory_game_006", msg);
                            serviceHelper.talkTextPlayGesture(msg, "QT/angry", 1.0, true);
                        }
                        interrupted = true;
                        break;
                    }

                    mutexLables.lock();
                    if(shown_lables.size()) {
                        //ROS_INFO_STREAM("shown "<< shown_lables[0]);
                        talk((shown_lables[0] == 'Q') ? "memory_game_007" : "memory_game_008");
                        playerMemory += shown_lables[0];
                        read = true;
                    }
                    mutexLables.unlock();
                }

                ros::Duration(0.5).sleep();
                if(playerMemory.size() <= robotMemorySize) { // ignore the last one
                    if(playerMemory.size() && (playerMemory != robotMemory.substr(0, playerMemory.size())))
                        break;
                }
                else // add the last gesture to robot memory
                    robotMemory.push_back(playerMemory.back());

            } // end of moves loop

            ROS_INFO_STREAM("Player memory: " << playerMemory);
            ROS_INFO_STREAM("Robot memory : " << robotMemory);
            if((playerMemory != robotMemory) && !interrupted) {
                //ros::Duration(1.0).sleep();
                talk("memory_game_009");
                serviceHelper.showEmotion("QT/shy");
                break;
            }

        } // end of main loop

  }

  virtual void exit() {
        int level = robotMemory.size()/2;
        ros::Duration game_duration = ros::Time::now() - start_time;
        if(robotMemory.size() < 5 ) {
            ROS_INFO("It was a short game!");
            talk("memory_game_010");
        } else if(robotMemory.size() >= 5 ) {
            ROS_INFO("Well Done! It was a good game!");
            talk("memory_game_011");
        } else if(robotMemory.size() >= 8 ) {
            talk("memory_game_012");
        }

        if(prevLevel != -1) {
            if(level < prevLevel)
                talk("memory_game_013");
            else
                talk("memory_game_014");
        }
        serviceHelper.showEmotionPlayGesture("QT/kiss", "QT/kiss", 2.0, true);

        if(!with_audio_files) {
            // set speech language to default
            if(!serviceHelper.speechConfig("default")) {
                ROS_WARN_STREAM("Cannot set speech language to 'default'");
            }
        }

    }

    void onObjectsDetected(const std::string& lables) {
        mutexLables.lock();
        shown_lables = lables;
        mutexLables.unlock();
    }

    void interrupt() {interrupted = true; }

    bool talk(std::string message){
        bool ret = true;
        if(with_audio_files) {
            ret = serviceHelper.talkAudio(message, audio_path);
            if(!ret)
                ROS_WARN_STREAM("Could not play audio " << message);
        }
        else {
            try{
                std::string talk_msg;
                node.getParam("/qt_memgame_app/messages/"+message, talk_msg);
                ROS_INFO_STREAM("talking '" << talk_msg <<"'");
                ret = serviceHelper.talkText(talk_msg);
            } catch(...) {
                ret = false;
            }
            if(!ret)
                ROS_WARN_STREAM("Could not talk message " << message);
        }
        return ret;
    }


private:
    ros::NodeHandle& node;
    bool interrupted;
    int prevLevel;
    ros::Time start_time;
    std::string shown_lables;
    std::string robotMemory;
    std::string playerMemory;
    boost::mutex mutexLables;
    rfsm::StateMachine& rfsm;
    std::string audio_path;
    bool with_audio_files;
    QTrobotServiceHelper& serviceHelper;
};


/**
 * @brief The WaitingStateCB class
 */
class WaitingStateCB : public rfsm::StateCallback {
public:
    WaitingStateCB (ros::NodeHandle& nh, rfsm::StateMachine& fsm, QTrobotServiceHelper& srvHelper):
        rfsm(fsm), inState(false), serviceHelper(srvHelper){
        idleAppClient = nh.serviceClient<qt_idle_app::suspend>("/qt_idle_app/suspend");
        if(!idleAppClient.exists()) {
            ROS_WARN("Could connect to /qt_idle_app/suspend");
        }
    }

    virtual void entry() {
        ROS_INFO("Waiting.entry()");
        // resume qt_idle_app
        suspendIdleApp(false);
        inState = true;
    }

    virtual void exit() {
        ROS_INFO("Waiting.exit()");
        // suspend qt_idle_app
        suspendIdleApp(true);
        // home all robot
        serviceHelper.homeAll();
        inState = false;
    }

    void onObjectsDetected(const std::string& lables) {
        // TODO check if both Q & T are shown
        if(inState && lables.size())
            rfsm.sendEvent("e_start_game");
    }

private:

    bool suspendIdleApp(bool flag) {
        qt_idle_app::suspend cmdSuspend;
        cmdSuspend.request.flag = flag;
        if(!idleAppClient.call(cmdSuspend)) {
            ROS_WARN("Could not call service qt_idle_app::suspend");
            return false;
        }
        return cmdSuspend.response.status;
    }


private:
    bool inState;
    rfsm::StateMachine& rfsm;
    ros::ServiceClient idleAppClient;
    QTrobotServiceHelper serviceHelper;
};


QTMemGameApp::QTMemGameApp(ros::NodeHandle& nh) : serviceHelper(nh) {

    std::string fsm_file;
    if(!nh.getParam("/qt_memgame_app/fsm", fsm_file)) {
        ROS_ERROR_STREAM("Cannot find param  /qt_memgame_app/fsm");
        ros::shutdown();
    }

    if(!rfsm.load(fsm_file)) {
        ROS_ERROR_STREAM("Cannot load "<< fsm_file);
        ros::shutdown();
    }

    //serviceSuspend = nh.advertiseService("qt_memgame_app/suspend", &QTMemGameApp::suspendCB, this);

    subObjects = nh.subscribe("/find_object/objects", 10, &QTMemGameApp::objectsSubCB, this);


    // Waiting callback
    waitingStateCB = new WaitingStateCB(nh, rfsm, serviceHelper);
    if(!rfsm.setStateCallback("Waiting", *waitingStateCB)) {
        ROS_ERROR_STREAM("Cannot set callback for Waiting from "<< fsm_file);
        ros::shutdown();
    }

    // Play callback
    playStateCB = new PlayStateCB(nh, rfsm, serviceHelper);
    if(!rfsm.setStateCallback("Game.Play", *playStateCB)) {
        ROS_ERROR_STREAM("Cannot set callback for Waiting from "<< fsm_file);
        ros::shutdown();
    }

    timer = nh.createTimer(ros::Duration(0.3), &QTMemGameApp::timerCallback, this);

}

QTMemGameApp::~QTMemGameApp() {
    timer.stop();
}


void QTMemGameApp::timerCallback(const ros::TimerEvent& event) {
    mutexRFSM.lock();
    rfsm.run();
    mutexRFSM.unlock();
}

bool QTMemGameApp::suspend(bool flag) {
  if(flag) {
      //dynamic_cast<LookAroundStateCB*>(lookAroundStateCB)->interrupt();
      dynamic_cast<PlayStateCB*>(playStateCB)->interrupt();
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

void QTMemGameApp::objectsSubCB(const std_msgs::Float32MultiArray::ConstPtr& objects) {
    if(objects->data.size() <= 0 )
        return;

    std::string lables;
    for(size_t i=0; i<objects->data.size(); i+=12) {
        if(objects->data[i] == 1)
            lables.push_back('Q');
        else if(objects->data[i] == 2)
            lables.push_back('T');
    }
    // call related callbacks
    if(lables.size()) {
        dynamic_cast<WaitingStateCB*>(waitingStateCB)->onObjectsDetected(lables);
        dynamic_cast<PlayStateCB*>(playStateCB)->onObjectsDetected(lables);
    }
}
