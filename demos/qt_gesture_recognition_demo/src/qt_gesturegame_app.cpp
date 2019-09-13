#include <ctime>
#include <cstdlib>
#include <sstream>
#include <std_msgs/Float64MultiArray.h>
#include <boost/thread/thread.hpp>
#include <yaml-cpp/yaml.h>
#include "qt_gesturegame_app/qt_gesturegame_app.h"
#include "qt_idle_app/suspend.h"



#define ROBOT_MEMORY_INCREMENT  1
static char GestureTable[] = {'U', 'R', 'L'};
static int current_user_id = -1;

/**
 * @brief The PlayStateCB class
 */
class PlayStateCB : public rfsm::StateCallback {
public:
    PlayStateCB (ros::NodeHandle& nh, rfsm::StateMachine& fsm, QTrobotServiceHelper& srvHelper):
        rfsm(fsm), serviceHelper(srvHelper){
        srand(time(0));
        if(!nh.getParam("/qt_gesturegame_app/with_audio_files", with_audio_files)) {
            ROS_WARN_STREAM("Cannot find param  /qt_gesturegame_app/with_audio_files.");
        }
        if(with_audio_files) {
            if(!nh.getParam("/qt_gesturegame_app/audio_path", audio_path)) {
                ROS_WARN_STREAM("Cannot find param  /qt_gesturegame_app/audio_path.");
            }
            ROS_INFO_STREAM("Using audios from '"<<audio_path<<"'");
        }
        else {
            std::string speech_message_file;
            if(!nh.getParam("/qt_gesturegame_app/speech_message_file", speech_message_file)) {
                ROS_WARN_STREAM("Cannot find param  /qt_gesturegame_app/speech_message_file.");
            }
            ROS_INFO_STREAM("Loading speech messages from '"<<speech_message_file<<"'");

            try{
                speechMessages = YAML::LoadFile(speech_message_file);
            }
            catch(YAML::BadFile) {
                ROS_ERROR_STREAM("Cannot load speech file '"<<speech_message_file<<"'");
                ros::shutdown();
                return;
            }

            ROS_INFO_STREAM("Using speech in " << speechMessages["language"].as<std::string>());
        }
    }

    virtual void entry() {
        ROS_INFO("Play.entry()");
        if(!with_audio_files) {
            // set speech language
            std::string lang = speechMessages["language"].as<std::string>();
            if(!serviceHelper.speechConfig(lang)) {
                ROS_WARN_STREAM("Cannot set speech language to " << lang);
            }
        }

        prevLevel = -1;
        if(with_audio_files)
            serviceHelper.talkAudioPlayGesture("memory_game_001", "QT/challenge", 2.0, true, audio_path);
        else
            serviceHelper.talkTextPlayGesture(speechMessages["memory_game_001"].as<std::string>(), "QT/challenge", 2.0, true);
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
                robotMemory += GestureTable[(rand()%3)];
            }

            ROS_INFO_STREAM("Show "<<robotMemory);
            if(robotMemory.size() == ROBOT_MEMORY_INCREMENT)
                talk("memory_game_003");
            else
                talk("memory_game_004");

            for(size_t i=0; i<robotMemory.size(); i++) {
                if(robotMemory[i] == 'U')
                    serviceHelper.playGesture("QT/up_right", 1.2);
                else if(robotMemory[i] == 'R')
                    serviceHelper.playGesture("QT/swipe_right", 1.2);
                else if(robotMemory[i] == 'L')
                    serviceHelper.playGesture("QT/swipe_left", 1.2);
                else
                    ROS_WARN_STREAM("Unknown gesture " << robotMemory[i]);
            }
            //ros::Duration(2.0).sleep();

            // clear player memory
            playerMemory.clear();
            ROS_INFO_STREAM("Your turn...");
            talk("memory_game_005");
            // loop until player take enough moves
            size_t robotMemorySize = robotMemory.size();
            while((playerMemory.size() <= robotMemorySize)
                  && !ros::isShuttingDown() && !interrupted) {

                mutexLables.lock();
                shown_gesture.clear();
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
                        else
                            serviceHelper.talkTextPlayGesture(speechMessages["memory_game_006"].as<std::string>(), "QT/angry", 1.0, true);
                        interrupted = true;
                        break;
                    }

                    mutexLables.lock();
                    if(shown_gesture.size()) {
                        ROS_INFO_STREAM("shown "<< shown_gesture);
                        read = true;
                        if(shown_gesture == "SWIPE UP") {
                            playerMemory += "U";
                            talk("memory_game_015");
                        }
                        else if(shown_gesture == "SWIPE RIGHT") {
                            playerMemory += "R";
                            talk("memory_game_007");
                        }
                        else if(shown_gesture == "SWIPE LEFT") {
                            playerMemory += "L";                            
                            talk("memory_game_008");
                        }
                        else {                            
                            read = false;
                        }
                    }
                    mutexLables.unlock();
                }

                //ros::Duration(0.5).sleep();
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

    void onGestureDetected(const std::string& gesture, int id) {
        if((gesture == "SWIPE DOWN") || (gesture == "WAVING"))
            return;
        if(id != current_user_id)
            return;
        mutexLables.lock();
        shown_gesture = gesture;
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
                ROS_INFO_STREAM("talking '" <<speechMessages[message].as<std::string>() <<"'");
                ret = serviceHelper.talkText(speechMessages[message].as<std::string>());
            } catch(...) {
                ret = false;
            }
            if(!ret)
                ROS_WARN_STREAM("Could not talk message " << message);
        }
        return ret;
    }


private:
    bool interrupted;
    int prevLevel;
    ros::Time start_time;
    std::string shown_gesture;
    std::string robotMemory;
    std::string playerMemory;
    boost::mutex mutexLables;
    rfsm::StateMachine& rfsm;
    std::string audio_path;
    bool with_audio_files;
    YAML::Node speechMessages;
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

    void onGestureDetected(const std::string& gesture, int id) {
        // TODO check if both Q & T are shown        
        if(inState && gesture == "SWIPE UP") {
	    /*	
            static int gesture_count = 0;
            static ros::Time prev_gesture_time = ros::Time::now();

            if((ros::Time::now()-prev_gesture_time).toSec() > 3.0) {
                gesture_count = 0;
                prev_gesture_time = ros::Time::now();
                return;
            }

            if(gesture_count >= 1) {
                gesture_count = 0;
                current_user_id = id;
                rfsm.sendEvent("e_start_game");
                return;
            }

            if(gesture_count == 0) {
                gesture_count++;
                prev_gesture_time = ros::Time::now();
                return;
            }
            gesture_count++;
            */
	    current_user_id = id;
            rfsm.sendEvent("e_start_game");   
        }
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
    QTrobotServiceHelper& serviceHelper;
};


QTGestureGameApp::QTGestureGameApp(ros::NodeHandle& nh) : serviceHelper(nh) {

    std::string fsm_file;
    if(!nh.getParam("/qt_gesturegame_app/fsm", fsm_file)) {
        ROS_ERROR_STREAM("Cannot find param  /qt_gesturegame_app/fsm");
        ros::shutdown();
    }

    if(!rfsm.load(fsm_file)) {
        ROS_ERROR_STREAM("Cannot load "<< fsm_file);
        ros::shutdown();
    }

    //serviceSuspend = nh.advertiseService("qt_gesturegame_app/suspend", &QTGestureGameApp::suspendCB, this);
    subGestures = nh.subscribe("/qt_nuitrack_app/gestures", 10, &QTGestureGameApp::gestureSubCB, this);


    // set speech language to English
    if(!serviceHelper.speechConfig("en-US")) {
        ROS_ERROR_STREAM("Cannot set speech language to 'en-US'");
        ros::shutdown();
    }

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

    fsmTimer = nh.createTimer(ros::Duration(0.3), &QTGestureGameApp::fsmTimerCallback, this);
}

QTGestureGameApp::~QTGestureGameApp() {
    fsmTimer.stop();
}


void QTGestureGameApp::fsmTimerCallback(const ros::TimerEvent& event) {
    mutexRFSM.lock();
    rfsm.run();
    mutexRFSM.unlock();
}



/*
void QTGestureGameApp::onNewGestures(const tdv::nuitrack::GestureData::Ptr gesture_data) {
    //ROS_INFO_STREAM("onNewGestures...");
    const std::vector<tdv::nuitrack::Gesture> gestures = gesture_data->getGestures();
    for( const tdv::nuitrack::Gesture& gesture : gestures ){
        ROS_INFO_STREAM(gesture.userId << ": " << type2string(gesture.type));        
        if(gesture.type == GestureType::GESTURE_SWIPE_UP ||
           gesture.type == GestureType::GESTURE_SWIPE_RIGHT ||
           gesture.type == GestureType::GESTURE_SWIPE_LEFT) {
            dynamic_cast<WaitingStateCB*>(waitingStateCB)->onGestureDetected(type2string(gesture.type));
            dynamic_cast<PlayStateCB*>(playStateCB)->onGestureDetected(type2string(gesture.type));
            return;
        }
    }
}
*/

bool QTGestureGameApp::suspend(bool flag) {
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

void QTGestureGameApp::gestureSubCB(const qt_nuitrack_app::Gestures::ConstPtr& gestures) {

    for(int i=0; i<gestures->gestures.size(); i++) {
        ROS_INFO_STREAM("gesture: " <<gestures->gestures[i].name << ", id: "<< gestures->gestures[i].id);
        dynamic_cast<WaitingStateCB*>(waitingStateCB)->onGestureDetected(gestures->gestures[i].name,
                                                                         gestures->gestures[i].id);
        dynamic_cast<PlayStateCB*>(playStateCB)->onGestureDetected(gestures->gestures[i].name,
                                                                   gestures->gestures[i].id);
    }
}


/*
void QTGestureGameApp::objectsSubCB(const std_msgs::Float32MultiArray::ConstPtr& objects) {
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
*/
