#include <ctime>
#include <cstdlib>
#include <sstream>
#include <std_msgs/Float64MultiArray.h>

#include "qt_emotion_app/qt_emotion_app.h"
#include "qt_idle_app/suspend.h"

#include <boost/thread/thread.hpp>
#include <yaml-cpp/yaml.h>


/**
 * @brief The PlayStateCB class
 */
class PlayStateCB : public rfsm::StateCallback {
public:
    PlayStateCB (ros::NodeHandle& nh, rfsm::StateMachine& fsm, QTrobotServiceHelper& srvHelper):
        rfsm(fsm), inState(false), serviceHelper(srvHelper), with_audio_files(false) {
        srand(time(0));
        if(!nh.getParam("/qt_emotion_app/with_audio_files", with_audio_files)) {
            ROS_WARN_STREAM("Cannot find param  /qt_emotion_app/with_audio_files.");
        }
        if(with_audio_files) {
            if(!nh.getParam("/qt_emotion_app/audio_path", audio_path)) {
                ROS_WARN_STREAM("Cannot find param  /qt_emotion_app/audio_path.");
            }
            ROS_INFO_STREAM("Using audios from '"<<audio_path<<"'");
        }
        else {
            std::string speech_message_file;
            if(!nh.getParam("/qt_emotion_app/speech_message_file", speech_message_file)) {
                ROS_WARN_STREAM("Cannot find param  /qt_emotion_app/speech_message_file.");
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
        inState = true;                
        ROS_INFO("Play.entry()");                

        ROS_INFO("Play.entry()");
        if(!with_audio_files) {
            // set speech language to English
            std::string lang = speechMessages["language"].as<std::string>();
            if(!serviceHelper.speechConfig(lang)) {
                ROS_WARN_STREAM("Cannot set speech language to " << lang);
            }
        }

        ROS_INFO_STREAM("emotion: " << shown_lables);
        if(shown_lables == "happy") {
            talk("emotion_recognition_001");
            serviceHelper.showEmotionPlayGesture("QT/happy", "QT/happy", 0.5, true);
        }
        else if(shown_lables == "sad") {
            talk("emotion_recognition_002");
            serviceHelper.showEmotionPlayGesture("QT/sad", "QT/sad", 1.0, true);
            talk("emotion_recognition_003");
        }
        else if(shown_lables == "angry") {
            talk("emotion_recognition_004");
            serviceHelper.showEmotionPlayGesture("QT/angry", "QT/angry", 0.5, true);
            talk("emotion_recognition_005");
            serviceHelper.showEmotionPlayGesture("QT/breathing_exercise", "QT/breathing_exercise", 0.5, true);
        }
        else if(shown_lables == "disgusted") {
            talk("emotion_recognition_006");
            serviceHelper.showEmotion("QT/disgusted");
            talk("emotion_recognition_007");
        }
        else if(shown_lables == "surprised") {
            talk("emotion_recognition_008");
            serviceHelper.showEmotionPlayGesture("QT/surprise", "QT/surprise", 2.0, true);
            talk("emotion_recognition_009");
        }
    }

  virtual void exit() {
        ROS_INFO("Play.exit()");
        serviceHelper.homeAll();
        ros::Duration(1.0).sleep();
        inState = false;

        if(!with_audio_files) {
            // set speech language to English
            if(!serviceHelper.speechConfig("default")) {
                ROS_WARN_STREAM("Cannot set back speech language to 'default'");
            }
        }
    }

    void onObjectsDetected(const std::string& lables) {
        mutexLables.lock();
        shown_lables = lables;
        mutexLables.unlock();
        if(!inState && lables.size())
            rfsm.sendEvent("e_start_game");
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
    bool inState;
    bool interrupted;
    std::string shown_lables;
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
        rfsm(fsm), serviceHelper(srvHelper){
        idleAppClient = nh.serviceClient<qt_idle_app::suspend>("/qt_idle_app/suspend");
        if(!idleAppClient.exists()) {
            ROS_WARN("Could connect to /qt_idle_app/suspend");
        }
    }

    virtual void entry() {
        ROS_INFO("Waiting.entry()");
        // resume qt_idle_app
        suspendIdleApp(false);
    }

    virtual void exit() {
        ROS_INFO("Waiting.exit()");
        // suspend qt_idle_app
        suspendIdleApp(true);
        // home all robot
        serviceHelper.homeAll();        
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


QTEmotionApp::QTEmotionApp(ros::NodeHandle& nh) : serviceHelper(nh) {

    std::string fsm_file;
    if(!nh.getParam("/qt_emotion_app/fsm", fsm_file)) {
        ROS_ERROR_STREAM("Cannot find param  /qt_emotion_app/fsm");
        ros::shutdown();
    }

    if(!rfsm.load(fsm_file)) {
        ROS_ERROR_STREAM("Cannot load "<< fsm_file);
        ros::shutdown();
    }

    //serviceSuspend = nh.advertiseService("qt_emotion_app/suspend", &QTEmotionApp::suspendCB, this);

    subObjects = nh.subscribe("/find_object/objects", 10, &QTEmotionApp::objectsSubCB, this);


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

    timer = nh.createTimer(ros::Duration(0.3), &QTEmotionApp::timerCallback, this);

}

QTEmotionApp::~QTEmotionApp() {
    timer.stop();
}


void QTEmotionApp::timerCallback(const ros::TimerEvent& event) {
    mutexRFSM.lock();
    rfsm.run();
    mutexRFSM.unlock();
}

bool QTEmotionApp::suspend(bool flag) {
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

void QTEmotionApp::objectsSubCB(const std_msgs::Float32MultiArray::ConstPtr& objects) {
    if(objects->data.size() <= 0 )
        return;

    std::string lables;
    for(size_t i=0; i<objects->data.size(); i+=12) {
        if(objects->data[i] == 3)
            lables = "surprised";
        else if(objects->data[i] == 4)
            lables = "disgusted";
        else if(objects->data[i] == 5)
            lables = "angry";
        else if(objects->data[i] == 6)
            lables = "sad";
        else if(objects->data[i] == 7)
            lables = "happy";
    }
    // call related callbacks
    if(lables.size()) {
        dynamic_cast<PlayStateCB*>(playStateCB)->onObjectsDetected(lables);
    }
}
