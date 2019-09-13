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


#ifndef QT_SERVICE_HELPER
#define QT_SERVICE_HELPER


#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

#include "qt_robot_interface/behavior_talk_audio.h"
#include "qt_robot_interface/behavior_talk_text.h"
#include "qt_robot_interface/speech_say.h"
#include "qt_robot_interface/speech_config.h"
#include "qt_robot_interface/audio_play.h"
#include "qt_robot_interface/emotion_show.h"
#include "qt_gesture_controller/gesture_play.h"
#include "qt_gesture_controller/gesture_record.h"
#include "qt_gesture_controller/gesture_save.h"
#include "qt_motors_controller/home.h"


class QTrobotServiceHelper {
public:
    QTrobotServiceHelper(ros::NodeHandle& nh) {

      // speech say service client
      speechSayClient = nh.serviceClient<qt_robot_interface::speech_say>("/qt_robot/speech/say");
      if(!speechSayClient.exists()) {
          ROS_WARN("Could connect to /qt_robot/speech/say service");
      }

      // speech config service client
      speechConfigClient = nh.serviceClient<qt_robot_interface::speech_config>("/qt_robot/speech/config");
      if(!speechConfigClient.exists()) {
          ROS_WARN("Could connect to /qt_robot/speech/config service");
      }

      // audio service client
      audioClient = nh.serviceClient<qt_robot_interface::audio_play>("/qt_robot/audio/play");
      if(!audioClient.exists()) {
          ROS_WARN("Could connect to /qt_robot/audio/play service");
      }

      // emotion service client
      emotionShowClient = nh.serviceClient<qt_robot_interface::emotion_show>("/qt_robot/emotion/show");
      if(!emotionShowClient.exists()) {
          ROS_WARN("Could connect to /qt_robot/emotion/show service");
      }

      // talk text service client
      talkAudioClient = nh.serviceClient<qt_robot_interface::behavior_talk_audio>("/qt_robot/behavior/talkAudio");
      if(!talkAudioClient.exists()) {
          ROS_WARN("Could connect to /qt_robot/behavior/talkAudio service");
      }
      // talk audio service client
      talkTextClient = nh.serviceClient<qt_robot_interface::behavior_talk_text>("/qt_robot/behavior/talkText");
      if(!talkTextClient.exists()) {
          ROS_WARN("Could connect to /qt_robot/behavior/talkText service");
      }

      // gesture service client
      gesturePlayClient = nh.serviceClient<qt_gesture_controller::gesture_play>("/qt_robot/gesture/play");
      if(!gesturePlayClient.exists()) {
          ROS_WARN("Could connect to /qt_robot/gesture/play service");
      }

      // gesture record service client
      gestureRecordClient = nh.serviceClient<qt_gesture_controller::gesture_record>("/qt_robot/gesture/record");
      if(!gestureRecordClient.exists()) {
          ROS_WARN("Could connect to /qt_robot/gesture/record service");
      }

      // gesture save service client
      gestureSaveClient = nh.serviceClient<qt_gesture_controller::gesture_save>("/qt_robot/gesture/save");
      if(!gestureSaveClient.exists()) {
          ROS_WARN("Could connect to /qt_robot/gesture/save service");
      }

      // motors clients
      homeClient = nh.serviceClient<qt_motors_controller::home>("/qt_robot/motors/home");
      if(!homeClient.exists()) {
          ROS_WARN("Could connect to /qt_robot/motors/home service");
      }

    }

    virtual ~QTrobotServiceHelper() { }

    bool showEmotion(std::string name) {
        qt_robot_interface::emotion_show cmd;
        cmd.request.name = name;
    if(!emotionShowClient.call(cmd)) {
            ROS_WARN("Could not call service emotion_show");
            return false;
        }
        return cmd.response.status;
    }

    bool talkText(std::string message) {
        qt_robot_interface::behavior_talk_text cmd;
        cmd.request.message = message;
        if(!talkTextClient.call(cmd)) {
            ROS_WARN("Could not call service talk_text");
            return false;
        }
        return cmd.response.status;
    }

    bool talkAudio(std::string filename, std::string filePath="") {
        qt_robot_interface::behavior_talk_audio cmd;
        cmd.request.filename = filename;
        cmd.request.filepath = filePath;
        if(!talkAudioClient.call(cmd)) {
            ROS_WARN("Could not call service talk_audio");
            return false;
        }
        return cmd.response.status;
    }

    bool playGesture(std::string name, float speed=0.0 ) {
        qt_gesture_controller::gesture_play cmd;
        cmd.request.speed = speed;
        cmd.request.name = name;
        if(!gesturePlayClient.call(cmd)) {
            ROS_WARN("Could not call service gesture_play");
            return false;
        }
        return cmd.response.status;
    }

    bool recordGesture(std::vector<std::string>& parts, bool idleParts=true, int wait=0, int timeout=0) {
        qt_gesture_controller::gesture_record cmd;
        cmd.request.parts = parts;
        cmd.request.idleParts = idleParts;
        cmd.request.wait = wait;
        cmd.request.timeout = timeout;
        if(!gestureRecordClient.call(cmd)) {
            ROS_WARN("Could not call service gesture_record");
            return false;
        }
        return cmd.response.status;
    }

    bool saveGesture(std::string name, std::string path="") {
        qt_gesture_controller::gesture_save cmd;
        cmd.request.name = name;
        cmd.request.path = path;
        if(!gestureSaveClient.call(cmd)) {
            ROS_WARN("Could not call service gesture_save");
            return false;
        }
        return cmd.response.status;
    }

    bool talkTextPlayGesture(std::string message, std::string name,
                             double delay_sync=0.0, bool reverse=false) {
        if(!reverse) {
            boost::thread* threadText = new boost::thread(boost::bind(&QTrobotServiceHelper::talkText, this, message));
            ros::Duration(delay_sync).sleep();
            boost::thread* threadGesture = new boost::thread(boost::bind(&QTrobotServiceHelper::playGesture, this, name, 0.0));
            threadText->join();
            threadGesture->join();
        }
        else
        {
            boost::thread* threadGesture = new boost::thread(boost::bind(&QTrobotServiceHelper::playGesture, this, name, 0.0));
            ros::Duration(delay_sync).sleep();
            boost::thread* threadText = new boost::thread(boost::bind(&QTrobotServiceHelper::talkText, this, message));
            threadText->join();
            threadGesture->join();
        }
        return true;
    }

    bool talkAudioPlayGesture(std::string audioName, std::string gesturName,
                             double delay_sync=0.0, bool reverse=false, std::string audioPath="") {
        if(!reverse) {
            boost::thread* threadAudio = new boost::thread(boost::bind(&QTrobotServiceHelper::talkAudio, this, audioName, audioPath));
            ros::Duration(delay_sync).sleep();
            boost::thread* threadGesture = new boost::thread(boost::bind(&QTrobotServiceHelper::playGesture, this, gesturName, 0.0));
            threadAudio->join();
            threadGesture->join();
        }
        else
        {
            boost::thread* threadGesture = new boost::thread(boost::bind(&QTrobotServiceHelper::playGesture, this, gesturName, 0.0));
            ros::Duration(delay_sync).sleep();
            boost::thread* threadAudio = new boost::thread(boost::bind(&QTrobotServiceHelper::talkAudio, this, audioName, audioPath));
            threadAudio->join();
            threadGesture->join();
        }
        return true;
    }


    bool showEmotionPlayGesture(std::string emotion, std::string gesture,
                                double delay_sync=0.0, bool reverse=false) {
        if(!reverse) {
            boost::thread* threadEmotion = new boost::thread(boost::bind(&QTrobotServiceHelper::showEmotion, this, emotion));
            ros::Duration(delay_sync).sleep();
            boost::thread* threadGesture = new boost::thread(boost::bind(&QTrobotServiceHelper::playGesture, this, gesture, 0.0));
            threadEmotion->join();
            threadGesture->join();
        }
        else
        {
            boost::thread* threadGesture = new boost::thread(boost::bind(&QTrobotServiceHelper::playGesture, this, gesture, 0.0));
            ros::Duration(delay_sync).sleep();
            boost::thread* threadEmotion = new boost::thread(boost::bind(&QTrobotServiceHelper::showEmotion, this, emotion));
            threadEmotion->join();
            threadGesture->join();
        }
        return true;
    }


    bool home(const std::vector<std::string>& parts) {
        qt_motors_controller::home cmd;
        for(size_t i=0; i<parts.size(); i++)
            cmd.request.parts.push_back(parts[i]);
        if(!homeClient.call(cmd)) {
            ROS_WARN("Could not call service home");
            return false;
        }
        return cmd.response.status;
    }

    bool homeAll() {
        std::vector<std::string> parts;
        parts.push_back("head");
        parts.push_back("right_arm");
        parts.push_back("left_arm");
        return home(parts);
    }

    bool speechSay(std::string message) {
      qt_robot_interface::speech_say cmd;
      cmd.request.message = message;
      if(!speechSayClient.call(cmd)) {
          ROS_WARN("Could not call service speech_say");
          return false;
      }
      return cmd.response.status;
    }

    bool speechConfig(std::string language, int pitch=140, int speed=80) {
      qt_robot_interface::speech_config cmd;
      cmd.request.language = language;
      cmd.request.pitch = pitch;
      cmd.request.speed = speed;
      if(!speechConfigClient.call(cmd)) {
          ROS_WARN("Could not call service speech_config");
          return false;
      }
      return cmd.response.status;
    }

    bool playAudio(std::string filename, std::string filepath="") {
      qt_robot_interface::audio_play cmd;
      cmd.request.filename = filename;
      cmd.request.filepath = filepath;
      if(!audioClient.call(cmd)) {
          ROS_WARN("Could not call service audio_play");
          return false;
      }
      return cmd.response.status;
    }

private:
  ros::ServiceClient homeClient;
  ros::ServiceClient talkAudioClient;
  ros::ServiceClient talkTextClient;
  ros::ServiceClient gesturePlayClient;
  ros::ServiceClient gestureRecordClient;
  ros::ServiceClient gestureSaveClient;
  ros::ServiceClient speechSayClient;
  ros::ServiceClient speechConfigClient;
  ros::ServiceClient audioClient;
  ros::ServiceClient emotionShowClient;
};


#endif
