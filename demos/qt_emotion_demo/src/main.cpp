#include <sstream>
#include "ros/ros.h"
#include "qt_emotion_app/qt_emotion_app.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "qt_emotion_app");
    ros::NodeHandle node;

    size_t nCallbacks = 2;

    QTEmotionApp qt_idle(node);

    ROS_INFO("qt_emotion_app is ready with %d callbacks", (int)nCallbacks);
    ros::AsyncSpinner spinner(nCallbacks);
    spinner.start();
    ros::waitForShutdown();
    ROS_INFO("qt_emotion_app is shutting down!");
    return 0;
}
