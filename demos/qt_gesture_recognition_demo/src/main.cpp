#include <sstream>
#include "ros/ros.h"
#include "qt_gesturegame_app/qt_gesturegame_app.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "qt_gesturegame_app");
    ros::NodeHandle node;

    size_t nCallbacks = 3;

    QTGestureGameApp qt_idle(node);

    ROS_INFO("qt_gesturegame_app is ready with %d callbacks", (int)nCallbacks);
    ros::AsyncSpinner spinner(nCallbacks);
    spinner.start();
    ros::waitForShutdown();
    ROS_INFO("qt_gesturegame_app is shutting down!");
    return 0;
}
