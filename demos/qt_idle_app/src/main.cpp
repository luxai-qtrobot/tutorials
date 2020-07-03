#include <sstream>
#include "ros/ros.h"
#include "qt_idle_app/qt_idle_app.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "qt_idle_app");
    ros::NodeHandle node;

    size_t nCallbacks = 2;

    QTIdleApp qt_idle(node);

    ROS_INFO("qt_idle_app is ready with %d callbacks", (int)nCallbacks);
    ros::AsyncSpinner spinner(nCallbacks);
    spinner.start();
    ros::waitForShutdown();
    ROS_INFO("qt_idle_app is shutting down!");
    return 0;
}
