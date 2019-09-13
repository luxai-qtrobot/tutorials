#include <sstream>
#include "ros/ros.h"
#include "qt_memgame_app/qt_memgame_app.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "qt_memgame_app");
    ros::NodeHandle node;

    size_t nCallbacks = 2;

    QTMemGameApp qt_idle(node);

    ROS_INFO("qt_memgame_app is ready with %d callbacks", (int)nCallbacks);
    ros::AsyncSpinner spinner(nCallbacks);
    spinner.start();
    ros::waitForShutdown();
    ROS_INFO("qt_memgame_app is shutting down!");
    return 0;
}
