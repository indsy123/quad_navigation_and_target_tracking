
#include <controller_sim.h>

int main(int argc, char **argv)
{

    // Launch controller node
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh("~");

    // Create subscribers
    controller c(&nh);

    ROS_INFO("done...spinning to ros");
    ros::spin();

    return 0;
}
