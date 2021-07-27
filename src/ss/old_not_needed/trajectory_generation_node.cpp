#include <LocalPlanner.h>


int main(int argc, char **argv)
{

    // Launch our ros node
    ros::init(argc, argv, "trajectory_generation_node");
    ros::NodeHandle nh("~");
    
    // call local planner
    localplanner lp(&nh);

    ROS_INFO("done...spinning to ros");
    ros::spin();

    return 0;
}

