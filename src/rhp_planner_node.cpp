#include <rhp_planner.h>


int main(int argc, char **argv)
{

    // Launch our ros node
    ros::init(argc, argv, "trajectory_generation_node");
    ros::NodeHandle nh("~");
    
    // call the planner
    planner lp(&nh);

    ROS_INFO("done...spinning to ros");

    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    spinner.spin(); // spin() will not return until the node has been shutdown

    //ros::spin();

    return 0;
}

