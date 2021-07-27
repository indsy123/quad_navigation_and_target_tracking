#include <vio2cog_transform.h>


int main(int argc, char **argv)
{

    // Launch our ros node
    ros::init(argc, argv, "transform_vio_to_cog");
    ros::NodeHandle nh("~");
    
    // call the planner
    vio2cog vc(&nh);

    ROS_INFO("done...spinning to ros");
    ros::spin();

    return 0;
}

