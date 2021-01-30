#include <ros/ros.h>
#include <t265_depth/t265_depth.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "t265_depth");
    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");

    t265_depth::t265Depth t265Depth(node, privateNode);
    ros::spin();
    return 0;
}
