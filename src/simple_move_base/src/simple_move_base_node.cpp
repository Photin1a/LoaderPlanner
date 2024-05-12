#include "simple_move_base.h"
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_move_base_node");
    ros::NodeHandle nh;
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    SimpleMoveBase move_base(buffer);
    ros::spin();
    return 0;
}
