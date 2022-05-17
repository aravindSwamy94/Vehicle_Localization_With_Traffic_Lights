#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Float64.h"
#include <move_base_msgs/MoveBaseAction.h>
//#include <actionlib/client/simple_action_client.h>

//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void DMCallback(const std_msgs::Float64& dm_msg)
{
  ROS_DEBUG_NAMED("test_only", "Hello %s", "World");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_publisher");
    ros::NodeHandle n;
    ros::spin();
    return 0;
}
