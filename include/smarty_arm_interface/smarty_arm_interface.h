#ifndef SMARTY_ARM_NODE_H
#define SMARTY_ARM_NODE_H

/* C++ headers */
#include <pthread.h>
#include <string.h>
#include <errno.h>
#include <vector>
#include <algorithm>

/* ROS headers */
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include "smarty_arm_interface/PTIPacket.h"
#include <dynamic_reconfigure/server.h>
#include <smarty_arm_interface/SmartyArmConfig.h>

/* C headers */
extern "C" {
#include "shm.h"
};

class SMARTY_ARM_Node {
 public:
    explicit SMARTY_ARM_Node(ros::NodeHandle& node, Arm *arm, std::string type);

    ~SMARTY_ARM_Node();

    void run();

 private:
    ros::NodeHandle nh_;

    ros::Subscriber smarty_arm_packet_sub;

    ros::Publisher smarty_arm_packet_pub;

    ros::Publisher smarty_arm_pose_pub; 

    Arm *arm;

    std::string node_type;

    void publish_ptipacket();
    void publish_pose_state();
    void ptipacket_callback(const smarty_arm_interface::PTIPacket::ConstPtr &msg);

    bool initSlave(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
};

#endif /* SMARTY_ARM_NODE */
