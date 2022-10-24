#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <smarty_arm_interface/SmartyArmConfig.h>

void callback(smarty_arm_interface::SmartyArmConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f %f", 
            config.x0_shift, config.y0_shift, config.z0_shift);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamic_position_shift");

  dynamic_reconfigure::Server<smarty_arm_interface::SmartyArmConfig> server;
  dynamic_reconfigure::Server<smarty_arm_interface::SmartyArmConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}