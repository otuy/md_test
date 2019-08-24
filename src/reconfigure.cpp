#include <ros/ros.h>

#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <dynamic_reconfigure/server.h>
#include <md_test/motorConfig.h>

std_msgs::UInt16 motor_id_msg;
std_msgs::UInt8 motor_cmd_msg;
std_msgs::Float32 motor_cmd_vel_msg;
ros::Publisher motor_id_pub;
ros::Publisher motor_cmd_pub;
ros::Publisher motor_cmd_vel_pub;

void callback(md_test::motorConfig &config, uint32_t level) {
    motor_id_msg.data = (uint16_t)atof(config.id.c_str());
    motor_cmd_msg.data = config.cmd;
    motor_cmd_vel_msg.data = config.cmd_vel;

    motor_id_pub.publish(motor_id_msg);
    motor_cmd_pub.publish(motor_cmd_msg);
    motor_cmd_vel_pub.publish(motor_cmd_vel_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "reconfigure");
    ros::NodeHandle n;

    dynamic_reconfigure::Server<md_test::motorConfig> server;
    dynamic_reconfigure::Server<md_test::motorConfig>::CallbackType f;

    motor_id_pub = n.advertise<std_msgs::UInt16>("motor_id", 100);
    motor_cmd_pub = n.advertise<std_msgs::UInt8>("motor_cmd", 100);
    motor_cmd_vel_pub = n.advertise<std_msgs::Float32>("motor_cmd_vel", 100);

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}