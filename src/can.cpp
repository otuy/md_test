/*
 * mr1_can.cpp
 *
 *  Created on: Feb 27, 2019
 *      Author: yuto
 */

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <boost/array.hpp>

#include <can_msgs/CanFrame.h>

#define CAN_MTU 8

template<typename T>
union _Encapsulator
{
    T data;
    uint64_t i;
};

template <typename T>
void can_unpack(const boost::array<uint8_t, CAN_MTU> &buf, T &data)
{
    _Encapsulator<T> _e;

    for(int i = 0; i < sizeof(T); i++)
    {
        _e.i = (_e.i << 8) | (uint64_t)(buf[i]);
    }

    data = _e.data;
}

template<typename T>
void can_pack(boost::array<uint8_t, CAN_MTU> &buf, const T data)
{
    _Encapsulator<T> _e;
    _e.data = data;

    for(int i = sizeof(T); i > 0;)
    {
        i--;
        buf[i] = _e.i & 0xff;
        _e.i >>= 8;
    }
}

class CanNode
{
public:
    CanNode(void);

private:
    void motorIDCallback(const std_msgs::UInt16::ConstPtr& msg);
    void motorCmdCallback(const std_msgs::UInt8::ConstPtr& msg);
    void motorCmdVelCallback(const std_msgs::Float32::ConstPtr& msg);

    // void canRxCallback(const can_msgs::CanFrame::ConstPtr &msg);

    template<typename T>
    void sendData(const uint16_t id, const T data);

    ros::NodeHandle _nh;
    ros::Publisher _can_tx_pub;
    ros::Subscriber _can_rx_sub;

    ros::Subscriber	_motor_id_sub;
    ros::Subscriber	_motor_cmd_sub;
    ros::Subscriber _motor_cmd_vel_sub;
    ros::Publisher _motor_status_pub;

    uint16_t id_motor_cmd         = 0x4f0;
    uint16_t id_motor_cmd_vel     = 0x4f1;
    uint16_t id_motor_status      = 0x4f3;
};

CanNode::CanNode(void)
{
    _can_tx_pub				    = _nh.advertise<can_msgs::CanFrame>("can_tx", 10);
    // _can_rx_sub				    = _nh.subscribe<can_msgs::CanFrame>("can_rx", 10, &CanNode::canRxCallback, this);

    _motor_id_sub			        = _nh.subscribe<std_msgs::UInt16>("motor_id", 10 , &CanNode::motorIDCallback, this);
    _motor_cmd_sub			        = _nh.subscribe<std_msgs::UInt8>("motor_cmd", 10 , &CanNode::motorCmdCallback, this);
    _motor_cmd_vel_sub	            = _nh.subscribe<std_msgs::Float32>("motor_cmd_vel", 10, &CanNode::motorCmdVelCallback, this);
}

void CanNode::motorIDCallback(const std_msgs::UInt16::ConstPtr& msg)
{
    id_motor_cmd = msg->data;
    id_motor_cmd_vel = msg->data + 1;
}

void CanNode::motorCmdCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    this->sendData(id_motor_cmd, msg->data);
}

void CanNode::motorCmdVelCallback(const std_msgs::Float32::ConstPtr& msg)
{
    this->sendData(id_motor_cmd_vel, msg->data);
}

// void Mr1CanNode::canRxCallback(const can_msgs::CanFrame::ConstPtr &msg)
// {
//     std_msgs::UInt16 _launcher_status_msg;
//     std_msgs::UInt16 _base_status_msg;
//     std_msgs::Float64 _base_odom_x_msg;
//     std_msgs::Float64 _base_odom_y_msg;
//     std_msgs::Float64 _base_odom_yaw_msg;
//     std_msgs::UInt8 _base_conf_msg;
//     std_msgs::UInt8 _load_motor_status_msg;
//     std_msgs::UInt8 _expand_motor_status_msg;

//     switch(msg->id)
//     {
//         case id_launcherStatus:
//             can_unpack(msg->data, _launcher_status_msg.data);
//             _launcher_status_pub.publish(_launcher_status_msg);
//             break;

//         case id_baseStatus:
//             can_unpack(msg->data, _base_status_msg.data);
//             _base_status_pub.publish(_base_status_msg);
//             break;

//         case id_baseOdomX:
//             can_unpack(msg->data, _base_odom_x_msg.data);
//             _base_odom_x_pub.publish(_base_odom_x_msg);
//             break;

//         case id_baseOdomY:
//             can_unpack(msg->data, _base_odom_y_msg.data);
//             _base_odom_y_pub.publish(_base_odom_y_msg);
//             break;

//         case id_baseOdomYaw:
//             can_unpack(msg->data, _base_odom_yaw_msg.data);
//             _base_odom_yaw_pub.publish(_base_odom_yaw_msg);
//             break;

//         case id_baseConf:
//             can_unpack(msg->data, _base_conf_msg.data);
//             _base_conf_pub.publish(_base_conf_msg);
//             break;

//         case id_load_motor_status:
//             can_unpack(msg->data, _load_motor_status_msg.data);
//             _load_motor_status_pub.publish(_load_motor_status_msg);
//             break;

//         case id_expand_motor_status:
//             can_unpack(msg->data, _expand_motor_status_msg.data);
//             _expand_motor_status_pub.publish(_expand_motor_status_msg);
//             break;

//         default:
//             break;
//     }
// }

template<typename T>
void CanNode::sendData(const uint16_t id, const T data)
{
    can_msgs::CanFrame frame;
    frame.id = id;
    frame.is_rtr = false;
    frame.is_extended = false;
    frame.is_error = false;

    frame.dlc = sizeof(T);

    can_pack<T>(frame.data, data);

    _can_tx_pub.publish(frame);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "can");
    ROS_INFO("can node has started.");

    CanNode *canNode = new CanNode();

    ros::spin();
}