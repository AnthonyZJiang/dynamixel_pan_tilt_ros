#include <ros/ros.h>
#include <dynamixel_pan_tilt_msgs/PanTiltCmd.h>
#include <dynamixel_pan_tilt_msgs/PanTiltStatus.h>
#include "dynamixel_pan_tilt_driver/indirect_sync_read.h"
#include "dynamixel_pan_tilt_driver/indirect_sync_write.h"
#include "dynamixel_pan_tilt_driver/pan_tilt_controller.h"

dynamixel::PortHandler *port_handler;
dynamixel::PacketHandler *packet_handler;
std::unique_ptr<IndirectSyncRead> indirect_sync_read;
std::unique_ptr<IndirectSyncWrite> indirect_sync_write;
ServoStatus pan_status;
ServoStatus tilt_status;
ros::Publisher pan_tilt_status_pub;
ros::Subscriber pan_tilt_sub;
ros::Timer periodic_update_timer;

void panTiltCallback(const dynamixel_pan_tilt_msgs::PanTiltCmd::ConstPtr &msg)
{
    if (msg->pan_val < 0)
    {
        indirect_sync_write->setData(0, 116, 2500);
    }
    else if (msg->pan_val > 0)
    {
        indirect_sync_write->setData(0, 116, 1500);
    }
    else
    {
        indirect_sync_write->setData(0, 116, pan_status.present_position);
    }
    if (msg->tilt_val < 0)
    {
        indirect_sync_write->setData(1, 116, 2500);
    }
    else if (msg->tilt_val > 0)
    {
        indirect_sync_write->setData(1, 116, 1500);
    }
    else
    {
        indirect_sync_write->setData(1, 116, tilt_status.present_position);
    }
    indirect_sync_write->setData(0, 112, abs(msg->pan_val));
    indirect_sync_write->setData(1, 112, abs(msg->tilt_val));
    int result = indirect_sync_write->txPacket();
    if (result != COMM_SUCCESS)
    {
        ROS_ERROR("Failed to write pan and tilt command: %s", packet_handler->getTxRxResult(result));
    }
}

void periodicUpdate(const ros::TimerEvent &event)
{
    int result = indirect_sync_read->txRxPacket();
    if (result != COMM_SUCCESS)
    {
        ROS_ERROR("Failed to read pan and tilt status: %s", packet_handler->getTxRxResult(result));
        return;
    }
    pan_status.present_position = indirect_sync_read->getData(0, 132);
    tilt_status.present_position = indirect_sync_read->getData(1, 132);
    pan_status.present_velocity = indirect_sync_read->getData(0, 128);
    tilt_status.present_velocity = indirect_sync_read->getData(1, 128);
    pan_status.torque_enable = indirect_sync_read->getData(0, 64);
    tilt_status.torque_enable = indirect_sync_read->getData(1, 64);
    pan_status.goal_position = indirect_sync_read->getData(0, 116);
    tilt_status.goal_position = indirect_sync_read->getData(1, 116);

    dynamixel_pan_tilt_msgs::PanTiltStatus msg;
    msg.pan_goal_position = pan_status.goal_position;
    msg.tilt_goal_position = tilt_status.goal_position;
    msg.pan_present_position = pan_status.present_position;
    msg.tilt_present_position = tilt_status.present_position;
    msg.pan_present_velocity = pan_status.present_velocity;
    msg.tilt_present_velocity = tilt_status.present_velocity;
    msg.pan_torque_enabled = pan_status.torque_enable;
    msg.tilt_torque_enabled = tilt_status.torque_enable;
    pan_tilt_status_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamixel_pan_tilt_driver");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    std::string port_name;
    int baud_rate;
    int dxl_pan_id;
    int dxl_tilt_id;

    priv_nh.param<std::string>("port_name", port_name, "/dev/ttyUSB0");
    priv_nh.param<int>("baud_rate", baud_rate, 1000000);
    priv_nh.param<int>("dxl_pan_id", dxl_pan_id, 0);
    priv_nh.param<int>("dxl_tilt_id", dxl_tilt_id, 1);

    port_handler = dynamixel::PortHandler::getPortHandler(port_name.c_str());
    port_handler->setBaudRate(baud_rate);
    port_handler->openPort();
    packet_handler = dynamixel::PacketHandler::getPacketHandler(2.0);
    indirect_sync_read = std::make_unique<IndirectSyncRead>(IndirectSyncRead(port_handler, packet_handler, 168, 224));
    indirect_sync_write = std::make_unique<IndirectSyncWrite>(IndirectSyncWrite(port_handler, packet_handler, 578, 634));
    ROS_INFO("Port opened");
    indirect_sync_write->addParam(108, 4); // profile acceleration
    indirect_sync_write->addParam(112, 4); // profile velocity
    indirect_sync_write->addParam(116, 4); // goal position
    indirect_sync_write->addParam(64, 1);
    indirect_sync_write->addId(dxl_pan_id);
    indirect_sync_write->addId(dxl_tilt_id);
    indirect_sync_write->setData(dxl_pan_id, 108, 40);
    indirect_sync_write->setData(dxl_tilt_id, 108, 40);
    indirect_sync_write->setData(dxl_pan_id, 64, 1);
    indirect_sync_write->setData(dxl_tilt_id, 64, 1);
    indirect_sync_read->addParam(64, 1); // torque enable
    indirect_sync_read->addParam(128, 4); // present velocity
    indirect_sync_read->addParam(132, 4); // present position
    indirect_sync_read->addParam(116, 4); // goal position
    indirect_sync_read->addId(dxl_pan_id);
    indirect_sync_read->addId(dxl_tilt_id);
    ROS_INFO("Indirect sync read and write added");
    int result;
    try {
        result = indirect_sync_write->initiate();
        if (result == COMM_SUCCESS)
        {
            ROS_INFO("Indirect sync write success");
        }
        else
        {
            ROS_ERROR("Indirect sync write failed, %s", packet_handler->getTxRxResult(result));
        }
        result = indirect_sync_read->initiate();
        if (result == COMM_SUCCESS)
        {
            ROS_INFO("Indirect sync read success");
        }
        else
        {
            ROS_ERROR("Indirect sync read failed, %s", packet_handler->getTxRxResult(result));
        }
    }
    catch(IndirectSyncIOException &e){
        ROS_ERROR("IndirectSyncIOException: %s", e.what());
    }

    pan_tilt_status_pub = nh.advertise<dynamixel_pan_tilt_msgs::PanTiltStatus>("pan_tilt_status", 1);
    periodic_update_timer = nh.createTimer(ros::Duration(0.05), periodicUpdate);
    pan_tilt_sub = nh.subscribe("pan_tilt", 1, panTiltCallback);

    // add subscribers for pan and tilt commands
    ros::spin();
    return 0;
}