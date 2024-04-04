#pragma once

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_pan_tilt_msgs/PanTiltCmd.h>
#include "dynamixel_pan_tilt_driver/indirect_sync_read.h"
#include "dynamixel_pan_tilt_driver/indirect_sync_write.h"

#define ADDR_OPERATING_MODE 11
#define ADDR_MAX_POSITION_LIMIT 48
#define ADDR_MIN_POSITION_LIMIT 52
#define ADDR_TORQUE_ENABLE 64
#define ADDR_HARDWARE_ERROR_STATUS 70
#define ADDR_PROFILE_ACCELERATION 108
#define ADDR_PROFILE_VELOCITY 112
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_LOAD 126
#define ADDR_PRESENT_VELOCITY 128
#define ADDR_PRESENT_POSITION 132
#define ADDR_INDIRECT_ADDRESS_START 168

#define LEN_OPERATING_MODE 1
#define LEN_MAX_POSITION_LIMIT 4
#define LEN_MIN_POSITION_LIMIT 4
#define LEN_TORQUE_ENABLE 1
#define LEN_HARDWARE_ERROR_STATUS 1
#define LEN_PROFILE_ACCELERATION 4
#define LEN_PROFILE_VELOCITY 4
#define LEN_GOAL_POSITION 4
#define LEN_PRESENT_LOAD 2
#define LEN_PRESENT_VELOCITY 4
#define LEN_PRESENT_POSITION 4

#define VAL_TORQUE_ENABLE 1
#define VAL_TORQUE_DISABLE 0
#define VAL_OPERATING_MODE_POSITION_CONTROL 3

#define ERNUM_HW_INPUT_VOLTAGE 1
#define ERNUM_HW_OVERHEATING 4
#define ERNUM_HW_MOTOR_ENCODER 8
#define ERNUM_HW_ELECTRIONICAL_SHOCK 16
#define ERNUM_HW_OVERLOAD 32


struct JointStatus
{
    ros::Time timestamp;
    bool online;
    uint16_t goal_position;
    uint16_t present_position;
    int16_t present_velocity;
    int16_t present_load;
    bool torque_enable;
    uint8_t hw_error;
    bool hw_error_overload;
    bool hw_error_overheating;
    bool hw_error_input_voltage;
    bool hw_error_electronical_shock;
    bool hw_error_motor_encoder;
};

struct ServoParams
{
    uint8_t id;
    uint16_t profile_velocity_default;
    uint16_t profile_acceleration;
    uint16_t position_max;
    uint16_t position_min;
    uint16_t position_home;
};

class PanTiltController
{
public:
    PanTiltController(ros::NodeHandle &node, ros::NodeHandle &private_nodehandle);
    ~PanTiltController();

private:
    ros::NodeHandle priv_nh;
    ros::Subscriber panTiltCmdVelSub;
    ros::Subscriber panTiltCmdIncSub;
    ros::Subscriber panTiltCmdPosSub;
    ros::Publisher panTiltStatusPub;
    ros::Timer periodicUpdateTimer;
    ros::ServiceServer panTiltCmdService;
    ros::ServiceServer softRebootService;
    diagnostic_updater::Updater diagnostics;

    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;
    std::unique_ptr<dynamixel::GroupSyncWrite> syncWriteOperatingModeHandler;
    std::unique_ptr<IndirectSyncRead> indirectSyncRead;
    std::unique_ptr<IndirectSyncWrite> indirectSyncWrite;

    bool broadcast_joint_status;
    
    // dynamixel::GroupSyncWrite syncWriteOperatingModeHandler;
    // IndirectSyncRead indirectSyncRead;
    // IndirectSyncWrite indirectSyncWrite;


    std::string portName;
    int baudRate;
    uint16_t addrIndirectAddressToWrite;
    uint16_t addrIndirectAddressDataOffset;
    bool enablePresentVelocity;

    JointStatus panStatus;
    JointStatus tiltStatus;
    ServoParams panParams;
    ServoParams tiltParams;

    void init();
    void home();
    void stop();
    void updateJointStatus();
    void updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

    void panTiltCmdCallback(const dynamixel_pan_tilt_msgs::PanTiltCmd::ConstPtr &msg);
    void panTiltCmdIncrementCallback(const dynamixel_pan_tilt_msgs::PanTiltCmd::ConstPtr &msg);
    void panTiltCmdPositionCallback(const dynamixel_pan_tilt_msgs::PanTiltCmd::ConstPtr &msg);
    void periodicUpdateCallback(const ros::TimerEvent &event);
    bool homeCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool softRebootCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool isOK();

    void setTorqueEnable(uint8_t id, bool enable);
    void setOperatingMode(uint8_t id, uint8_t mode);

    void writeNByteTxRx(uint8_t id, uint16_t address, uint16_t length, uint32_t data);
};