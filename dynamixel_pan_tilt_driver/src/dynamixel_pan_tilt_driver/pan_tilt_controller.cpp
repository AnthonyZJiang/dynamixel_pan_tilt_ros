#include "dynamixel_pan_tilt_driver/pan_tilt_controller.h"
#include <dynamixel_pan_tilt_msgs/JointStatus.h>

std::string bool_to_str(bool b)
{
    return b ? "True" : "False";
}

PanTiltController::PanTiltController(ros::NodeHandle &node, ros::NodeHandle &private_nodehandle) :
    priv_nh(private_nodehandle), diagnostics(node, private_nodehandle, ros::this_node::getName())
{
    ROS_INFO_NAMED("PanTiltDriver", "Pan Tilt Controller initializing");
    panStatus = JointStatus();
    tiltStatus = JointStatus();
    panParams = ServoParams();
    tiltParams = ServoParams();

    ROS_INFO_NAMED("PanTiltDriver", "Pan Tilt Controller parameters:");
    int val;
    priv_nh.param<std::string>("port_name", portName, "/dev/ttyUSB0");
    priv_nh.param<int>("baud_rate", baudRate, 1000000);
    priv_nh.param<bool>("enable_present_velocity", enablePresentVelocity, true);
    priv_nh.param<int>("addr_indirect_address_data_offset", val, 56);
    addrIndirectAddressDataOffset = val;
    priv_nh.param<int>("dxl_pan_id", val, 0);
    panParams.id = val;
    priv_nh.param<int>("dxl_tilt_id", val, 1);
    tiltParams.id = val;
    priv_nh.param<int>("dxl_pan_profile_velocity_default", val, 100);
    panParams.profile_velocity_default = val;
    priv_nh.param<int>("dxl_tilt_profile_velocity_default", val, 100);
    tiltParams.profile_velocity_default = val;
    priv_nh.param<int>("dxl_pan_profile_acceleration", val, 100);
    panParams.profile_acceleration = val;
    priv_nh.param<int>("dxl_tilt_profile_acceleration", val, 100);
    tiltParams.profile_acceleration = val;
    priv_nh.param<int>("dxl_pan_position_max", val, 3300);
    panParams.position_max = val;
    priv_nh.param<int>("dxl_tilt_position_max", val, 3300);
    tiltParams.position_max = val;
    priv_nh.param<int>("dxl_pan_position_min", val, 796);
    panParams.position_min = val;
    priv_nh.param<int>("dxl_tilt_position_min", val, 796);
    tiltParams.position_min = val;
    priv_nh.param<int>("dxl_pan_position_home", val, 2048);
    panParams.position_home = val;
    priv_nh.param<int>("dxl_tilt_position_home", val, 2048);
    tiltParams.position_home = val;
    priv_nh.param<bool>("broadcast_joint_status", broadcast_joint_status, false);


    ROS_INFO_NAMED("PanTiltDriver", "| port_name: %s", portName.c_str());
    ROS_INFO_NAMED("PanTiltDriver", "| baud_rate: %d", baudRate);
    ROS_INFO_NAMED("PanTiltDriver", "| enable_present_velocity: %s", enablePresentVelocity ? "true" : "false");
    ROS_INFO_NAMED("PanTiltDriver", "| addr_indirect_address_data_offset: %d", addrIndirectAddressDataOffset);
    ROS_INFO_NAMED("PanTiltDriver", "| pan joint (ID %d):", panParams.id);
    ROS_INFO_NAMED("PanTiltDriver", "| - profile_velocity_default: %d", panParams.profile_velocity_default);
    ROS_INFO_NAMED("PanTiltDriver", "| - profile_acceleration: %d", panParams.profile_acceleration);
    ROS_INFO_NAMED("PanTiltDriver", "| - position_max: %d", panParams.position_max);
    ROS_INFO_NAMED("PanTiltDriver", "| - position_min: %d", panParams.position_min);
    ROS_INFO_NAMED("PanTiltDriver", "| - position_home: %d", panParams.position_home);
    ROS_INFO_NAMED("PanTiltDriver", "| tilt joint (ID %d):", tiltParams.id);
    ROS_INFO_NAMED("PanTiltDriver", "| - profile_velocity_default: %d", tiltParams.profile_velocity_default);
    ROS_INFO_NAMED("PanTiltDriver", "| - profile_acceleration: %d", tiltParams.profile_acceleration);
    ROS_INFO_NAMED("PanTiltDriver", "| - position_max: %d", tiltParams.position_max);
    ROS_INFO_NAMED("PanTiltDriver", "| - position_min: %d", tiltParams.position_min);
    ROS_INFO_NAMED("PanTiltDriver", "| - position_home: %d", tiltParams.position_home);

    portHandler = dynamixel::PortHandler::getPortHandler(portName.c_str());
    packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);
    portHandler->setBaudRate(baudRate);
    portHandler->openPort();

    
    init();

    panTiltCmdVelSub = priv_nh.subscribe("cmd_vel", 1, &PanTiltController::panTiltCmdCallback, this);
    panTiltCmdIncSub = priv_nh.subscribe("cmd_inc", 1, &PanTiltController::panTiltCmdIncrementCallback, this);
    panTiltCmdPosSub = priv_nh.subscribe("cmd_pos", 1, &PanTiltController::panTiltCmdPositionCallback, this);
    if (broadcast_joint_status)
    {
        panTiltStatusPub = priv_nh.advertise<dynamixel_pan_tilt_msgs::JointStatus>("status", 1);
    }
    periodicUpdateTimer = priv_nh.createTimer(ros::Duration(0.1), &PanTiltController::periodicUpdateCallback, this);
    panTiltCmdService = priv_nh.advertiseService("home", &PanTiltController::homeCallback, this);
    softRebootService = priv_nh.advertiseService("soft_reboot", &PanTiltController::softRebootCallback, this);

    ROS_INFO_NAMED("PanTiltDriver", "Pan Tilt Controller initialized");
}

PanTiltController::~PanTiltController()
{
    portHandler->closePort();
}

void PanTiltController::init()
{
    indirectSyncRead = std::make_unique<IndirectSyncRead>(portHandler, packetHandler, ADDR_INDIRECT_ADDRESS_START, ADDR_INDIRECT_ADDRESS_START + addrIndirectAddressDataOffset);
    indirectSyncRead->addParam(ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD);
    if (enablePresentVelocity)
        indirectSyncRead->addParam(ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
    indirectSyncRead->addParam(ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    indirectSyncRead->addParam(ADDR_HARDWARE_ERROR_STATUS, LEN_HARDWARE_ERROR_STATUS);
    indirectSyncRead->addParam(ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
    indirectSyncRead->addParam(ADDR_TORQUE_ENABLE, LEN_TORQUE_ENABLE);
    indirectSyncRead->addId(panParams.id);
    indirectSyncRead->addId(tiltParams.id);
    indirectSyncRead->initiate();

    uint16_t readLength = indirectSyncRead->getDataLength();
    assert (readLength*2 <= addrIndirectAddressDataOffset);
    
    // reuse the goal position and torque enable addresses for sync write.
    uint16_t addrToWrite = (readLength - LEN_TORQUE_ENABLE - LEN_GOAL_POSITION) * 2 + ADDR_INDIRECT_ADDRESS_START;
    uint16_t addrToWriteData = addrIndirectAddressDataOffset + (addrToWrite + ADDR_INDIRECT_ADDRESS_START)/2;

    indirectSyncWrite = std::make_unique<IndirectSyncWrite>(portHandler, packetHandler, addrToWrite, addrToWriteData);
    indirectSyncRead->addParam(ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD);
    if (enablePresentVelocity)
        indirectSyncRead->addParam(ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
    indirectSyncRead->addParam(ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    indirectSyncRead->addParam(ADDR_HARDWARE_ERROR_STATUS, LEN_HARDWARE_ERROR_STATUS);
    indirectSyncRead->addParam(ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
    indirectSyncWrite->addParam(ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
    indirectSyncWrite->addParam(ADDR_TORQUE_ENABLE, LEN_TORQUE_ENABLE);
    indirectSyncWrite->addParam(ADDR_PROFILE_VELOCITY, LEN_PRESENT_VELOCITY);
    indirectSyncWrite->addId(panParams.id);
    indirectSyncWrite->addId(tiltParams.id);
    indirectSyncWrite->initiate();

    assert (addrToWrite + indirectSyncWrite->getDataLength() * 2 <= addrIndirectAddressDataOffset + ADDR_INDIRECT_ADDRESS_START);

    setTorqueEnable(panParams.id, false);
    setOperatingMode(panParams.id, VAL_OPERATING_MODE_POSITION_CONTROL);
    writeNByteTxRx(panParams.id, ADDR_MAX_POSITION_LIMIT, LEN_MAX_POSITION_LIMIT, panParams.position_max);
    writeNByteTxRx(panParams.id, ADDR_MIN_POSITION_LIMIT, LEN_MIN_POSITION_LIMIT, panParams.position_min);
    writeNByteTxRx(panParams.id, ADDR_PROFILE_ACCELERATION, LEN_PROFILE_ACCELERATION, panParams.profile_acceleration);

    setTorqueEnable(tiltParams.id, false);
    setOperatingMode(tiltParams.id, VAL_OPERATING_MODE_POSITION_CONTROL);
    writeNByteTxRx(tiltParams.id, ADDR_MAX_POSITION_LIMIT, LEN_MAX_POSITION_LIMIT, tiltParams.position_max);
    writeNByteTxRx(tiltParams.id, ADDR_MIN_POSITION_LIMIT, LEN_MIN_POSITION_LIMIT, tiltParams.position_min);
    writeNByteTxRx(tiltParams.id, ADDR_PROFILE_ACCELERATION, LEN_PROFILE_ACCELERATION, tiltParams.profile_acceleration);

    diagnostics.setHardwareID("dxl_pan_tilt");
    diagnostics.add("update_diagnostics", this, &PanTiltController::updateDiagnostics);

    ROS_DEBUG_NAMED("PanTiltDriver", "SyncRead starting: %d, length: %d, data address: %d", ADDR_INDIRECT_ADDRESS_START, readLength, ADDR_INDIRECT_ADDRESS_START + addrIndirectAddressDataOffset);
    ROS_DEBUG_NAMED("PanTiltDriver", "SyncWrite starting: %d, length: %d, data address: %d", addrToWrite, indirectSyncWrite->getDataLength(), addrToWriteData);
}

void PanTiltController::home()
{
    
}

void PanTiltController::updateJointStatus()
{
    uint8_t hw_error;
    int result = indirectSyncRead->txRxPacket();
    if (result != COMM_SUCCESS)
    {
        ROS_ERROR_THROTTLE_NAMED(1, "PanTiltDriver", "Failed to send dynamixel pan tilt status request: %s", packetHandler->getTxRxResult(result));
    }
    if (!indirectSyncRead->isAvailable())
    {
        ROS_ERROR_THROTTLE_NAMED(1, "PanTiltDriver", "Failed to read dynamixel pan tilt status: not all data available");
    }
    ros::Time now = ros::Time::now();
    panStatus.timestamp = now;
    panStatus.online = packetHandler->ping(portHandler, panParams.id) == COMM_SUCCESS;
    if (panStatus.online)
    {
        panStatus.present_position = indirectSyncRead->getData(panParams.id, ADDR_PRESENT_POSITION);
        panStatus.present_velocity = enablePresentVelocity ? indirectSyncRead->getData(panParams.id, ADDR_PRESENT_VELOCITY) : 0;
        panStatus.present_load = indirectSyncRead->getData(panParams.id, ADDR_PRESENT_LOAD);
        panStatus.torque_enable = indirectSyncRead->getData(panParams.id, ADDR_TORQUE_ENABLE) == VAL_TORQUE_ENABLE;
        panStatus.goal_position = indirectSyncRead->getData(panParams.id, ADDR_GOAL_POSITION);
        hw_error = indirectSyncRead->getData(panParams.id, ADDR_HARDWARE_ERROR_STATUS);
        panStatus.hw_error = hw_error;
        panStatus.hw_error_overload = hw_error & ERNUM_HW_OVERLOAD;
        panStatus.hw_error_overheating = hw_error & ERNUM_HW_OVERHEATING;
        panStatus.hw_error_input_voltage = hw_error & ERNUM_HW_INPUT_VOLTAGE;
        panStatus.hw_error_electronical_shock = hw_error & ERNUM_HW_ELECTRIONICAL_SHOCK;
        panStatus.hw_error_motor_encoder = hw_error & ERNUM_HW_MOTOR_ENCODER;
    }

    tiltStatus.timestamp = now;
    tiltStatus.online = packetHandler->ping(portHandler, tiltParams.id) == COMM_SUCCESS;
    if (tiltStatus.online)
    {
        tiltStatus.present_position = indirectSyncRead->getData(tiltParams.id, ADDR_PRESENT_POSITION);
        tiltStatus.present_velocity = enablePresentVelocity ? indirectSyncRead->getData(panParams.id, ADDR_PRESENT_VELOCITY) : 0;
        tiltStatus.present_load = indirectSyncRead->getData(tiltParams.id, ADDR_PRESENT_LOAD);
        tiltStatus.torque_enable = indirectSyncRead->getData(tiltParams.id, ADDR_TORQUE_ENABLE) == VAL_TORQUE_ENABLE;
        tiltStatus.goal_position = indirectSyncRead->getData(tiltParams.id, ADDR_GOAL_POSITION);
        hw_error = indirectSyncRead->getData(tiltParams.id, ADDR_HARDWARE_ERROR_STATUS);
        tiltStatus.hw_error = hw_error;
        tiltStatus.hw_error_overload = hw_error & ERNUM_HW_OVERLOAD;
        tiltStatus.hw_error_overheating = hw_error & ERNUM_HW_OVERHEATING;
        tiltStatus.hw_error_electronical_shock = hw_error & ERNUM_HW_ELECTRIONICAL_SHOCK;
        tiltStatus.hw_error_input_voltage = hw_error & ERNUM_HW_INPUT_VOLTAGE;
        tiltStatus.hw_error_motor_encoder = hw_error & ERNUM_HW_MOTOR_ENCODER;
    }
    isOK();
}

void PanTiltController::periodicUpdateCallback(const ros::TimerEvent &)
{
    updateJointStatus();
    diagnostics.update();

    if (!broadcast_joint_status)
    {
        return;
    }
    dynamixel_pan_tilt_msgs::JointStatus msg;
    msg.header.stamp = panStatus.timestamp;
    msg.online = {panStatus.online, tiltStatus.online};
    msg.goal_position = {panStatus.goal_position, tiltStatus.goal_position};
    msg.present_position = {panStatus.present_position, tiltStatus.present_position};
    msg.present_velocity = {panStatus.present_velocity, tiltStatus.present_velocity};
    msg.present_load = {panStatus.present_load, tiltStatus.present_load};
    msg.torque_enable = {panStatus.torque_enable, tiltStatus.torque_enable};
    msg.hw_error_overload = {panStatus.hw_error_overload, tiltStatus.hw_error_overload};
    msg.hw_error_overheating = {panStatus.hw_error_overheating, tiltStatus.hw_error_overheating};
    msg.hw_error_input_voltage = {panStatus.hw_error_input_voltage, tiltStatus.hw_error_input_voltage};
    msg.hw_error_electronical_shock = {panStatus.hw_error_electronical_shock, tiltStatus.hw_error_electronical_shock};
    msg.hw_error_motor_encoder = {panStatus.hw_error_motor_encoder, tiltStatus.hw_error_motor_encoder};
    panTiltStatusPub.publish(msg);
}

void PanTiltController::updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    if (!panStatus.online || !tiltStatus.online) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Pan tilt joint offline.");
    } else
    if (panStatus.hw_error > 0 || tiltStatus.hw_error > 0) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Pan tilt joint hardware error.");
    } else
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
    }
    stat.addf("online", "pan: %s, tilt: %s", bool_to_str(tiltStatus.online), bool_to_str(panStatus.online));
    stat.addf("goal_position", "pan: %d, tilt: %d", panStatus.goal_position, tiltStatus.goal_position);
    stat.addf("present_position", "pan: %d, tilt: %d", panStatus.present_position, tiltStatus.present_position);
    stat.addf("present_velocity", "pan: %d, tilt: %d", panStatus.present_velocity, tiltStatus.present_velocity);
    stat.addf("present_load", "pan: %d, tilt: %d", panStatus.present_load, tiltStatus.present_load);
    stat.addf("torque_enable", "pan: %s, tilt: %s", bool_to_str(panStatus.torque_enable), bool_to_str(tiltStatus.torque_enable));
    stat.addf("hw_error_overload", "pan: %s, tilt: %s", bool_to_str(panStatus.hw_error_overload), bool_to_str(tiltStatus.hw_error_overload));
    stat.addf("hw_error_overheating", "pan: %s, tilt: %s", bool_to_str(panStatus.hw_error_overheating), bool_to_str(tiltStatus.hw_error_overheating));
    stat.addf("hw_error_input_voltage", "pan: %s, tilt: %s", bool_to_str(panStatus.hw_error_input_voltage), bool_to_str(tiltStatus.hw_error_input_voltage));
    stat.addf("hw_error_electronical_shock", "pan: %s, tilt: %s", bool_to_str(panStatus.hw_error_electronical_shock), bool_to_str(tiltStatus.hw_error_electronical_shock));
    stat.addf("hw_error_motor_encoder", "pan: %s, tilt: %s", bool_to_str(panStatus.hw_error_motor_encoder), bool_to_str(tiltStatus.hw_error_motor_encoder));
}

void PanTiltController::panTiltCmdCallback(const dynamixel_pan_tilt_msgs::PanTiltCmd::ConstPtr &msg)
{
    if (!isOK())
    {
        return;
    }
    int pan_val = std::ceil(msg->pan_val);
    int tilt_val = std::ceil(msg->tilt_val);
    if (pan_val < 0)
    {
        indirectSyncWrite->setData(panParams.id, ADDR_GOAL_POSITION, panParams.position_max);
    }
    else if (pan_val > 0)
    {
        indirectSyncWrite->setData(panParams.id, ADDR_GOAL_POSITION, panParams.position_min);
    }
    else
    {
        indirectSyncWrite->setData(panParams.id, ADDR_GOAL_POSITION, panStatus.present_position);
    }
    if (tilt_val < 0)
    {
        indirectSyncWrite->setData(tiltParams.id, ADDR_GOAL_POSITION, panParams.position_max);
    }
    else if (tilt_val > 0)
    {
        indirectSyncWrite->setData(tiltParams.id, ADDR_GOAL_POSITION, panParams.position_min);
    }
    else
    {
        indirectSyncWrite->setData(tiltParams.id, ADDR_GOAL_POSITION, tiltStatus.present_position);
    }
    indirectSyncWrite->setData(panParams.id, ADDR_PROFILE_VELOCITY, abs(pan_val));
    indirectSyncWrite->setData(tiltParams.id, ADDR_PROFILE_VELOCITY, abs(tilt_val));
    indirectSyncWrite->setData(panParams.id, ADDR_TORQUE_ENABLE, VAL_TORQUE_ENABLE);
    indirectSyncWrite->setData(tiltParams.id, ADDR_TORQUE_ENABLE, VAL_TORQUE_ENABLE);

    ROS_DEBUG_NAMED("PanTiltDriver", "Pan Tilt Controller: pan_val: %d, tilt_val: %d", int(pan_val), int(tilt_val));

    indirectSyncWrite->clearTxQueue();
    int result = indirectSyncWrite->txPacket();
    if (result != COMM_SUCCESS)
    {
        ROS_ERROR_NAMED("PanTiltDriver", "Failed to transmit dynamixel pan tilt command: %s", packetHandler->getTxRxResult(result));
    }
}

void PanTiltController::panTiltCmdIncrementCallback(const dynamixel_pan_tilt_msgs::PanTiltCmd::ConstPtr &msg)
{
    if (!isOK())
    {
        return;
    }
    uint16_t pos = panStatus.present_position + msg->pan_val;
    pos = std::min(std::max(pos, panParams.position_min), panParams.position_max);
    indirectSyncWrite->setData(panParams.id, ADDR_GOAL_POSITION, pos);
    pos = tiltStatus.present_position + msg->tilt_val;
    pos = std::min(std::max(pos, tiltParams.position_min), tiltParams.position_max);
    indirectSyncWrite->setData(tiltParams.id, ADDR_GOAL_POSITION, pos);
    //TODO handle zeros!
    indirectSyncWrite->setData(panParams.id, ADDR_PROFILE_VELOCITY, abs(std::ceil((msg->pan_val))));
    indirectSyncWrite->setData(tiltParams.id, ADDR_PROFILE_VELOCITY, abs(std::ceil((msg->tilt_val))));
    indirectSyncWrite->setData(panParams.id, ADDR_TORQUE_ENABLE, VAL_TORQUE_ENABLE);
    indirectSyncWrite->setData(tiltParams.id, ADDR_TORQUE_ENABLE, VAL_TORQUE_ENABLE);

    indirectSyncWrite->clearTxQueue();
    int result = indirectSyncWrite->txPacket();
    if (result != COMM_SUCCESS)
    {
        ROS_ERROR_NAMED("PanTiltDriver", "Failed to transmit dynamixel pan tilt command: %s", packetHandler->getTxRxResult(result));
    }

}

void PanTiltController::panTiltCmdPositionCallback(const dynamixel_pan_tilt_msgs::PanTiltCmd::ConstPtr &msg)
{
    if (!isOK())
    {
        return;
    }
    if (msg->pan_val > 1 || msg->pan_val < 0 || msg->tilt_val > 1 || msg->tilt_val < 0)
    {
        ROS_ERROR_NAMED("PanTiltDriver", "pan_val or tilt_val must be a value between 0 and 1.");
        return;
    }
    uint16_t pos = msg->pan_val * (panParams.position_max - panParams.position_min) + panParams.position_min;
    indirectSyncWrite->setData(panParams.id, ADDR_GOAL_POSITION, int(round(pos)));
    pos = msg->tilt_val * (tiltParams.position_max - tiltParams.position_min) + tiltParams.position_min;
    indirectSyncWrite->setData(tiltParams.id, ADDR_GOAL_POSITION, int(round(pos)));

    indirectSyncWrite->setData(panParams.id, ADDR_PROFILE_VELOCITY, panParams.profile_velocity_default);
    indirectSyncWrite->setData(tiltParams.id, ADDR_PROFILE_VELOCITY, tiltParams.profile_velocity_default);
    indirectSyncWrite->setData(panParams.id, ADDR_TORQUE_ENABLE, VAL_TORQUE_ENABLE);
    indirectSyncWrite->setData(tiltParams.id, ADDR_TORQUE_ENABLE, VAL_TORQUE_ENABLE);

    indirectSyncWrite->clearTxQueue();
    int result = indirectSyncWrite->txPacket();
    if (result != COMM_SUCCESS)
    {
        ROS_ERROR_NAMED("PanTiltDriver", "Failed to transmit dynamixel pan tilt command: %s", packetHandler->getTxRxResult(result));
    }
}

bool PanTiltController::homeCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if (!isOK())
    {
        res.success = false;
        res.message = "Dynamixel pan tilt is in error.";
        return true;
    }
    indirectSyncWrite->setData(panParams.id, ADDR_GOAL_POSITION, panParams.position_home);
    indirectSyncWrite->setData(tiltParams.id, ADDR_GOAL_POSITION, tiltParams.position_home);

    indirectSyncWrite->setData(panParams.id, ADDR_PROFILE_VELOCITY, panParams.profile_velocity_default);
    indirectSyncWrite->setData(tiltParams.id, ADDR_PROFILE_VELOCITY, tiltParams.profile_velocity_default);
    indirectSyncWrite->setData(panParams.id, ADDR_TORQUE_ENABLE, VAL_TORQUE_ENABLE);
    indirectSyncWrite->setData(tiltParams.id, ADDR_TORQUE_ENABLE, VAL_TORQUE_ENABLE);

    indirectSyncWrite->clearTxQueue();
    int result = indirectSyncWrite->txPacket();
    if (result != COMM_SUCCESS)
    {
        std::string msg = "Failed to transmit dynamixel pan tilt home command: " + std::string(packetHandler->getTxRxResult(result));
        ROS_ERROR_STREAM_NAMED("PanTiltDriver", msg);
        res.success = false;
        res.message = msg;
        return true;
    }
    res.success = true;
    return true;
}

bool PanTiltController::softRebootCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if (!isOK())
    {
        res.success = false;
        res.message = "Dynamixel pan tilt is in error.";
        return true;
    }
    uint8_t error;
    int result = packetHandler->reboot(portHandler, panParams.id, &error);
    if (result != COMM_SUCCESS)
    {
        std::string msg = "Failed to reboot pan servo: " + std::string(packetHandler->getTxRxResult(result)) + " | " + std::string(packetHandler->getRxPacketError(error))+  ". ";
        ROS_ERROR_STREAM_NAMED("PanTiltDriver", msg);
        res.success = false;
        res.message = msg;
    }
    result = packetHandler->reboot(portHandler, tiltParams.id, &error);
    if (result != COMM_SUCCESS)
    {
        std::string msg = "Failed to reboot tilt servo: " + std::string(packetHandler->getTxRxResult(result)) + " | " + std::string(packetHandler->getRxPacketError(error)) + ". ";
        ROS_ERROR_STREAM_NAMED("PanTiltDriver", msg);
        res.success = false;
        res.message += msg;
    }
    return true;
}

bool PanTiltController::isOK()
{
    bool ok = true;
    if (!panStatus.online)
    {
        ROS_ERROR_THROTTLE_NAMED(1, "PanTiltDriver", "Pan servo is not online");
        ok = false;
    }
    else if (panStatus.hw_error > 0)
    {
        ROS_ERROR_THROTTLE_NAMED(1, "PanTiltDriver", "Pan servo is in error: %s%s%s%s",
                           panStatus.hw_error_overload ? "overload " : "",
                           panStatus.hw_error_overheating ? "overheating " : "",
                           panStatus.hw_error_electronical_shock ? "electronical_shock " : "",
                           panStatus.hw_error_motor_encoder ? "motor_encoder " : "");
        ok = false;
    }
    if (!tiltStatus.online)
    {
        ROS_ERROR_THROTTLE_NAMED(1, "PanTiltDriver", "Tilt servo is not online");
        ok = false;
    }
    else if (tiltStatus.hw_error > 0)
    {
        ROS_ERROR_THROTTLE_NAMED(1, "PanTiltDriver", "Tilt servo is in error: %s%s%s%s",
                           tiltStatus.hw_error_overload ? "overload " : "",
                           tiltStatus.hw_error_overheating ? "overheating " : "",
                           tiltStatus.hw_error_electronical_shock ? "electronical_shock " : "",
                           tiltStatus.hw_error_motor_encoder ? "motor_encoder " : "");
        ok = false;
    }
    return ok;
}

void PanTiltController::setTorqueEnable(uint8_t id, bool enable)
{
    writeNByteTxRx(id, ADDR_TORQUE_ENABLE, LEN_TORQUE_ENABLE, enable ? VAL_TORQUE_ENABLE : VAL_TORQUE_DISABLE);
}

void PanTiltController::setOperatingMode(uint8_t id, uint8_t mode)
{
    writeNByteTxRx(id, ADDR_OPERATING_MODE, LEN_OPERATING_MODE, mode);
}

void PanTiltController::writeNByteTxRx(uint8_t id, uint16_t address, uint16_t length, uint32_t data)
{
    uint8_t error;
    int result;
    if (length == 1)
        result = packetHandler->write1ByteTxRx(portHandler, id, address, data, &error);
    else if (length == 2)
        result = packetHandler->write2ByteTxRx(portHandler, id, address, data, &error);
    else if (length == 4)
        result = packetHandler->write4ByteTxRx(portHandler, id, address, data, &error);
    else
    {
        ROS_ERROR_NAMED("PanTiltDriver", "Invalid length %d while writing value %d to address %d", length, data, address);
        return;
    }
    if (result != COMM_SUCCESS)
    {
        ROS_ERROR_NAMED("PanTiltDriver", "Failed to trasmit command while writing value %d to address %d: %s", 
                    data,
                    address,
                    packetHandler->getTxRxResult(result));
    }
    if (error != 0)
    {
        ROS_ERROR_NAMED("PanTiltDriver", "Dynamixel pan tilt error occurred while writing value %d to address %d: %s", 
                    data,
                    address,
                    packetHandler->getRxPacketError(error));
    }
}