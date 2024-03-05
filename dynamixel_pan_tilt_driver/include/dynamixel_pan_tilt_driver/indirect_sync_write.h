#pragma once

#include <ros/ros.h>
#include <bits/stdc++.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

class IndirectSyncWrite
{
public:
    IndirectSyncWrite(dynamixel::PortHandler *port_handler, dynamixel::PacketHandler *packet_handler, 
                        uint16_t addr_indirect_address, uint16_t addr_indirect_data);
    // ~IndirectSyncWrite();

    void clear();
    void addParam(uint16_t address, uint8_t length);
    void addParamList(std::vector<std::pair<uint16_t, uint8_t>> params);
    void addId(uint8_t id);
    void addIdList(std::vector<uint8_t> ids);
    void setData(uint8_t id, uint16_t address, int value);
    int initiate();
    int txPacket();
    void clearTxQueue();
    const uint8_t getDataLength() { return data_length_; }

private:
    void preparePacket();
    void getSyncWriteHandler();

    dynamixel::PortHandler *port_handler_;
    dynamixel::PacketHandler *packet_handler_;
    std::unique_ptr<dynamixel::GroupSyncWrite> sync_write_handler_;
    uint16_t addr_indirect_address_;
    uint16_t addr_indirect_data_;
    uint8_t data_length_;

    std::vector<std::pair<uint16_t, uint8_t>> params_;
    std::vector<uint8_t> ids_;
    std::unordered_map<uint8_t, std::unordered_map<uint16_t, int>> data_;
};
