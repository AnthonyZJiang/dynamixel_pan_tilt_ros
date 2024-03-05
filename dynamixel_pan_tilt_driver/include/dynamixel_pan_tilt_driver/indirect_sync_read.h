#pragma once

#include <ros/ros.h>
#include <bits/stdc++.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

class IndirectSyncRead
{
public:
    IndirectSyncRead(dynamixel::PortHandler *port_handler, dynamixel::PacketHandler *packet_handler, 
                        uint16_t addr_indirect_address, uint16_t addr_indirect_data);
    // ~IndirectSyncRead();
    
    void clear();
    void addParam(uint16_t address, uint8_t length);
    void addParamList(std::vector<std::pair<uint16_t, uint8_t>> params);
    void addId(uint8_t id);
    void addIdList(std::vector<uint8_t> ids);
    int initiate();
    int txRxPacket();
    bool isAvailable();
    bool isAvailable(uint8_t id);
    bool isAvailable(uint8_t id, uint16_t address);
    int32_t getData(uint8_t id, uint16_t address);
    const uint8_t getDataLength() { return data_length_; }

private:
    void getSyncReadHandler();

    dynamixel::PortHandler *port_handler_;
    dynamixel::PacketHandler *packet_handler_;
    std::unique_ptr<dynamixel::GroupSyncRead> sync_read_handler_;
    uint16_t addr_indirect_address_;
    uint16_t addr_indirect_data_;
    uint8_t data_length_;
    std::unordered_map<uint16_t, std::vector<uint16_t>> address_table_;
    std::vector<std::pair<uint16_t, uint8_t>> params_;
    std::vector<uint8_t> ids_;
};
