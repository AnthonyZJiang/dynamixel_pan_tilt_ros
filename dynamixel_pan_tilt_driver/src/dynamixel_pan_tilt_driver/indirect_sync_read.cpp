#include "dynamixel_pan_tilt_driver/indirect_sync_read.h"
#include "dynamixel_pan_tilt_driver/pan_tilt_exceptions.h"

IndirectSyncRead::IndirectSyncRead(dynamixel::PortHandler *port_handler, dynamixel::PacketHandler *packet_handler, uint16_t addr_indirect_address, uint16_t addr_indirect_data)
{
    port_handler_ = port_handler;
    packet_handler_ = packet_handler;
    addr_indirect_address_ = addr_indirect_address;
    addr_indirect_data_ = addr_indirect_data;
    data_length_ = 0;
}

// IndirectSyncRead::~IndirectSyncRead()
// {
//     clear();
// }

void IndirectSyncRead::clear()
{
    params_.clear();
    ids_.clear();
    address_table_.clear();
    sync_read_handler_.reset();
}

void IndirectSyncRead::addParam(uint16_t address, uint8_t length)
{
    params_.push_back(std::make_pair(address, length));
}

void IndirectSyncRead::addParamList(std::vector<std::pair<uint16_t, uint8_t> > params)
{
    params_.insert(params_.end(), params.begin(), params.end());
}

void IndirectSyncRead::addId(uint8_t id)
{
    ids_.push_back(id);
}

void IndirectSyncRead::addIdList(std::vector<uint8_t> ids)
{
    ids_.insert(ids_.end(), ids.begin(), ids.end());
}

int IndirectSyncRead::initiate()
{
    if (ids_.empty() || params_.empty())
    {
        throw IndirectSyncIOException("No ID or parameter added");
    }
    data_length_ = 0;
    uint16_t write_address = addr_indirect_address_;
    uint16_t read_address = addr_indirect_data_;
    for (std::vector<std::pair<uint16_t, uint8_t> >::iterator it = params_.begin(); it != params_.end(); ++it)
    {
        uint16_t param_address = it->first;
        uint8_t param_length = it->second;
        std::vector<uint16_t> param_val = {read_address, param_length};
        address_table_[param_address] = param_val;
        ROS_DEBUG("IndirectSyncRead::initiate: Address table: added read address: %d with length: %d for param address: %d", read_address, param_length, param_address);
        for (uint8_t i = 0; i < param_length; i++)
        {
            for (std::vector<uint8_t>::iterator it_id = ids_.begin(); it_id != ids_.end(); ++it_id)
            {   
                int result = packet_handler_->write2ByteTxRx(port_handler_, *it_id, write_address, param_address + i);
                if (result != COMM_SUCCESS)
                {
                    return result;
                }
                ROS_DEBUG("IndirectSyncRead::initiate: Wrote value: %d to address: %d", param_address + i, write_address);
            }
            write_address += 2;
        }
        data_length_ += param_length;
        read_address += param_length;
    }
    getSyncReadHandler();
    params_.clear();
    return 0;
}

int IndirectSyncRead::txRxPacket()
{
    return sync_read_handler_->txRxPacket();
}

bool IndirectSyncRead::isAvailable(uint8_t id, uint16_t address)
{
    return sync_read_handler_->isAvailable(id, address_table_[address][0], address_table_[address][1]);
}

bool IndirectSyncRead::isAvailable(uint8_t id)
{
    for (std::unordered_map<uint16_t, std::vector<uint16_t> >::iterator it = address_table_.begin(); it != address_table_.end(); ++it)
    {
        return sync_read_handler_->isAvailable(id, it->second[0], it->second[1]);
    }
    return false;
}

bool IndirectSyncRead::isAvailable()
{
    for (std::vector<uint8_t>::iterator it = ids_.begin(); it != ids_.end(); ++it)
    {
        return isAvailable(*it);
    }
    return false;
}

int32_t IndirectSyncRead::getData(uint8_t id, uint16_t address)
{
    // ROS_INFO("Getting data from ID %d, address %d", id, address);
    if (address_table_.find(address) == address_table_.end())
    {
        throw IndirectSyncIOException("Address " + std::to_string(address) + " does not exist.");
    }
    uint32_t val = sync_read_handler_->getData(id, address_table_[address][0], address_table_[address][1]);
    // deal with 2's complement
    // ROS_INFO("Data: %d", val);
    if (address_table_[address][1] == 2 && val > 0x7FFF)
    {
        val = val - 0x10000;
    }
    else if (address_table_[address][1] == 4 && val > 0x7FFFFFFF)
    {
        val = val - 0x100000000;
    }
    return val;
}

void IndirectSyncRead::getSyncReadHandler()
{
    sync_read_handler_ = std::make_unique<dynamixel::GroupSyncRead>(port_handler_, packet_handler_, addr_indirect_data_, data_length_);
    for (std::vector<uint8_t>::iterator it = ids_.begin(); it != ids_.end(); ++it)
    {
        if (!sync_read_handler_->addParam(*it))
        {
            throw IndirectSyncIOException("Failed to get sync read handler: ID " + std::to_string(*it) + " does not exist.");
        }
    }
    // ROS_INFO("Sync read handler initiated with data length %d, address %d", data_length_, addr_indirect_data_);
}
