#include "dynamixel_pan_tilt_driver/indirect_sync_write.h"
#include "dynamixel_pan_tilt_driver/pan_tilt_exceptions.h"

IndirectSyncWrite::IndirectSyncWrite(dynamixel::PortHandler *port_handler, dynamixel::PacketHandler *packet_handler, uint16_t addr_indirect_address, uint16_t addr_indirect_data)
{
    port_handler_ = port_handler;
    packet_handler_ = packet_handler;
    addr_indirect_address_ = addr_indirect_address;
    addr_indirect_data_ = addr_indirect_data;
    data_length_ = 0;
}

// IndirectSyncWrite::~IndirectSyncWrite()
// {
//     clear();
// }

void IndirectSyncWrite::clear()
{
    params_.clear();
    ids_.clear();
    sync_write_handler_.reset();
}

void IndirectSyncWrite::addParam(uint16_t address, uint8_t length)
{
    params_.push_back(std::make_pair(address, length));
}

void IndirectSyncWrite::addParamList(std::vector<std::pair<uint16_t, uint8_t> > params)
{
    params_.insert(params_.end(), params.begin(), params.end());
}

void IndirectSyncWrite::addId(uint8_t id)
{
    ids_.push_back(id);
}

void IndirectSyncWrite::addIdList(std::vector<uint8_t> ids)
{
    ids_.insert(ids_.end(), ids.begin(), ids.end());
}

void IndirectSyncWrite::setData(uint8_t id, uint16_t address, int value)
{
    if (std::find(ids_.begin(), ids_.end(), id) == ids_.end())
    {
        throw IndirectSyncIOException("ID " + std::to_string(id) + " is not found in id list.");
    }
    data_[id][address] = value;
}

int IndirectSyncWrite::initiate()
{
    if (ids_.empty() || params_.empty())
    {
        throw IndirectSyncIOException("No ids or params added");
    }
    data_length_ = 0;
    uint16_t write_address = addr_indirect_address_;
    for (std::vector<std::pair<uint16_t, uint8_t> >::iterator it = params_.begin(); it != params_.end(); ++it)
    {
        uint16_t param_address = it->first;
        uint8_t param_length = it->second;
        for (uint8_t i = 0; i < param_length; i++)
        {
            for (std::vector<uint8_t>::iterator it_id = ids_.begin(); it_id != ids_.end(); ++it_id)
            {   
                int result = packet_handler_->write2ByteTxRx(port_handler_, *it_id, write_address, param_address + i);
                if (result != COMM_SUCCESS)
                {
                    return result;
                }
                ROS_DEBUG("IndirectSyncWrite::initiate: Wrote value: %d to address: %d", param_address + i, write_address);
            }
            write_address += 2;
        }
        data_length_ += param_length;
    }
    getSyncWriteHandler();
    return 0;
}

int IndirectSyncWrite::txPacket()
{
    preparePacket();
    int result = sync_write_handler_->txPacket();
    return result;
}

void IndirectSyncWrite::clearTxQueue()
{
    sync_write_handler_->clearParam();
}

void IndirectSyncWrite::preparePacket()
{
    if (ids_.size() != data_.size())
    {
        throw IndirectSyncIOException("Data not set for all ids");
    }
    for (std::vector<uint8_t>::iterator it_id = ids_.begin(); it_id != ids_.end(); ++it_id)
    {
        if (data_[*it_id].size() != params_.size())
        {
            // print all params
            for (std::vector<std::pair<uint16_t, uint8_t>>::iterator it = params_.begin(); it!=params_.end(); ++it)
            {
                ROS_DEBUG("IndirectSyncWrite::preparePacket: Added params -- Address: %d, Length: %d", it->first, it->second);
            }
            for (std::unordered_map<uint16_t, int>::iterator it = data_[*it_id].begin(); it != data_[*it_id].end(); ++it)
            {
                ROS_DEBUG("IndirectSyncWrite::preparePacket: Added data ---- Address: %d, Value: %d", it->first, it->second);
            }
            throw IndirectSyncIOException("Data not set for all params");
        }
        uint8_t byte_array[data_length_];
        uint8_t byte_array_index = 0;
        for (std::vector<std::pair<uint16_t, uint8_t> >::iterator it_param = params_.begin(); it_param != params_.end(); ++it_param)
        {
            int val = data_[*it_id][it_param->first];
            if (it_param->second == 1)
            {
                byte_array[byte_array_index] = val;
                byte_array_index++;
            }
            else
            {
                if (it_param->second >= 2)
                {
                    byte_array[byte_array_index] = DXL_LOBYTE(DXL_LOWORD(val));
                    byte_array_index++;
                    byte_array[byte_array_index] = DXL_HIBYTE(DXL_LOWORD(val));
                    byte_array_index++;
                }
                if (it_param->second == 4)
                {
                    byte_array[byte_array_index] = DXL_LOBYTE(DXL_HIWORD(val));
                    byte_array_index++;
                    byte_array[byte_array_index] = DXL_HIBYTE(DXL_HIWORD(val));
                    byte_array_index++;
                }
            }
            ROS_DEBUG("IndirectSyncWrite::preparePacket: Data added to byte array | ID: %d, Address: %d, Value: %d", *it_id, it_param->first, val);
        }

        sync_write_handler_->addParam(*it_id, byte_array);
    }
}

void IndirectSyncWrite::getSyncWriteHandler()
{
    sync_write_handler_ = std::make_unique<dynamixel::GroupSyncWrite>(port_handler_, packet_handler_, addr_indirect_data_, data_length_);
}