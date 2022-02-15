/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MNRO 2017
 *
 *  ==================================================================================================
 */

#pragma once

#include <cstring>
#include <memory>
#include <string>

namespace crf {
namespace communication {
namespace datapackets {

class PacketHeader {
 public:
    PacketHeader();
    ~PacketHeader() = default;

    /*
     * @brief
     * @return
     */
    std::string serialize() const;
    /*
     * @brief
     * @param
     * @return
     */
    bool deserialize(const std::string& buffer);
    /*
     * @brief
     * @return
     */
    std::string toJSONString() const;
    /*
     * @brief
     * @return
     */
    unsigned int size() const;
    /*
     * @brief
     * @return
     */
    uint16_t type() const;
    void type(uint16_t type);
    /*
     * @brief
     * @return
     */
    uint32_t length() const;
    void length(uint32_t length);
    /*
     * @brief
     * @return
     */
    int64_t timeStamp() const;
    void timeStamp(int64_t timeStamp);
    /*
     * @brief
     * @return
     */
    uint8_t writerID() const;
    void writerID(uint8_t writerID);

 private:
    static const unsigned int size_ = 16;
    uint16_t type_;
    uint32_t length_;
    int64_t timeStamp_;
    uint8_t writerID_;

    /*
     * @brief
     * @return
     */
    uint8_t calculateCrc() const;
};

}  // namespace datapackets
}  // namespace communication
}  // namespace crf
