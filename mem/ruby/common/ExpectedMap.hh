/*
 * Copyright (c) 2021 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __MEM_RUBY_COMMON_EXPECTEDMAP_HH__
#define __MEM_RUBY_COMMON_EXPECTEDMAP_HH__

#include <cassert>
#include <iostream>
#include <unordered_map>

namespace gem5
{

namespace ruby
{

// ExpectedMap helper class is used to facilitate tracking of pending
// response and data messages in the CHI protocol. It offers additional
// functionality when compared to plain counters:
//  - tracks the expected type for received messages
//  - tracks segmented data messages (i.e. when a line transfer is split in
//    multiple messages)

template<typename RespType, typename DataType>
class ExpectedMap
{
  private:

    template<typename Type>
    struct ExpectedState
    {
        struct EnumClassHash
        {
            std::size_t operator()(Type t) const
            {
                return static_cast<std::size_t>(t);
            }
        };

      private:
        // chunks is the number segmented messages we expect to receive
        // before incrementing numReceived. This is tipically always 1 for all
        // non-data messages
        int chunks;
        int currChunk;
        int numReceived;
        std::unordered_map<Type, bool, EnumClassHash> expectedTypes;

      public:
        ExpectedState()
            :chunks(1), currChunk(0), numReceived(0)
        {}

        void
        clear(int msg_chunks)
        {
            chunks = msg_chunks;
            currChunk = 0;
            numReceived = 0;
            expectedTypes.clear();
        }

        void
        addExpectedType(const Type &val)
        {
            expectedTypes[val] = false;
        }

        int received() const { return numReceived; }

        bool
        increaseReceived(const Type &val)
        {
            if (expectedTypes.find(val) == expectedTypes.end())
                return false;

            expectedTypes[val] = true;
            ++currChunk;
            if (currChunk == chunks) {
                ++numReceived;
                currChunk = 0;
            }

            return true;
        }

        bool
        receivedType(const Type &val) const
        {
            auto i = expectedTypes.find(val);
            if (i != expectedTypes.end())
                return i->second;
            else
                return false;
        }
    };

    ExpectedState<DataType> expectedData;
    ExpectedState<RespType> expectedResp;
    int totalExpected;

  public:
    ExpectedMap()
        :expectedData(), expectedResp(), totalExpected(0)
    {}

    // Clear the tracking state and specified the number of chunks are required
    // to receive a complete data message
    void
    clear(int dataChunks)
    {
        expectedData.clear(dataChunks);
        expectedResp.clear(1);
        totalExpected = 0;
    }

    // Register an expected response message type
    void
    addExpectedRespType(const RespType &val)
    {
        expectedResp.addExpectedType(val);
    }

    // Register an expected data message type
    void
    addExpectedDataType(const DataType &val)
    {
        expectedData.addExpectedType(val);
    }

    // Set the number of expected messages
    void setExpectedCount(int val) { totalExpected = val; }

    void addExpectedCount(int val) { totalExpected += val; }

    // Returns the number of messages received.
    // Notice that a data message counts as received only after all of
    // its chunks are received.
    int
    received() const
    {
        return expectedData.received() + expectedResp.received();
    }

    // Returns the remaining number of expected messages
    int expected() const { return totalExpected - received(); }

    // Has any expected message ?
    bool hasExpected() const { return expected() != 0; }

    // Has received any data ?
    bool hasReceivedData() const { return expectedData.received() != 0; }

    // Has received any response ?
    bool hasReceivedResp() const { return expectedResp.received() != 0; }


    // Notifies that a response message was received
    bool
    receiveResp(const RespType &val)
    {
        assert(received() < totalExpected);
        return expectedResp.increaseReceived(val);
    }

    // Notifies that a data message chunk was received
    bool
    receiveData(const DataType &val)
    {
        assert(received() <= totalExpected);
        return expectedData.increaseReceived(val);
    }

    // Has received any data of the given type ?
    bool
    receivedDataType(const DataType &val) const
    {
        return expectedData.receivedType(val);
    }

    // Has received any response of the given type ?
    bool
    receivedRespType(const RespType &val) const
    {
        return expectedResp.receivedType(val);
    }

    void
    print(std::ostream& out) const
    {
        out << expected();
    }
};

template<typename RespType, typename DataType>
inline std::ostream&
operator<<(std::ostream& out, const ExpectedMap<RespType,DataType>& obj)
{
    obj.print(out);
    return out;
}

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_COMMON_EXPECTEDMAP_HH__
