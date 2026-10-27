/*
 * Copyright (c) 2015, 2021, 2026 Arm Limited
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
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * All rights reserved.
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

// Request-port callbacks are derived from the MemTest tester. This small
// scripted tester records retries and response times without modeling a CPU.
#ifndef __SALAM_TESTING_DEVICE_TESTER_HH__
#define __SALAM_TESTING_DEVICE_TESTER_HH__

#include <memory>
#include <vector>

#include "dev/dma_device.hh"
#include "mem/port.hh"
#include "params/SALAMDeviceTester.hh"
#include "salam/comm_interface.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

namespace gem5
{

class SALAMDeviceTester : public ClockedObject
{
  private:
    class TestPort : public RequestPort
    {
      private:
        SALAMDeviceTester &tester;

      public:
        TestPort(const std::string &name, SALAMDeviceTester &owner)
            : RequestPort(name), tester(owner)
        {}

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
    } port;

    class TestReadFifo : public DmaReadFifo
    {
      public:
        TestReadFifo(DmaPort &port, size_t capacity, unsigned max_request)
            : DmaReadFifo(port, capacity, max_request, 2)
        {}

        unsigned blocksEnded = 0;
        unsigned dataNotifications = 0;

      protected:
        void
        onEndOfBlock() override
        {
            ++blocksEnded;
        }
        void
        onDataAvailable() override
        {
            ++dataNotifications;
        }
    };

    DmaPort fifoPort;
    std::unique_ptr<TestReadFifo> readFifo;
    const RequestorID requestorId;
    PacketPtr retryPkt = nullptr;
    SALAMDeviceTester *responseReader = nullptr;
    Addr responseReadAddress = 0;
    unsigned responseReadSize = 0;
    Tick rejectUntil = 0;
    bool responseBlocked = false;
    EventFunctionWrapper responseRetryEvent;
    std::vector<std::vector<uint64_t>> responses;
    std::vector<std::vector<uint64_t>> attempts;
    std::vector<Tick> requestRetries;
    std::vector<Tick> responseRejections;

    PacketPtr packet(bool read, Addr addr, unsigned size, uint64_t data);
    bool sendPacket(PacketPtr pkt);
    void retryResponse();

  public:
    PARAMS(SALAMDeviceTester);
    SALAMDeviceTester(const Params &p);
    Port &getPort(const std::string &name,
                  PortID idx = InvalidPortID) override;
    bool send(bool read, Addr addr, unsigned size, uint64_t data);
    void startReadFifo(Addr addr, unsigned size, unsigned capacity,
                       unsigned max_request);
    std::vector<uint8_t> readFifoBytes(unsigned size);
    std::vector<uint64_t> readFifoState() const;
    void readOnNextWriteResponse(SALAMDeviceTester *reader, Addr addr,
                                 unsigned size);
    uint64_t readCommArgument(CommInterface *comm, unsigned offset,
                              unsigned size);
    uint64_t functionalRead(Addr addr, unsigned size);
    void functionalWrite(Addr addr, unsigned size, uint64_t data);
    void functionalWriteMasked(Addr addr, unsigned size, uint64_t data,
                               const std::vector<bool> &byte_enable);
    void
    blockResponsesUntil(Tick tick)
    {
        rejectUntil = tick;
    }
    auto
    getResponses() const
    {
        return responses;
    }
    auto
    getAttempts() const
    {
        return attempts;
    }
    auto
    getRequestRetries() const
    {
        return requestRetries;
    }
    auto
    getResponseRejections() const
    {
        return responseRejections;
    }
};

} // namespace gem5

#endif // __SALAM_TESTING_DEVICE_TESTER_HH__
