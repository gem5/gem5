/*
 * Copyright (c) 2017-2021 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CPU_TESTERS_PROTOCOL_TESTER_PROTOCOL_TESTER_HH_
#define CPU_TESTERS_PROTOCOL_TESTER_PROTOCOL_TESTER_HH_

/*
 * The tester includes the main ProtocolTester that manages all ports to the
 * memory system.
 * TesterThreads are mapped to certain data port(s)
 *
 * TesterThreads inject memory requests through their data ports.
 * The tester receives and validates responses from the memory.
 *
 * Main components
 *    - AddressManager: generate DRF request streams &
 *                      validate data response against an internal log_table
 *    - Episode: a sequence of requests
 *    - Thread: either GPU wavefront or CPU thread
 */

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "base/types.hh"
#include "cpu/testers/gpu_ruby_test/address_manager.hh"
#include "mem/packet.hh"
#include "mem/ruby/system/RubyPort.hh"
#include "mem/token_port.hh"
#include "params/ProtocolTester.hh"

namespace gem5
{

class TesterThread;
class CpuThread;
class GpuWavefront;

class ProtocolTester : public ClockedObject
{
  public:
    class SeqPort : public RequestPort
    {
      public:
        SeqPort(const std::string &_name, ProtocolTester *_tester, PortID _id,
                PortID _index)
            : RequestPort(_name, _id)
        {}

      protected:
        virtual bool recvTimingResp(PacketPtr pkt);

        virtual void
        recvReqRetry()
        {
            panic("%s does not expect a retry\n", name());
        }
    };

    class GMTokenPort : public TokenRequestPort
    {
      public:
        GMTokenPort(const std::string &name, ProtocolTester *_tester,
                    PortID id = InvalidPortID)
            : TokenRequestPort(name, _tester, id)
        {}

        ~GMTokenPort() {}

      protected:
        bool
        recvTimingResp(PacketPtr)
        {
            return false;
        }

        void
        recvReqRetry()
        {}
    };

    struct SenderState : public Packet::SenderState
    {
        TesterThread *th;

        SenderState(TesterThread *_th)
        {
            assert(_th);
            th = _th;
        }

        ~SenderState() {}
    };

  public:
    typedef ProtocolTesterParams Params;
    ProtocolTester(const Params &p);
    ~ProtocolTester();

    typedef AddressManager::Location Location;
    typedef AddressManager::Value Value;

    void init() override;

    RequestorID
    requestorId()
    {
        return _requestorId;
    };

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    int
    getEpisodeLength() const
    {
        return episodeLength;
    }

    // return pointer to the address manager
    AddressManager *
    getAddressManager() const
    {
        return addrManager;
    }

    // return true if the tester should stop issuing new episodes
    bool checkExit();
    // verify if a location to be picked for LD/ST will satisfy
    // data race free requirement
    bool checkDRF(Location atomic_loc, Location loc, bool isStore) const;

    // return the next episode id and increment it
    int
    getNextEpisodeID()
    {
        return nextEpisodeId++;
    }

    // get action sequence number
    int
    getActionSeqNum()
    {
        return actionCount++;
    }

    // dump error log into a file and exit the simulation
    void dumpErrorLog(std::stringstream &ss);

  private:
    RequestorID _requestorId;

    // list of parameters taken from python scripts
    int numCpuPorts;
    int numDmaPorts;
    int numVectorPorts;
    int numSqcPorts;
    int numScalarPorts;
    int numTokenPorts;
    int numCusPerSqc;
    int numCusPerScalar;
    int numWfsPerCu;
    int numWisPerWf;
    int numCuTokens;
    // parameters controlling the address range that the tester can access
    int numAtomicLocs;
    int numNormalLocsPerAtomic;
    // the number of actions in an episode (episodeLength +- random number)
    int episodeLength;
    // the maximum number of episodes to be completed by this tester
    int maxNumEpisodes;
    // are we debuggin the tester
    bool debugTester;

    // all available requestor ports connected to Ruby
    std::vector<RequestPort *> cpuPorts;      // cpu data ports
    std::vector<RequestPort *> dmaPorts;      // DMA data ports
    std::vector<RequestPort *> cuVectorPorts; // ports to GPU vector cache
    std::vector<RequestPort *> cuSqcPorts;    // ports to GPU inst cache
    std::vector<RequestPort *> cuScalarPorts; // ports to GPU scalar cache
    std::vector<TokenManager *> cuTokenManagers;
    std::vector<GMTokenPort *> cuTokenPorts;
    // all CPU, DMA, and GPU threads
    std::vector<CpuThread *> cpuThreads;
    std::vector<DmaThread *> dmaThreads;
    std::vector<GpuWavefront *> wfs;

    // address manager that (1) generates DRF sequences of requests,
    //                      (2) manages an internal log table and
    //                      (3) validate response data
    AddressManager *addrManager;

    // number of CPUs and CUs
    int numCpus;
    int numDmas;
    int numCus;
    // unique id of the next episode
    int nextEpisodeId;

    // global action count. Overflow is fine. It's used to uniquely identify
    // per-wave & per-instruction memory requests in the coalescer
    int actionCount;

    // if an exit signal was already sent
    bool sentExitSignal;

    OutputStream *logFile;
};

} // namespace gem5

#endif /* CPU_TESTERS_PROTOCOL_TESTER_PROTOCOL_TESTER_HH_ */
