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

#include "cpu/testers/gpu_ruby_test/protocol_tester.hh"

#include <algorithm>
#include <ctime>
#include <fstream>

#include "base/random.hh"
#include "cpu/testers/gpu_ruby_test/cpu_thread.hh"
#include "cpu/testers/gpu_ruby_test/dma_thread.hh"
#include "cpu/testers/gpu_ruby_test/gpu_wavefront.hh"
#include "cpu/testers/gpu_ruby_test/tester_thread.hh"
#include "debug/ProtocolTest.hh"
#include "mem/request.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

namespace gem5
{

ProtocolTester::ProtocolTester(const Params &p)
      : ClockedObject(p),
        _requestorId(p.system->getRequestorId(this)),
        numCpuPorts(p.port_cpu_ports_connection_count),
        numDmaPorts(p.port_dma_ports_connection_count),
        numVectorPorts(p.port_cu_vector_ports_connection_count),
        numSqcPorts(p.port_cu_sqc_ports_connection_count),
        numScalarPorts(p.port_cu_scalar_ports_connection_count),
        numTokenPorts(p.port_cu_token_ports_connection_count),
        numCusPerSqc(p.cus_per_sqc),
        numCusPerScalar(p.cus_per_scalar),
        numWfsPerCu(p.wavefronts_per_cu),
        numWisPerWf(p.workitems_per_wavefront),
        numCuTokens(p.max_cu_tokens),
        numAtomicLocs(p.num_atomic_locations),
        numNormalLocsPerAtomic(p.num_normal_locs_per_atomic),
        episodeLength(p.episode_length),
        maxNumEpisodes(p.max_num_episodes),
        debugTester(p.debug_tester),
        cpuThreads(p.cpu_threads),
        dmaThreads(p.dma_threads),
        wfs(p.wavefronts)
{
    int idx = 0;  // global port index

    numCpus = numCpuPorts;     // 1 cpu port per CPU
    numDmas = numDmaPorts;     // 1 dma port per DMA
    numCus = numVectorPorts;   // 1 vector port per CU

    // create all physical cpu's data ports
    for (int i = 0; i < numCpuPorts; ++i) {
        DPRINTF(ProtocolTest, "Creating %s\n",
                csprintf("%s-cpuPort%d", name(), i));
        cpuPorts.push_back(new SeqPort(csprintf("%s-cpuPort%d", name(), i),
                                       this, i, idx));
        idx++;
    }

    // create all physical DMA data ports
    for (int i = 0; i < numDmaPorts; ++i) {
        DPRINTF(ProtocolTest, "Creating %s\n",
                csprintf("%s-dmaPort%d", name(), i));
        dmaPorts.push_back(new SeqPort(csprintf("%s-dmaPort%d", name(), i),
                                       this, i, idx));
        idx++;
    }

    // create all physical gpu's data ports
    for (int i = 0; i < numVectorPorts; ++i) {
        DPRINTF(ProtocolTest, "Creating %s\n",
                csprintf("%s-cuVectorPort%d", name(), i));
        cuVectorPorts.push_back(new SeqPort(csprintf("%s-cuVectorPort%d",
                                                     name(), i),
                                            this, i, idx));
        idx++;
    }

    for (int i = 0; i < numScalarPorts; ++i) {
        DPRINTF(ProtocolTest, "Creating %s\n",
                              csprintf("%s-cuScalarPort%d", name(), i));
        cuScalarPorts.push_back(new SeqPort(csprintf("%s-cuScalarPort%d",
                                                     name(), i),
                                            this, i, idx));
        idx++;
    }

    for (int i = 0; i < numSqcPorts; ++i) {
        DPRINTF(ProtocolTest, "Creating %s\n",
                              csprintf("%s-cuSqcPort%d", name(), i));
        cuSqcPorts.push_back(new SeqPort(csprintf("%s-cuSqcPort%d",
                                                  name(), i),
                                         this, i, idx));
        idx++;
    }

    for (int i = 0; i < numTokenPorts; ++i) {
        cuTokenPorts.push_back(new GMTokenPort(csprintf("%s-cuTokenPort%d",
                                                        name(), i),
                                               this, i));
        cuTokenManagers.push_back(new TokenManager(numCuTokens));
        cuTokenPorts[i]->setTokenManager(cuTokenManagers[i]);
    }

    // create an address manager
    addrManager = new AddressManager(numAtomicLocs,
                                       numNormalLocsPerAtomic);
    nextEpisodeId = 0;

    if (!debugTester)
      warn("Data race check is not enabled\n");

    sentExitSignal = false;

    // set random seed number, if specified.
    // Note: random_m5 will use a fixed key if random_seed is not set.
    // This ensures a reproducable.
    if (p.random_seed != 0) {
        random_mt.init(p.random_seed);
    } else {
        warn(
            "If `random_seed == 0` (or `random_seed` is unset) "
            "ProtocolTester does not seed the RNG. This will NOT result in "
            "the RNG generating different results each run. In this case the "
            "RNG is seeded by a default value. This differs from behavior in "
            "previous versions of gem5. Setting `random_seed` to a non-zero "
            "value is strongly recommended."
        );
    }

    actionCount = 0;

    // create a new log file
    logFile = simout.create(p.log_file);
    assert(logFile);

    // print test configs
    std::stringstream ss;
    ss << "GPU Ruby test's configurations" << std::endl
       << "\tNumber of CPUs: " << numCpus << std::endl
       << "\tNumber of DMAs: " << numDmas << std::endl
       << "\tNumber of CUs: " << numCus << std::endl
       << "\tNumber of wavefronts per CU: " << numWfsPerCu << std::endl
       << "\tWavefront size: " << numWisPerWf << std::endl
       << "\tNumber of atomic locations: " << numAtomicLocs << std::endl
       << "\tNumber of non-atomic locations: "
       << numNormalLocsPerAtomic * numAtomicLocs << std::endl
       << "\tEpisode length: " << episodeLength << std::endl
       << "\tTest length (max number of episodes): " << maxNumEpisodes
       << std::endl
       << "\tRandom seed: " << p.random_seed
       << std::endl;

    ccprintf(*(logFile->stream()), "%s", ss.str());
    logFile->stream()->flush();
}

ProtocolTester::~ProtocolTester()
{
    for (int i = 0; i < cpuPorts.size(); ++i)
        delete cpuPorts[i];
    for (int i = 0; i < dmaPorts.size(); ++i)
        delete dmaPorts[i];
    for (int i = 0; i < cuVectorPorts.size(); ++i)
        delete cuVectorPorts[i];
    for (int i = 0; i < cuScalarPorts.size(); ++i)
        delete cuScalarPorts[i];
    for (int i = 0; i < cuSqcPorts.size(); ++i)
        delete cuSqcPorts[i];
    delete addrManager;

    // close the log file
    simout.close(logFile);
}

void
ProtocolTester::init()
{
    DPRINTF(ProtocolTest, "Attach threads to ports\n");

    // connect cpu threads to cpu's ports
    for (int cpu_id = 0; cpu_id < numCpus; ++cpu_id) {
        cpuThreads[cpu_id]->attachTesterThreadToPorts(this,
                                      static_cast<SeqPort*>(cpuPorts[cpu_id]));
        cpuThreads[cpu_id]->scheduleWakeup();
        cpuThreads[cpu_id]->scheduleDeadlockCheckEvent();
    }

    // connect dma threads to dma's ports
    for (int dma_id = 0; dma_id < numDmas; ++dma_id) {
        dmaThreads[dma_id]->attachTesterThreadToPorts(this,
                                      static_cast<SeqPort*>(dmaPorts[dma_id]));
        dmaThreads[dma_id]->scheduleWakeup();
        dmaThreads[dma_id]->scheduleDeadlockCheckEvent();
    }

    // connect gpu wavefronts to gpu's ports
    int wfId = 0;
    int vectorPortId = 0;
    int sqcPortId = 0;
    int scalarPortId = 0;

    for (int cu_id = 0; cu_id < numCus; ++cu_id) {
        vectorPortId = cu_id;
        sqcPortId = cu_id/numCusPerSqc;
        scalarPortId = cu_id/numCusPerScalar;

        for (int i = 0; i < numWfsPerCu; ++i) {
            wfId = cu_id * numWfsPerCu + i;
            wfs[wfId]->attachTesterThreadToPorts(this,
                           static_cast<SeqPort*>(cuVectorPorts[vectorPortId]),
                           cuTokenPorts[vectorPortId],
                           static_cast<SeqPort*>(cuSqcPorts[sqcPortId]),
                           static_cast<SeqPort*>(cuScalarPorts[scalarPortId]));
            wfs[wfId]->scheduleWakeup();
            wfs[wfId]->scheduleDeadlockCheckEvent();
        }
    }
}

Port&
ProtocolTester::getPort(const std::string &if_name, PortID idx)
{
    if (if_name != "cpu_ports" && if_name != "dma_ports" &&
        if_name != "cu_vector_ports" && if_name != "cu_sqc_ports" &&
        if_name != "cu_scalar_ports" && if_name != "cu_token_ports") {
        // pass along to super class
        return ClockedObject::getPort(if_name, idx);
    } else {
        if (if_name == "cpu_ports") {
            if (idx > numCpuPorts)
                panic("ProtocolTester: unknown cpu port %d\n", idx);
            return *cpuPorts[idx];
        } else if (if_name == "dma_ports") {
            if (idx > numDmaPorts)
                panic("ProtocolTester: unknown dma port %d\n", idx);
            return *dmaPorts[idx];
        } else if (if_name == "cu_vector_ports") {
            if (idx > numVectorPorts)
                panic("ProtocolTester: unknown cu vect port %d\n", idx);
            return *cuVectorPorts[idx];
        } else if (if_name == "cu_sqc_ports") {
            if (idx > numSqcPorts)
                panic("ProtocolTester: unknown cu sqc port %d\n", idx);
            return *cuSqcPorts[idx];
        } else if (if_name == "cu_token_ports") {
            if (idx > numTokenPorts)
                panic("ProtocolTester: unknown cu token port %d\n", idx);
            return *cuTokenPorts[idx];
        } else {
            assert(if_name == "cu_scalar_ports");
            if (idx > numScalarPorts)
                panic("ProtocolTester: unknown cu scal port %d\n", idx);
            return *cuScalarPorts[idx];
        }
    }

    assert(false);
}

bool
ProtocolTester::checkExit()
{
    if (nextEpisodeId > maxNumEpisodes) {
        if (!sentExitSignal) {
            // all done
            inform("Total completed episodes: %d\n", nextEpisodeId - 1);
            exitSimLoop("GPU Ruby Tester: Passed!");
            sentExitSignal = true;
        }
        return true;
    }
    return false;
}

bool
ProtocolTester::checkDRF(Location atomic_loc,
                         Location loc, bool isStore) const
{
    if (debugTester) {
        // go through all active episodes in all threads
        for (const TesterThread* th : wfs) {
            if (!th->checkDRF(atomic_loc, loc, isStore))
                return false;
        }

        for (const TesterThread* th : cpuThreads) {
            if (!th->checkDRF(atomic_loc, loc, isStore))
                return false;
        }

        for (const TesterThread* th : dmaThreads) {
            if (!th->checkDRF(atomic_loc, loc, isStore))
                return false;
        }
    }

    return true;
}

void
ProtocolTester::dumpErrorLog(std::stringstream& ss)
{
    if (!sentExitSignal) {
        // go through all threads and dump their outstanding requests
        for (auto t : cpuThreads) {
            t->printAllOutstandingReqs(ss);
        }

        for (auto t : dmaThreads) {
            t->printAllOutstandingReqs(ss);
        }

        for (auto t : wfs) {
            t->printAllOutstandingReqs(ss);
        }

        // dump error log into a file
        assert(logFile);
        ccprintf(*(logFile->stream()), "%s", ss.str());
        logFile->stream()->flush();

        sentExitSignal = true;
        // terminate the simulation
        panic("GPU Ruby Tester: Failed!\n");
    }
}

bool
ProtocolTester::SeqPort::recvTimingResp(PacketPtr pkt)
{
    // get the requesting thread from the original sender state
    ProtocolTester::SenderState* senderState =
                    safe_cast<ProtocolTester::SenderState*>(pkt->senderState);
    TesterThread *th = senderState->th;

    th->hitCallback(pkt);

    return true;
}

} // namespace gem5
