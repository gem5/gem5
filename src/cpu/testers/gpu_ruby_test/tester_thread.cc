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

#include "cpu/testers/gpu_ruby_test/tester_thread.hh"

#include <fstream>

#include "base/random.hh"
#include "debug/ProtocolTest.hh"

namespace gem5
{

TesterThread::TesterThread(const Params &p)
      : ClockedObject(p),
        threadEvent(this, "TesterThread tick"),
        deadlockCheckEvent(this),
        threadId(p.thread_id),
        numLanes(p.num_lanes),
        tester(nullptr), addrManager(nullptr), port(nullptr),
        scalarPort(nullptr), sqcPort(nullptr), curEpisode(nullptr),
        curAction(nullptr), pendingLdStCount(0), pendingFenceCount(0),
        pendingAtomicCount(0), lastActiveCycle(Cycles(0)),
        deadlockThreshold(p.deadlock_threshold)
{
}

TesterThread::~TesterThread()
{
    for (auto ep : episodeHistory) {
        assert(ep != nullptr);
        delete ep;
    }
}

void
TesterThread::wakeup()
{
    // this thread is waken up by one of the following events
    //      - hitCallback is called
    //      - a new episode is created

    // check if this is the first episode in this thread
    if (curEpisode == nullptr) {
        issueNewEpisode();
        assert(curEpisode);
    }

    if (isNextActionReady()) {
        // isNextActionReady should check if the action list is empty
        assert(curAction != nullptr);

        // issue the next action
        issueNextAction();
    } else {
        // check for completion of the current episode
        // completion = no outstanding requests + not having more actions
        if (!curEpisode->hasMoreActions() &&
            pendingLdStCount == 0 &&
            pendingFenceCount == 0 &&
            pendingAtomicCount == 0) {

            curEpisode->completeEpisode();

            // check if it's time to stop the tester
            if (tester->checkExit()) {
                // no more event is scheduled for this thread
                return;
            }

            // issue the next episode
            issueNewEpisode();
            assert(curEpisode);

            // now we get a new episode
            // let's wake up the thread in the next cycle
            if (!threadEvent.scheduled()) {
                scheduleWakeup();
            }
        }
    }
}

void
TesterThread::scheduleWakeup()
{
    assert(!threadEvent.scheduled());
    schedule(threadEvent, nextCycle());
}

void
TesterThread::scheduleDeadlockCheckEvent()
{
    // after this first schedule, the deadlock event is scheduled by itself
    assert(!deadlockCheckEvent.scheduled());
    schedule(deadlockCheckEvent, nextCycle());
}

void
TesterThread::attachTesterThreadToPorts(ProtocolTester *_tester,
                            ProtocolTester::SeqPort *_port,
                            ProtocolTester::GMTokenPort *_tokenPort,
                            ProtocolTester::SeqPort *_scalarPort,
                            ProtocolTester::SeqPort *_sqcPort)
{
    tester = _tester;
    port = _port;
    tokenPort = _tokenPort;
    scalarPort = _scalarPort;
    sqcPort = _sqcPort;

    assert(tester && port);
    addrManager = tester->getAddressManager();
    assert(addrManager);
}

void
TesterThread::issueNewEpisode()
{
    int num_reg_loads = \
        random_mt.random<unsigned int>() % tester->getEpisodeLength();
    int num_reg_stores = tester->getEpisodeLength() - num_reg_loads;

    // create a new episode
    curEpisode = new Episode(tester, this, num_reg_loads, num_reg_stores);
    episodeHistory.push_back(curEpisode);
}

int
TesterThread::getTokensNeeded()
{
    if (!tokenPort) {
        return 0;
    }

    int tokens_needed = 0;
    curAction = curEpisode->peekCurAction();

    switch(curAction->getType()) {
        case Episode::Action::Type::ATOMIC:
            tokens_needed = numLanes;
            break;
        case Episode::Action::Type::LOAD:
        case Episode::Action::Type::STORE:
            for (int lane = 0; lane < numLanes; ++lane) {
                Location loc = curAction->getLocation(lane);

                if (loc != AddressManager::INVALID_LOCATION && loc >= 0) {
                    tokens_needed++;
                }
            }
            break;
        default:
            tokens_needed = 0;
    }

    return tokens_needed;
}

bool
TesterThread::isNextActionReady()
{
    if (!curEpisode->hasMoreActions()) {
        return false;
    } else {
        curAction = curEpisode->peekCurAction();

        // Only GPU wavefront threads have a token port. For all other types
        // of threads evaluate to true.
        bool haveTokens = true;

        switch(curAction->getType()) {
            case Episode::Action::Type::ATOMIC:
                haveTokens = tokenPort ?
                    tokenPort->haveTokens(getTokensNeeded()) : true;

                // an atomic action must wait for all previous requests
                // to complete
                if (pendingLdStCount == 0 &&
                    pendingFenceCount == 0 &&
                    pendingAtomicCount == 0 &&
                    haveTokens) {
                    return true;
                }

                return false;
            case Episode::Action::Type::ACQUIRE:
                // we should not see any outstanding ld_st or fence here
                assert(pendingLdStCount == 0 &&
                       pendingFenceCount == 0);

                // an acquire action must wait for all previous atomic
                // requests to complete
                if (pendingAtomicCount == 0) {
                    return true;
                }

                return false;
            case Episode::Action::Type::RELEASE:
                // we should not see any outstanding atomic or fence here
                assert(pendingAtomicCount == 0 &&
                       pendingFenceCount == 0);

                // a release action must wait for all previous ld/st
                // requests to complete
                if (pendingLdStCount == 0) {
                    return true;
                }

                return false;
            case Episode::Action::Type::LOAD:
            case Episode::Action::Type::STORE:
                // we should not see any outstanding atomic here
                assert(pendingAtomicCount == 0);

                // can't issue if there is a pending fence
                if (pendingFenceCount > 0) {
                    return false;
                }

                // a Load or Store is ready if it doesn't overlap
                // with any outstanding request
                for (int lane = 0; lane < numLanes; ++lane) {
                    Location loc = curAction->getLocation(lane);

                    if (loc != AddressManager::INVALID_LOCATION && loc >= 0) {
                        Addr addr = addrManager->getAddress(loc);

                        if (outstandingLoads.find(addr) !=
                            outstandingLoads.end()) {
                            return false;
                        }

                        if (outstandingStores.find(addr) !=
                            outstandingStores.end()) {
                            return false;
                        }

                        if (outstandingAtomics.find(addr) !=
                            outstandingAtomics.end()) {
                            // this is not an atomic action, so the address
                            // should not be in outstandingAtomics list
                            assert(false);
                        }
                    }
                }

                haveTokens = tokenPort ?
                    tokenPort->haveTokens(getTokensNeeded()) : true;
                if (!haveTokens) {
                    return false;
                }

                return true;
            default:
                panic("The tester got an invalid action\n");
        }
    }
}

void
TesterThread::issueNextAction()
{
    switch(curAction->getType()) {
        case Episode::Action::Type::ATOMIC:
            if (tokenPort) {
                tokenPort->acquireTokens(getTokensNeeded());
            }
            issueAtomicOps();
            break;
        case Episode::Action::Type::ACQUIRE:
            issueAcquireOp();
            break;
        case Episode::Action::Type::RELEASE:
            issueReleaseOp();
            break;
        case Episode::Action::Type::LOAD:
            if (tokenPort) {
                tokenPort->acquireTokens(getTokensNeeded());
            }
            issueLoadOps();
            break;
        case Episode::Action::Type::STORE:
            if (tokenPort) {
                tokenPort->acquireTokens(getTokensNeeded());
            }
            issueStoreOps();
            break;
        default:
            panic("The tester got an invalid action\n");
    }

    // the current action has been issued, pop it from the action list
    curEpisode->popAction();
    lastActiveCycle = curCycle();

    // we may be able to schedule the next action
    // just wake up this thread in the next cycle
    if (!threadEvent.scheduled()) {
        scheduleWakeup();
    }
}

void
TesterThread::addOutstandingReqs(OutstandingReqTable& req_table, Addr address,
                           int lane, Location loc, Value stored_val)
{
    OutstandingReqTable::iterator it = req_table.find(address);
    OutstandingReq req(lane, loc, stored_val, curCycle());

    if (it == req_table.end()) {
        // insert a new list of requests for this address
        req_table.insert(std::pair<Addr, OutstandingReqList>(address,
                                                OutstandingReqList(1, req)));
    } else {
        // add a new request
        (it->second).push_back(req);
    }
}

TesterThread::OutstandingReq
TesterThread::popOutstandingReq(OutstandingReqTable& req_table, Addr addr)
{
    OutstandingReqTable::iterator it = req_table.find(addr);

    // there must be exactly one list of requests for this address in the table
    assert(it != req_table.end());

    // get the request list
    OutstandingReqList& req_list = it->second;
    assert(!req_list.empty());

    // save a request
    OutstandingReq ret_req = req_list.back();

    // remove the request from the list
    req_list.pop_back();

    // if the list is now empty, remove it from req_table
    if (req_list.empty()) {
        req_table.erase(it);
    }

    return ret_req;
}

void
TesterThread::validateAtomicResp(Location loc, int lane, Value ret_val)
{
    if (!addrManager->validateAtomicResp(loc, ret_val)) {
        std::stringstream ss;
        Addr addr = addrManager->getAddress(loc);

        // basic info
        ss << threadName << ": Atomic Op returned unexpected value\n"
           << "\tEpisode " << curEpisode->getEpisodeId() << "\n"
           << "\tLane ID " << lane << "\n"
           << "\tAddress " << ruby::printAddress(addr) << "\n"
           << "\tAtomic Op's return value " << ret_val << "\n";

        // print out basic info
        warn("%s\n", ss.str());

        // TODO add more detailed info

        // dump all error info and exit the simulation
        tester->dumpErrorLog(ss);
    }
}

void
TesterThread::validateLoadResp(Location loc, int lane, Value ret_val)
{
    if (ret_val != addrManager->getLoggedValue(loc)) {
        std::stringstream ss;
        Addr addr = addrManager->getAddress(loc);

        // basic info
        ss << threadName << ": Loaded value is not consistent with "
           << "the last stored value\n"
           << "\tTesterThread " << threadId << "\n"
           << "\tEpisode " << curEpisode->getEpisodeId() << "\n"
           << "\tLane ID " << lane << "\n"
           << "\tAddress " << ruby::printAddress(addr) << "\n"
           << "\tLoaded value " << ret_val << "\n"
           << "\tLast writer " << addrManager->printLastWriter(loc) << "\n";

        // print out basic info
        warn("%s\n", ss.str());

        // TODO add more detailed info

        // dump all error info and exit the simulation
        tester->dumpErrorLog(ss);
    }
}

bool
TesterThread::checkDRF(Location atomic_loc, Location loc, bool isStore) const
{
    if (curEpisode && curEpisode->isEpsActive()) {
        // check against the current episode this thread is executing
        return curEpisode->checkDRF(atomic_loc, loc, isStore, numLanes);
    }

    return true;
}

void
TesterThread::checkDeadlock()
{
    if ((curCycle() - lastActiveCycle) > deadlockThreshold) {
        // deadlock detected
        std::stringstream ss;

        ss << threadName << ": Deadlock detected\n"
           << "\tLast active cycle: " <<  lastActiveCycle << "\n"
           << "\tCurrent cycle: " << curCycle() << "\n"
           << "\tDeadlock threshold: " << deadlockThreshold << "\n";

        // print out basic info
        warn("%s\n", ss.str());

        // dump all error info and exit the simulation
        tester->dumpErrorLog(ss);
    } else if (!tester->checkExit()) {
        // schedule a future deadlock check event
        assert(!deadlockCheckEvent.scheduled());
        schedule(deadlockCheckEvent,
                 deadlockThreshold * clockPeriod() + curTick());
    }
}

void
TesterThread::printOutstandingReqs(const OutstandingReqTable& table,
                             std::stringstream& ss) const
{
    Cycles cur_cycle = curCycle();

    for (const auto& m : table) {
        for (const auto& req : m.second) {
            ss << "\t\t\tAddr " << ruby::printAddress(m.first)
               << ": delta (curCycle - issueCycle) = "
               << (cur_cycle - req.issueCycle) << std::endl;
        }
    }
}

void
TesterThread::printAllOutstandingReqs(std::stringstream& ss) const
{
    // dump all outstanding requests of this thread
    ss << "\t\tOutstanding Loads:\n";
    printOutstandingReqs(outstandingLoads, ss);
    ss << "\t\tOutstanding Stores:\n";
    printOutstandingReqs(outstandingStores, ss);
    ss << "\t\tOutstanding Atomics:\n";
    printOutstandingReqs(outstandingAtomics, ss);
    ss << "\t\tNumber of outstanding acquires & releases: "
       << pendingFenceCount << std::endl;
}

} // namespace gem5
