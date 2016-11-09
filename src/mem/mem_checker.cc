/*
 * Copyright (c) 2014 ARM Limited
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
 *
 * Authors: Rune Holm
 *          Marco Elver
 */

#include "mem/mem_checker.hh"

#include <cassert>

void
MemChecker::WriteCluster::startWrite(MemChecker::Serial serial, Tick _start,
                                     uint8_t data)
{
    assert(!isComplete());

    if (start == TICK_FUTURE) {
        // Initialize a fresh write cluster
        start = _start;
    }
    chatty_assert(start <= _start, "WriteClusters must filled in order!");

    ++numIncomplete;

    if (complete != TICK_FUTURE) {
       // Reopen a closed write cluster
        assert(_start < complete);  // should open a new write cluster, instead;
        // also somewhat fishy wrt causality / ordering of calls vs time
        // progression TODO: Check me!
        complete = TICK_FUTURE;
    }

    // Create new transaction, and denote completion time to be in the future.
    writes.insert(std::make_pair(serial,
                  MemChecker::Transaction(serial, _start, TICK_FUTURE, data)));
}

void
MemChecker::WriteCluster::completeWrite(MemChecker::Serial serial, Tick _complete)
{
    auto it = writes.find(serial);

    if (it == writes.end()) {
        warn("Could not locate write transaction: serial = %d, complete = %d\n",
             serial, _complete);
        return;
    }

    // Record completion time of the write
    assert(it->second.complete == TICK_FUTURE);
    it->second.complete = _complete;

    // Update max completion time for the cluster
    if (completeMax < _complete) {
        completeMax = _complete;
    }

    if (--numIncomplete == 0) {
        // All writes have completed, this cluster is now complete and will be
        // assigned the max of completion tick values among all writes.
        //
        // Note that we cannot simply keep updating complete, because that would
        // count the cluster as closed already.  Instead, we keep TICK_FUTURE
        // until all writes have completed.
        complete = completeMax;
    }
}

void
MemChecker::WriteCluster::abortWrite(MemChecker::Serial serial)
{
    if (!writes.erase(serial)) {
        warn("Could not locate write transaction: serial = %d\n", serial);
        return;
    }

    if (--numIncomplete == 0 && !writes.empty()) {
        // This write cluster is now complete, and we can assign the current
        // completeMax value.
        complete = completeMax;
    }

    // Note: this WriteCluster is in pristine state if this was the only
    // write present; the cluster will get reused through
    // getIncompleteWriteCluster().
}

void
MemChecker::ByteTracker::startRead(MemChecker::Serial serial, Tick start)
{
    outstandingReads.insert(std::make_pair(serial,
            MemChecker::Transaction(serial, start, TICK_FUTURE)));
}

bool
MemChecker::ByteTracker::inExpectedData(Tick start, Tick complete, uint8_t data)
{
    _lastExpectedData.clear();

    bool wc_overlap = true;

    // Find the last value read from the location
    const Transaction& last_obs =
        *lastCompletedTransaction(&readObservations, start);
    bool last_obs_valid = (last_obs.complete != TICK_INITIAL);

    // Scan backwards through the write clusters to find the closest younger
    // preceding & overlapping writes.
    for (auto cluster = writeClusters.rbegin();
         cluster != writeClusters.rend() && wc_overlap; ++cluster) {
        for (const auto& addr_write : cluster->writes) {
            const Transaction& write = addr_write.second;

            if (write.complete < last_obs.start) {
                // If this write transaction completed before the last
                // observation, we ignore it as the last_observation has the
                // correct value
                continue;
            }

            if (write.data == data) {
                // Found a match, end search.
                return true;
            }

            // Record possible, but non-matching data for debugging
            _lastExpectedData.push_back(write.data);

            if (write.complete > start) {
                // This write overlapped with the transaction we want to check
                // -> continue checking the overlapping write cluster
                continue;
            }

            // This write cluster has writes that have completed before the
            // checked transaction. There is no need to check an earlier
            // write-cluster -> set the exit condition for the outer loop
            wc_overlap = false;

            if (last_obs.complete < write.start) {
                // We found a write which started after the last observed read,
                // therefore we can not longer consider the value seen by the
                // last observation as a valid expected value.
                //
                // Once all writes have been iterated through, we can check if
                // the last observation is still valid to compare against.
                last_obs_valid = false;
            }
        }
    }

    // We have not found any matching write, so far; check other sources of
    // confirmation
    if (last_obs_valid) {
        // The last observation is not outdated according to the writes we have
        // seen so far.
        assert(last_obs.complete <= start);
        if (last_obs.data == data) {
            // Matched data from last observation -> all good
            return true;
        }
        // Record non-matching, but possible value
        _lastExpectedData.push_back(last_obs.data);
    } else {
        // We have not seen any valid observation, and the only writes
        // observed are overlapping, so anything (in particular the
        // initialisation value) goes
        // NOTE: We can overlap with multiple write clusters, here
        if (!writeClusters.empty() && wc_overlap) {
            // ensure that all write clusters really overlap this read
            assert(writeClusters.begin()->start < complete &&
                   writeClusters.rbegin()->complete > start);
            return true;
        }
    }

    if (_lastExpectedData.empty()) {
        assert(last_obs.complete == TICK_INITIAL);
        // We have not found any possible (non-matching data). Can happen in
        // initial system state
        DPRINTF(MemChecker, "no last observation nor write! start = %d, "\
                "complete = %d, data = %#x\n", start, complete, data);
        return true;
    }
    return false;
}

bool
MemChecker::ByteTracker::completeRead(MemChecker::Serial serial,
                                      Tick complete, uint8_t data)
{
    auto it = outstandingReads.find(serial);

    if (it == outstandingReads.end()) {
        // Can happen if concurrent with reset_address_range
        warn("Could not locate read transaction: serial = %d, complete = %d\n",
             serial, complete);
        return true;
    }

    Tick start = it->second.start;
    outstandingReads.erase(it);

    // Verify data
    const bool result = inExpectedData(start, complete, data);

    readObservations.emplace_back(serial, start, complete, data);
    pruneTransactions();

    return result;
}

MemChecker::WriteCluster*
MemChecker::ByteTracker::getIncompleteWriteCluster()
{
    if (writeClusters.empty() || writeClusters.back().isComplete()) {
        writeClusters.emplace_back();
    }

    return &writeClusters.back();
}

void
MemChecker::ByteTracker::startWrite(MemChecker::Serial serial, Tick start,
                                    uint8_t data)
{
    getIncompleteWriteCluster()->startWrite(serial, start, data);
}

void
MemChecker::ByteTracker::completeWrite(MemChecker::Serial serial, Tick complete)
{
    getIncompleteWriteCluster()->completeWrite(serial, complete);
    pruneTransactions();
}

void
MemChecker::ByteTracker::abortWrite(MemChecker::Serial serial)
{
    getIncompleteWriteCluster()->abortWrite(serial);
}

void
MemChecker::ByteTracker::pruneTransactions()
{
    // Obtain tick of first outstanding read. If there are no outstanding
    // reads, we use curTick(), i.e. we will remove all readObservation except
    // the most recent one.
    const Tick before = outstandingReads.empty() ? curTick() :
                        outstandingReads.begin()->second.start;

    // Pruning of readObservations
    readObservations.erase(readObservations.begin(),
                           lastCompletedTransaction(&readObservations, before));

    // Pruning of writeClusters
    if (!writeClusters.empty()) {
        writeClusters.erase(writeClusters.begin(),
                            lastCompletedTransaction(&writeClusters, before));
    }
}

bool
MemChecker::completeRead(MemChecker::Serial serial, Tick complete,
                         Addr addr, size_t size, uint8_t *data)
{
    bool result = true;

    DPRINTF(MemChecker,
            "completing read: serial = %d, complete = %d, "
            "addr = %#llx, size = %d\n", serial, complete, addr, size);

    for (size_t i = 0; i < size; ++i) {
        ByteTracker *tracker = getByteTracker(addr + i);

        if (!tracker->completeRead(serial, complete, data[i])) {
            // Generate error message, and aggregate all failures for the bytes
            // considered in this transaction in one message.
            if (result) {
                result = false;
                errorMessage = "";
            } else {
                errorMessage += "\n";
            }

            errorMessage += csprintf("  Read transaction for address %#llx "
                                     "failed: received %#x, expected ",
                                     (unsigned long long)(addr + i), data[i]);

            for (size_t j = 0; j < tracker->lastExpectedData().size(); ++j) {
                errorMessage +=
                    csprintf("%#x%s",
                             tracker->lastExpectedData()[j],
                             (j == tracker->lastExpectedData().size() - 1)
                             ? "" : "|");
            }
        }
    }

    if (!result) {
        DPRINTF(MemChecker, "read of %#llx @ cycle %d failed:\n%s\n", addr,
                complete, errorMessage);
    }

    return result;
}

void
MemChecker::reset(Addr addr, size_t size)
{
    for (size_t i = 0; i < size; ++i) {
        byte_trackers.erase(addr + i);
    }
}

MemChecker*
MemCheckerParams::create()
{
    return new MemChecker(this);
}
