/*
 * Copyright 2021 Google Inc.
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

#include "sim/workload.hh"

#include "base/remote_gdb.hh"
#include "cpu/thread_context.hh"
#include "sim/debug.hh"

namespace gem5
{

void
Workload::registerThreadContext(ThreadContext *tc)
{
    std::set<ThreadContext *>::iterator it;
    bool success;
    std::tie(it, success) = threads.insert(tc);
    panic_if(!success, "Failed to add thread context %d.",
            tc->contextId());

    if (gdb)
        gdb->addThreadContext(tc);
}

void
Workload::replaceThreadContext(ThreadContext *tc)
{
    ContextID id = tc->contextId();

    for (auto *old: threads) {
        if (old->contextId() != id)
            continue;
        threads.erase(old);

        std::set<ThreadContext *>::iterator it;
        bool success;
        std::tie(it, success) = threads.insert(tc);
        panic_if(!success,
                "Failed to insert replacement thread context %d.", id);

        if (gdb)
            gdb->replaceThreadContext(tc);

        return;
    }
    panic("Replacement thread context %d doesn't match any known id.", id);
}

bool
Workload::trapToGdb(GDBSignal signal, ContextID ctx_id)
{
    if (gdb && gdb->isAttached()) {
        gdb->trap(ctx_id, signal);
        return true;
    }
    return false;
};
bool
Workload::sendToGdb(std::string msg){
     if (gdb)
        return gdb->sendMessage(msg);
    else
        return false;
 }


void
Workload::startup()
{
    SimObject::startup();

    // Now that we're about to start simulation, wait for GDB connections if
    // requested.
    if (gdb && waitForRemoteGDB) {
        inform("%s: Waiting for a remote GDB connection on %s.", name(),
                gdb->hostSocket());
        gdb->connect();
    }
}

} // namespace gem5
