/*
 * Copyright (c) 2024 The Board of Trustees of the Leland Stanford
 * Junior University
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

#include "cpu/o3/comm.hh"

#include <algorithm>

#include "cpu/o3/dyn_inst.hh"

namespace gem5
{

namespace o3
{

/**
 * Remove instructions belonging to given thread from plain
 * instruction array. Automatically updates the array size.
 */
static void
removeThreadInsts(ThreadID tid, DynInstPtr *insts, int &size)
{
    auto has_tid = [tid] (const DynInstPtr &inst) -> bool {
        return inst->threadNumber == tid;
    };
    DynInstPtr *last = std::remove_if(insts, insts + size, has_tid);
    std::fill(last, insts + size, nullptr);
    size = last - insts;
}

void
FetchStruct::clearStates(ThreadID tid)
{
    removeThreadInsts(tid, insts, size);
}

void
DecodeStruct::clearStates(ThreadID tid)
{
    removeThreadInsts(tid, insts, size);
}

void
RenameStruct::clearStates(ThreadID tid)
{
    removeThreadInsts(tid, insts, size);
}

void
IEWStruct::clearStates(ThreadID tid)
{
    removeThreadInsts(tid, insts, size);
    mispredictInst[tid] = nullptr;
    pc[tid] = nullptr;
    squash[tid] = false;
    branchMispredict[tid] = false;
}

void
TimeStruct::clearStates(ThreadID tid)
{
    // Reset the thread's decode, rename, IEW, and commit structs to
    // their default values.
    decodeInfo[tid] = DecodeComm();
    renameInfo[tid] = RenameComm();
    iewInfo[tid] = IewComm();
    commitInfo[tid] = CommitComm();

    decodeBlock[tid] = false;
    decodeUnblock[tid] = false;
    renameBlock[tid] = false;
    renameUnblock[tid] = false;
    iewBlock[tid] = false;
    iewUnblock[tid] = false;
}

}
}
