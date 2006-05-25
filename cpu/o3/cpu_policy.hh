/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#ifndef __CPU_O3_CPU_POLICY_HH__
#define __CPU_O3_CPU_POLICY_HH__

#include "cpu/o3/bpred_unit.hh"
#include "cpu/o3/free_list.hh"
#include "cpu/o3/inst_queue.hh"
#include "cpu/o3/lsq.hh"
#include "cpu/o3/lsq_unit.hh"
#include "cpu/o3/mem_dep_unit.hh"
#include "cpu/o3/regfile.hh"
#include "cpu/o3/rename_map.hh"
#include "cpu/o3/rob.hh"
#include "cpu/o3/store_set.hh"

#include "cpu/o3/commit.hh"
#include "cpu/o3/decode.hh"
#include "cpu/o3/fetch.hh"
#include "cpu/o3/iew.hh"
#include "cpu/o3/rename.hh"

#include "cpu/o3/comm.hh"

template<class Impl>
struct SimpleCPUPolicy
{
    typedef BPredUnit<Impl> BPredUnit;
    typedef PhysRegFile<Impl> RegFile;
    typedef SimpleFreeList FreeList;
    typedef SimpleRenameMap RenameMap;
    typedef ROB<Impl> ROB;
    typedef InstructionQueue<Impl> IQ;
    typedef MemDepUnit<StoreSet, Impl> MemDepUnit;
    typedef LSQ<Impl> LSQ;
    typedef LSQUnit<Impl> LSQUnit;


    typedef DefaultFetch<Impl> Fetch;
    typedef DefaultDecode<Impl> Decode;
    typedef DefaultRename<Impl> Rename;
    typedef DefaultIEW<Impl> IEW;
    typedef DefaultCommit<Impl> Commit;

    /** The struct for communication between fetch and decode. */
    typedef DefaultFetchDefaultDecode<Impl> FetchStruct;

    /** The struct for communication between decode and rename. */
    typedef DefaultDecodeDefaultRename<Impl> DecodeStruct;

    /** The struct for communication between rename and IEW. */
    typedef DefaultRenameDefaultIEW<Impl> RenameStruct;

    /** The struct for communication between IEW and commit. */
    typedef DefaultIEWDefaultCommit<Impl> IEWStruct;

    /** The struct for communication within the IEW stage. */
    typedef IssueStruct<Impl> IssueStruct;

    /** The struct for all backwards communication. */
    typedef TimeBufStruct<Impl> TimeStruct;

};

#endif //__CPU_O3_CPU_POLICY_HH__
