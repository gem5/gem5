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

#ifndef __CPU_BETA_CPU_CPU_POLICY_HH__
#define __CPU_BETA_CPU_CPU_POLICY_HH__

#include "cpu/beta_cpu/bpred_unit.hh"
#include "cpu/beta_cpu/free_list.hh"
#include "cpu/beta_cpu/inst_queue.hh"
#include "cpu/beta_cpu/ldstq.hh"
#include "cpu/beta_cpu/mem_dep_unit.hh"
#include "cpu/beta_cpu/regfile.hh"
#include "cpu/beta_cpu/rename_map.hh"
#include "cpu/beta_cpu/rob.hh"
#include "cpu/beta_cpu/store_set.hh"

#include "cpu/beta_cpu/commit.hh"
#include "cpu/beta_cpu/decode.hh"
#include "cpu/beta_cpu/fetch.hh"
#include "cpu/beta_cpu/iew.hh"
#include "cpu/beta_cpu/rename.hh"

#include "cpu/beta_cpu/comm.hh"

template<class Impl>
struct SimpleCPUPolicy
{
    typedef TwobitBPredUnit<Impl> BPredUnit;
    typedef PhysRegFile<Impl> RegFile;
    typedef SimpleFreeList FreeList;
    typedef SimpleRenameMap RenameMap;
    typedef ROB<Impl> ROB;
    typedef InstructionQueue<Impl> IQ;
    typedef MemDepUnit<StoreSet, Impl> MemDepUnit;
    typedef LDSTQ<Impl> LDSTQ;

    typedef SimpleFetch<Impl> Fetch;
    typedef SimpleDecode<Impl> Decode;
    typedef SimpleRename<Impl> Rename;
    typedef SimpleIEW<Impl> IEW;
    typedef SimpleCommit<Impl> Commit;

    /** The struct for communication between fetch and decode. */
    typedef SimpleFetchSimpleDecode<Impl> FetchStruct;

    /** The struct for communication between decode and rename. */
    typedef SimpleDecodeSimpleRename<Impl> DecodeStruct;

    /** The struct for communication between rename and IEW. */
    typedef SimpleRenameSimpleIEW<Impl> RenameStruct;

    /** The struct for communication between IEW and commit. */
    typedef SimpleIEWSimpleCommit<Impl> IEWStruct;

    /** The struct for communication within the IEW stage. */
    typedef IssueStruct<Impl> IssueStruct;

    /** The struct for all backwards communication. */
    typedef TimeBufStruct TimeStruct;

};

#endif //__CPU_BETA_CPU_CPU_POLICY_HH__
