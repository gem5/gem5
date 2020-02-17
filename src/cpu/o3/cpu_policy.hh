/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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

#include "cpu/o3/comm.hh"
#include "cpu/o3/commit.hh"
#include "cpu/o3/decode.hh"
#include "cpu/o3/fetch.hh"
#include "cpu/o3/free_list.hh"
#include "cpu/o3/iew.hh"
#include "cpu/o3/inst_queue.hh"
#include "cpu/o3/lsq.hh"
#include "cpu/o3/lsq_unit.hh"
#include "cpu/o3/mem_dep_unit.hh"
#include "cpu/o3/regfile.hh"
#include "cpu/o3/rename.hh"
#include "cpu/o3/rename_map.hh"
#include "cpu/o3/rob.hh"
#include "cpu/o3/store_set.hh"

/**
 * Struct that defines the key classes to be used by the CPU.  All
 * classes use the typedefs defined here to determine what are the
 * classes of the other stages and communication buffers.  In order to
 * change a structure such as the IQ, simply change the typedef here
 * to use the desired class instead, and recompile.  In order to
 * create a different CPU to be used simultaneously with this one, see
 * the alpha_impl.hh file for instructions.
 */
template<class Impl>
struct SimpleCPUPolicy
{
    /** Typedef for the freelist of registers. */
    typedef UnifiedFreeList FreeList;
    /** Typedef for the rename map. */
    typedef UnifiedRenameMap RenameMap;
    /** Typedef for the ROB. */
    typedef ::ROB<Impl> ROB;
    /** Typedef for the instruction queue/scheduler. */
    typedef InstructionQueue<Impl> IQ;
    /** Typedef for the memory dependence unit. */
    typedef ::MemDepUnit<StoreSet, Impl> MemDepUnit;
    /** Typedef for the LSQ. */
    typedef ::LSQ<Impl> LSQ;
    /** Typedef for the thread-specific LSQ units. */
    typedef ::LSQUnit<Impl> LSQUnit;

    /** Typedef for fetch. */
    typedef DefaultFetch<Impl> Fetch;
    /** Typedef for decode. */
    typedef DefaultDecode<Impl> Decode;
    /** Typedef for rename. */
    typedef DefaultRename<Impl> Rename;
    /** Typedef for Issue/Execute/Writeback. */
    typedef DefaultIEW<Impl> IEW;
    /** Typedef for commit. */
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
    typedef ::IssueStruct<Impl> IssueStruct;

    /** The struct for all backwards communication. */
    typedef TimeBufStruct<Impl> TimeStruct;

};

#endif //__CPU_O3_CPU_POLICY_HH__
