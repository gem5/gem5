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

#ifndef __CPU_O3_IMPL_HH__
#define __CPU_O3_IMPL_HH__

#include "cpu/o3/comm.hh"

/** Implementation specific struct that defines several key types to the
 *  CPU, the stages within the CPU, the time buffers, and the DynInst.
 *  The struct defines the ISA, the CPU policy, the specific DynInst, the
 *  specific O3CPU, and all of the structs from the time buffers to do
 *  communication.
 *  This is one of the key things that must be defined for each hardware
 *  specific CPU implementation.
 */
struct O3CPUImpl
{
    /** The struct for communication between fetch and decode. */
    typedef DefaultFetchDefaultDecode<O3CPUImpl> FetchStruct;

    /** The struct for communication between decode and rename. */
    typedef DefaultDecodeDefaultRename<O3CPUImpl> DecodeStruct;

    /** The struct for communication between rename and IEW. */
    typedef DefaultRenameDefaultIEW<O3CPUImpl> RenameStruct;

    /** The struct for communication between IEW and commit. */
    typedef DefaultIEWDefaultCommit<O3CPUImpl> IEWStruct;

    /** The struct for communication within the IEW stage. */
    typedef ::IssueStruct<O3CPUImpl> IssueStruct;

    /** The struct for all backwards communication. */
    typedef TimeBufStruct<O3CPUImpl> TimeStruct;
};

#endif // __CPU_O3_SPARC_IMPL_HH__
