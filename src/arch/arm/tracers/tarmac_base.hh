/*
 * Copyright (c) 2011,2017-2019 ARM Limited
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
 */

/**
 * @file: This file contains the data structure used to rappresent
 *        Tarmac entities/informations. These data structures will
 *        be used and extended by either the Tarmac Parser and
 *        the Tarmac Tracer.
 *        Instruction execution is matched by Records, so that for
 *        every instruction executed there is a corresponding record.
 *        A trace is made of Records (Generated or Parsed) and a record
 *        is made of Entries.
 */

#ifndef __ARCH_ARM_TRACERS_TARMAC_BASE_HH__
#define __ARCH_ARM_TRACERS_TARMAC_BASE_HH__

#include "arch/arm/types.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "cpu/static_inst.hh"
#include "sim/insttracer.hh"

namespace gem5
{

class ThreadContext;

namespace trace
{

class TarmacBaseRecord : public InstRecord
{
  public:
    /** TARMAC trace record type. */
    enum TarmacRecordType
    {
        TARMAC_INST,
        TARMAC_REG,
        TARMAC_MEM,
        TARMAC_UNSUPPORTED,
    };

    /** ARM instruction set state. */
    enum ISetState
    {
        ISET_ARM,
        ISET_THUMB,
        ISET_A64,
        ISET_UNSUPPORTED
    };

    /** ARM register type. */
    enum RegType
    {
        REG_R,
        REG_X,
        REG_S,
        REG_D,
        REG_P,
        REG_Q,
        REG_Z,
        REG_MISC
    };

    /** TARMAC instruction trace record. */
    struct InstEntry
    {
        InstEntry() = default;
        InstEntry(ThreadContext *thread, const PCStateBase &pc,
                  const StaticInstPtr staticInst, bool predicate);

        bool taken;
        Addr addr;
        ArmISA::MachInst opcode;
        ISetState isetstate;
        ArmISA::OperatingMode mode;
    };

    /** TARMAC register trace record. */
    struct RegEntry
    {
        enum RegElement
        {
            Lo = 0,
            Hi = 1,
            // Max = (max SVE vector length) 2048b / 64 = 32
            Max = 32
        };

        RegEntry() = default;
        RegEntry(const PCStateBase &pc);

        RegType type;
        RegIndex index;
        ISetState isetstate;
        std::vector<uint64_t> values;
    };

    /** TARMAC memory access trace record (stores only). */
    struct MemEntry
    {
        MemEntry() = default;
        MemEntry(uint8_t _size, Addr _addr, uint64_t _data);

        uint8_t size;
        Addr addr;
        uint64_t data;
    };

  public:
    TarmacBaseRecord(Tick _when, ThreadContext *_thread,
                     const StaticInstPtr _staticInst, const PCStateBase &_pc,
                     const StaticInstPtr _macroStaticInst = nullptr);

    virtual void dump() = 0;

    /**
     * Returns the Instruction Set State according to the current
     * PCState.
     *
     * @param pc program counter (PCState) variable
     * @return Instruction Set State for the given PCState
     */
    static ISetState pcToISetState(const PCStateBase &pc);
};

} // namespace trace
} // namespace gem5

#endif // __ARCH_ARM_TRACERS_TARMAC_BASE_HH__
