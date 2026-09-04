/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * (University of Wisconsin-Madison)
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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

#ifndef __HWMODEL_INSTOPCODES_HH__
#define __HWMODEL_INSTOPCODES_HH__

#include <cstdint>
#include <map>

#include "params/InstOpCodes.hh"
#include "sim/sim_object.hh"

using namespace gem5;

class InstOpCodes : public SimObject
{
  public:
    uint32_t counter_inst;
    uint32_t gep_inst;
    uint32_t phi_inst;
    uint32_t select_inst;
    uint32_t ret_inst;
    uint32_t br_inst;
    uint32_t switch_inst;
    uint32_t indirectbr_inst;
    uint32_t invoke_inst;
    uint32_t resume_inst;
    uint32_t unreachable_inst;
    uint32_t icmp_inst;
    uint32_t fcmp_inst;
    uint32_t trunc_inst;
    uint32_t zext_inst;
    uint32_t sext_inst;
    uint32_t fptrunc_inst;
    uint32_t fpext_inst;
    uint32_t fptoui_inst;
    uint32_t fptosi_inst;
    uint32_t uitofp_inst;
    uint32_t sitofp_inst;
    uint32_t ptrtoint_inst;
    uint32_t inttoptr_inst;
    uint32_t bitcast_inst;
    uint32_t addrspacecast_inst;
    uint32_t call_inst;
    uint32_t vaarg_inst;
    uint32_t landingpad_inst;
    uint32_t catchpad_inst;
    uint32_t alloca_inst;
    uint32_t load_inst;
    uint32_t store_inst;
    uint32_t fence_inst;
    uint32_t cmpxchg_inst;
    uint32_t atomicrmw_inst;
    uint32_t extractvalue_inst;
    uint32_t insertvalue_inst;
    uint32_t extractelement_inst;
    uint32_t insertelement_inst;
    uint32_t shufflevector_inst;
    uint32_t shl_inst;
    uint32_t lshr_inst;
    uint32_t ashr_inst;
    uint32_t and_inst;
    uint32_t or_inst;
    uint32_t xor_inst;
    uint32_t add_inst;
    uint32_t sub_inst;
    uint32_t mul_inst;
    uint32_t udiv_inst;
    uint32_t sdiv_inst;
    uint32_t urem_inst;
    uint32_t srem_inst;
    uint32_t fadd_inst;
    uint32_t fsub_inst;
    uint32_t fmul_inst;
    uint32_t fdiv_inst;
    uint32_t frem_inst;

    std::map<int, int> usage;

    InstOpCodes();
    InstOpCodes(const InstOpCodesParams &p);
    void
    update_usage(uint64_t OpCode)
    {
        usage[OpCode]++;
    };
    uint64_t
    get_usage(uint64_t OpCode)
    {
        return usage[OpCode];
    }
};

#endif //__HWMODEL_INSTOPCODES_HH__
