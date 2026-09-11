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

#ifndef __HWMODEL_INSTRUCTION_CONFIG_HH__
#define __HWMODEL_INSTRUCTION_CONFIG_HH__

#include "params/InstConfig.hh"
#include "sim/sim_object.hh"

// AUTO-GENERATED HEADERS (See util/SALAM-docs/README_SALAM.md for details)
#include <cstdlib>
#include <iostream>
#include <vector>

#include "instructions/add.hh"
#include "instructions/addrspacecast.hh"
#include "instructions/alloca.hh"
#include "instructions/and_inst.hh"
#include "instructions/ashr.hh"
#include "instructions/base.hh"
#include "instructions/bitcast.hh"
#include "instructions/br.hh"
#include "instructions/call.hh"
#include "instructions/fadd.hh"
#include "instructions/fcmp.hh"
#include "instructions/fdiv.hh"
#include "instructions/fence.hh"
#include "instructions/fmul.hh"
#include "instructions/fmuladd.hh"
#include "instructions/fneg.hh"
#include "instructions/fpext.hh"
#include "instructions/fptosi.hh"
#include "instructions/fptoui.hh"
#include "instructions/fptrunc.hh"
#include "instructions/frem.hh"
#include "instructions/fsub.hh"
#include "instructions/gep.hh"
#include "instructions/icmp.hh"
#include "instructions/indirectbr.hh"
#include "instructions/inttoptr.hh"
#include "instructions/invoke.hh"
#include "instructions/landingpad.hh"
#include "instructions/load.hh"
#include "instructions/lshr.hh"
#include "instructions/mul.hh"
#include "instructions/or_inst.hh"
#include "instructions/phi.hh"
#include "instructions/ptrtoint.hh"
#include "instructions/resume.hh"
#include "instructions/ret.hh"
#include "instructions/sdiv.hh"
#include "instructions/select.hh"
#include "instructions/sext.hh"
#include "instructions/shl.hh"
#include "instructions/srem.hh"
#include "instructions/store.hh"
#include "instructions/sub.hh"
#include "instructions/switch_inst.hh"
#include "instructions/trunc.hh"
#include "instructions/udiv.hh"
#include "instructions/uitofp.hh"
#include "instructions/unreachable.hh"
#include "instructions/urem.hh"
#include "instructions/vaarg.hh"
#include "instructions/xor_inst.hh"
#include "instructions/zext.hh"

using namespace gem5;

class InstConfigBase;

class InstConfig : public SimObject
{
  private:
  protected:
  public:
    // AUTO-GENERATED CLASS MEMBERS (See util/SALAM-docs/README_SALAM.md for
    // details)
    Add *_add;
    Addrspacecast *_addrspacecast;
    Alloca *_alloca;
    AndInst *_and_inst;
    Ashr *_ashr;
    Bitcast *_bitcast;
    Br *_br;
    Call *_call;
    Fadd *_fadd;
    Fcmp *_fcmp;
    Fdiv *_fdiv;
    Fence *_fence;
    Fmul *_fmul;
    Fmuladd *_fmuladd;
    Fneg *_fneg;
    Fpext *_fpext;
    Fptosi *_fptosi;
    Fptoui *_fptoui;
    Fptrunc *_fptrunc;
    Frem *_frem;
    Fsub *_fsub;
    Gep *_gep;
    Icmp *_icmp;
    Indirectbr *_indirectbr;
    Inttoptr *_inttoptr;
    Invoke *_invoke;
    Landingpad *_landingpad;
    Load *_load;
    Lshr *_lshr;
    Mul *_mul;
    OrInst *_or_inst;
    Phi *_phi;
    Ptrtoint *_ptrtoint;
    Resume *_resume;
    Ret *_ret;
    Sdiv *_sdiv;
    Select *_select;
    Sext *_sext;
    Shl *_shl;
    Srem *_srem;
    Store *_store;
    Sub *_sub;
    SwitchInst *_switch_inst;
    Trunc *_trunc;
    Udiv *_udiv;
    Uitofp *_uitofp;
    Unreachable *_unreachable;
    Urem *_urem;
    Vaarg *_vaarg;
    XorInst *_xor_inst;
    Zext *_zext;
    InstConfig();
    // DEFAULT CONSTRUCTOR (See util/SALAM-docs/README_SALAM.md for details)
    InstConfig(const InstConfigParams &params);
    // END DEFAULT CONSTRUCTOR
    std::vector<InstConfigBase *> inst_list;
};
#endif //__INSTRUCTION_CONFIG_HH__
