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

#include "cycle_counts.hh"

CycleCounts::CycleCounts(const CycleCountsParams &p)
    : SimObject(p),
      counter_inst(p.counter),
      gep_inst(p.gep),
      phi_inst(p.phi),
      select_inst(p.select),
      ret_inst(p.ret),
      br_inst(p.br),
      switch_inst(p.switch_inst),
      indirectbr_inst(p.indirectbr),
      invoke_inst(p.invoke),
      resume_inst(p.resume),
      unreachable_inst(p.unreachable),
      icmp_inst(p.icmp),
      fcmp_inst(p.fcmp),
      trunc_inst(p.trunc),
      zext_inst(p.zext),
      sext_inst(p.sext),
      fptrunc_inst(p.fptrunc),
      fpext_inst(p.fpext),
      fptoui_inst(p.fptoui),
      fptosi_inst(p.fptosi),
      uitofp_inst(p.uitofp),
      sitofp_inst(p.sitofp),
      ptrtoint_inst(p.ptrtoint),
      inttoptr_inst(p.inttoptr),
      bitcast_inst(p.bitcast),
      addrspacecast_inst(p.addrspacecast),
      call_inst(p.call),
      vaarg_inst(p.vaarg),
      landingpad_inst(p.landingpad),
      catchpad_inst(p.catchpad),
      alloca_inst(p.alloca),
      load_inst(p.load),
      store_inst(p.store),
      fence_inst(p.fence),
      cmpxchg_inst(p.cmpxchg),
      atomicrmw_inst(p.atomicrmw),
      extractvalue_inst(p.extractvalue),
      insertvalue_inst(p.insertvalue),
      extractelement_inst(p.extractelement),
      insertelement_inst(p.insertelement),
      shufflevector_inst(p.shufflevector),
      shl_inst(p.shl),
      lshr_inst(p.lshr),
      ashr_inst(p.ashr),
      and_inst(p.and_inst),
      or_inst(p.or_inst),
      xor_inst(p.xor_inst),
      add_inst(p.add),
      sub_inst(p.sub),
      mul_inst(p.mul),
      udiv_inst(p.udiv),
      sdiv_inst(p.sdiv),
      urem_inst(p.urem),
      srem_inst(p.srem),
      fadd_inst(p.fadd),
      fsub_inst(p.fsub),
      fmul_inst(p.fmul),
      fdiv_inst(p.fdiv),
      frem_inst(p.frem)
{}
