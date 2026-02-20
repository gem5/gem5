/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "instruction_config.hh"

// GENERATED CONSTRUCTOR - DO NOT MODIFY
InstConfig::InstConfig(const InstConfigParams &params)
    : SimObject(params),
      _add(params.add),
      _addrspacecast(params.addrspacecast),
      _alloca(params.alloca),
      _and_inst(params.and_inst),
      _ashr(params.ashr),
      _bitcast(params.bitcast),
      _br(params.br),
      _call(params.call),
      _fadd(params.fadd),
      _fcmp(params.fcmp),
      _fdiv(params.fdiv),
      _fence(params.fence),
      _fmul(params.fmul),
      _fmuladd(params.fmuladd),
      _fneg(params.fneg),
      _fpext(params.fpext),
      _fptosi(params.fptosi),
      _fptoui(params.fptoui),
      _fptrunc(params.fptrunc),
      _frem(params.frem),
      _fsub(params.fsub),
      _gep(params.gep),
      _icmp(params.icmp),
      _indirectbr(params.indirectbr),
      _inttoptr(params.inttoptr),
      _invoke(params.invoke),
      _landingpad(params.landingpad),
      _load(params.load),
      _lshr(params.lshr),
      _mul(params.mul),
      _or_inst(params.or_inst),
      _phi(params.phi),
      _ptrtoint(params.ptrtoint),
      _resume(params.resume),
      _ret(params.ret),
      _sdiv(params.sdiv),
      _select(params.select),
      _sext(params.sext),
      _shl(params.shl),
      _srem(params.srem),
      _store(params.store),
      _sub(params.sub),
      _switch_inst(params.switch_inst),
      _trunc(params.trunc),
      _udiv(params.udiv),
      _uitofp(params.uitofp),
      _unreachable(params.unreachable),
      _urem(params.urem),
      _vaarg(params.vaarg),
      _xor_inst(params.xor_inst),
      _zext(params.zext)
{
    inst_list.push_back(_add);
    inst_list.push_back(_addrspacecast);
    inst_list.push_back(_alloca);
    inst_list.push_back(_and_inst);
    inst_list.push_back(_ashr);
    inst_list.push_back(_bitcast);
    inst_list.push_back(_br);
    inst_list.push_back(_call);
    inst_list.push_back(_fadd);
    inst_list.push_back(_fcmp);
    inst_list.push_back(_fdiv);
    inst_list.push_back(_fence);
    inst_list.push_back(_fmul);
    inst_list.push_back(_fmuladd);
    inst_list.push_back(_fneg);
    inst_list.push_back(_fpext);
    inst_list.push_back(_fptosi);
    inst_list.push_back(_fptoui);
    inst_list.push_back(_fptrunc);
    inst_list.push_back(_frem);
    inst_list.push_back(_fsub);
    inst_list.push_back(_gep);
    inst_list.push_back(_icmp);
    inst_list.push_back(_indirectbr);
    inst_list.push_back(_inttoptr);
    inst_list.push_back(_invoke);
    inst_list.push_back(_landingpad);
    inst_list.push_back(_load);
    inst_list.push_back(_lshr);
    inst_list.push_back(_mul);
    inst_list.push_back(_or_inst);
    inst_list.push_back(_phi);
    inst_list.push_back(_ptrtoint);
    inst_list.push_back(_resume);
    inst_list.push_back(_ret);
    inst_list.push_back(_sdiv);
    inst_list.push_back(_select);
    inst_list.push_back(_sext);
    inst_list.push_back(_shl);
    inst_list.push_back(_srem);
    inst_list.push_back(_store);
    inst_list.push_back(_sub);
    inst_list.push_back(_switch_inst);
    inst_list.push_back(_trunc);
    inst_list.push_back(_udiv);
    inst_list.push_back(_uitofp);
    inst_list.push_back(_unreachable);
    inst_list.push_back(_urem);
    inst_list.push_back(_vaarg);
    inst_list.push_back(_xor_inst);
    inst_list.push_back(_zext);
}
// END OF GENERATED CONSTRUCTOR
