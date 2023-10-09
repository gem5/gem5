/*
 * Copyright (c) 2013 ARM Limited
 * Copyright (c) 2014 Sven Karlsson
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
 * Copyright (c) 2017 The University of Virginia
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

#ifndef __ARCH_RISCV_PCSTATE_HH__
#define __ARCH_RISCV_PCSTATE_HH__

#include "arch/generic/pcstate.hh"
#include "arch/riscv/regs/vector.hh"
#include "enums/RiscvType.hh"

namespace gem5
{
namespace RiscvISA
{

using RiscvType = enums::RiscvType;
constexpr enums::RiscvType RV32 = enums::RV32;
constexpr enums::RiscvType RV64 = enums::RV64;

class PCState : public GenericISA::UPCState<4>
{
  protected:
    typedef GenericISA::UPCState<4> Base;

    bool _compressed = false;
    RiscvType _rvType = RV64;
    uint64_t _vlenb = VLENB;
    VTYPE _vtype = (1ULL << 63); // vtype.vill = 1 at initial;
    uint32_t _vl = 0;

  public:
    PCState(const PCState &other) : Base(other),
        _rvType(other._rvType), _vlenb(other._vlenb),
        _vtype(other._vtype), _vl(other._vl)
    {}
    PCState &operator=(const PCState &other) = default;
    PCState() = default;
    explicit PCState(Addr addr) { set(addr); }
    explicit PCState(Addr addr, RiscvType rvType, uint64_t vlenb = VLENB)
    {
        set(addr);
        _rvType = rvType;
        _vlenb = vlenb;
    }

    PCStateBase *clone() const override { return new PCState(*this); }

    void
    update(const PCStateBase &other) override
    {
        Base::update(other);
        auto &pcstate = other.as<PCState>();
        _compressed = pcstate._compressed;
        _rvType = pcstate._rvType;
        _vlenb = pcstate._vlenb;
        _vtype = pcstate._vtype;
        _vl = pcstate._vl;
    }

    void compressed(bool c) { _compressed = c; }
    bool compressed() const { return _compressed; }

    void rvType(RiscvType rvType) { _rvType = rvType; }
    RiscvType rvType() const { return _rvType; }

    void vlenb(uint64_t v) { _vlenb = v; }
    uint64_t vlenb() const { return _vlenb; }

    void vtype(VTYPE v) { _vtype = v; }
    VTYPE vtype() const { return _vtype; }

    void vl(uint32_t v) { _vl = v; }
    uint32_t vl() const { return _vl; }

    uint8_t size() const override { return _compressed ? 2 : 4; }

    bool
    branching() const override
    {
        return npc() != pc() + size() || nupc() != upc() + 1;
    }

    bool
    equals(const PCStateBase &other) const override
    {
        auto &opc = other.as<PCState>();
        return Base::equals(other) &&
            _vlenb == opc._vlenb &&
            _vtype == opc._vtype &&
            _vl == opc._vl;
    }

    void
    serialize(CheckpointOut &cp) const override
    {
        Base::serialize(cp);
        SERIALIZE_SCALAR(_rvType);
        SERIALIZE_SCALAR(_vlenb);
        SERIALIZE_SCALAR(_vtype);
        SERIALIZE_SCALAR(_vl);
        SERIALIZE_SCALAR(_compressed);
    }

    void
    unserialize(CheckpointIn &cp) override
    {
        Base::unserialize(cp);
        UNSERIALIZE_SCALAR(_rvType);
        UNSERIALIZE_SCALAR(_vlenb);
        UNSERIALIZE_SCALAR(_vtype);
        UNSERIALIZE_SCALAR(_vl);
        UNSERIALIZE_SCALAR(_compressed);
    }
};

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_PCSTATE_HH__
