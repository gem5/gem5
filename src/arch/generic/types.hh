/*
 * Copyright (c) 2010 Gabe Black
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
 *
 * Authors: Gabe Black
 */

#ifndef __ARCH_GENERIC_TYPES_HH__
#define __ARCH_GENERIC_TYPES_HH__

#include <iostream>

#include "base/trace.hh"
#include "base/types.hh"
#include "sim/serialize.hh"

namespace GenericISA
{

// The guaranteed interface.
class PCStateBase : public Serializable
{
  protected:
    Addr _pc;
    Addr _npc;

    PCStateBase() : _pc(0), _npc(0) {}
    PCStateBase(Addr val) : _pc(0), _npc(0) { set(val); }

  public:
    /**
     * Returns the memory address the bytes of this instruction came from.
     *
     * @return Memory address of the current instruction's encoding.
     */
    Addr
    instAddr() const
    {
        return _pc;
    }

    /**
     * Returns the memory address the bytes of the next instruction came from.
     *
     * @return Memory address of the next instruction's encoding.
     */
    Addr
    nextInstAddr() const
    {
        return _npc;
    }

    /**
     * Returns the current micropc.
     *
     * @return The current micropc.
     */
    MicroPC
    microPC() const
    {
        return 0;
    }

    /**
     * Force this PC to reflect a particular value, resetting all its other
     * fields around it. This is useful for in place (re)initialization.
     *
     * @param val The value to set the PC to.
     */
    void set(Addr val);

    bool
    operator == (const PCStateBase &opc) const
    {
        return _pc == opc._pc && _npc == opc._npc;
    }

    bool
    operator != (const PCStateBase &opc) const
    {
        return !(*this == opc);
    }

    void
    serialize(CheckpointOut &cp) const override
    {
        SERIALIZE_SCALAR(_pc);
        SERIALIZE_SCALAR(_npc);
    }

    void
    unserialize(CheckpointIn &cp) override
    {
        UNSERIALIZE_SCALAR(_pc);
        UNSERIALIZE_SCALAR(_npc);
    }
};


/*
 * Different flavors of PC state. Only ISA specific code should rely on
 * any particular type of PC state being available. All other code should
 * use the interface above.
 */

// The most basic type of PC.
template <class MachInst>
class SimplePCState : public PCStateBase
{
  protected:
    typedef PCStateBase Base;

  public:

    Addr pc() const { return _pc; }
    void pc(Addr val) { _pc = val; }

    Addr npc() const { return _npc; }
    void npc(Addr val) { _npc = val; }

    void
    set(Addr val)
    {
        pc(val);
        npc(val + sizeof(MachInst));
    };

    SimplePCState() {}
    SimplePCState(Addr val) { set(val); }

    bool
    branching() const
    {
        return this->npc() != this->pc() + sizeof(MachInst);
    }

    // Advance the PC.
    void
    advance()
    {
        _pc = _npc;
        _npc += sizeof(MachInst);
    }
};

template <class MachInst>
std::ostream &
operator<<(std::ostream & os, const SimplePCState<MachInst> &pc)
{
    ccprintf(os, "(%#x=>%#x)", pc.pc(), pc.npc());
    return os;
}

// A PC and microcode PC.
template <class MachInst>
class UPCState : public SimplePCState<MachInst>
{
  protected:
    typedef SimplePCState<MachInst> Base;

    MicroPC _upc;
    MicroPC _nupc;

  public:

    MicroPC upc() const { return _upc; }
    void upc(MicroPC val) { _upc = val; }

    MicroPC nupc() const { return _nupc; }
    void nupc(MicroPC val) { _nupc = val; }

    MicroPC
    microPC() const
    {
        return _upc;
    }

    void
    set(Addr val)
    {
        Base::set(val);
        upc(0);
        nupc(1);
    }

    UPCState() : _upc(0), _nupc(0) {}
    UPCState(Addr val) : _upc(0), _nupc(0) { set(val); }

    bool
    branching() const
    {
        return this->npc() != this->pc() + sizeof(MachInst) ||
               this->nupc() != this->upc() + 1;
    }

    // Advance the upc within the instruction.
    void
    uAdvance()
    {
        _upc = _nupc;
        _nupc++;
    }

    // End the macroop by resetting the upc and advancing the regular pc.
    void
    uEnd()
    {
        this->advance();
        _upc = 0;
        _nupc = 1;
    }

    bool
    operator == (const UPCState<MachInst> &opc) const
    {
        return Base::_pc == opc._pc &&
               Base::_npc == opc._npc &&
               _upc == opc._upc && _nupc == opc._nupc;
    }

    bool
    operator != (const UPCState<MachInst> &opc) const
    {
        return !(*this == opc);
    }

    void
    serialize(CheckpointOut &cp) const override
    {
        Base::serialize(cp);
        SERIALIZE_SCALAR(_upc);
        SERIALIZE_SCALAR(_nupc);
    }

    void
    unserialize(CheckpointIn &cp) override
    {
        Base::unserialize(cp);
        UNSERIALIZE_SCALAR(_upc);
        UNSERIALIZE_SCALAR(_nupc);
    }
};

template <class MachInst>
std::ostream &
operator<<(std::ostream & os, const UPCState<MachInst> &pc)
{
    ccprintf(os, "(%#x=>%#x).(%d=>%d)",
            pc.pc(), pc.npc(), pc.upc(), pc.nupc());
    return os;
}

// A PC with a delay slot.
template <class MachInst>
class DelaySlotPCState : public SimplePCState<MachInst>
{
  protected:
    typedef SimplePCState<MachInst> Base;

    Addr _nnpc;

  public:

    Addr nnpc() const { return _nnpc; }
    void nnpc(Addr val) { _nnpc = val; }

    void
    set(Addr val)
    {
        Base::set(val);
        nnpc(val + 2 * sizeof(MachInst));
    }

    DelaySlotPCState() {}
    DelaySlotPCState(Addr val) { set(val); }

    bool
    branching() const
    {
        return !(this->nnpc() == this->npc() + sizeof(MachInst) &&
                 (this->npc() == this->pc() + sizeof(MachInst) ||
                  this->npc() == this->pc() + 2 * sizeof(MachInst)));
    }

    // Advance the PC.
    void
    advance()
    {
        Base::_pc = Base::_npc;
        Base::_npc = _nnpc;
        _nnpc += sizeof(MachInst);
    }

    bool
    operator == (const DelaySlotPCState<MachInst> &opc) const
    {
        return Base::_pc == opc._pc &&
               Base::_npc == opc._npc &&
               _nnpc == opc._nnpc;
    }

    bool
    operator != (const DelaySlotPCState<MachInst> &opc) const
    {
        return !(*this == opc);
    }

    void
    serialize(CheckpointOut &cp) const override
    {
        Base::serialize(cp);
        SERIALIZE_SCALAR(_nnpc);
    }

    void
    unserialize(CheckpointIn &cp) override
    {
        Base::unserialize(cp);
        UNSERIALIZE_SCALAR(_nnpc);
    }
};

template <class MachInst>
std::ostream &
operator<<(std::ostream & os, const DelaySlotPCState<MachInst> &pc)
{
    ccprintf(os, "(%#x=>%#x=>%#x)",
            pc.pc(), pc.npc(), pc.nnpc());
    return os;
}

// A PC with a delay slot and a microcode PC.
template <class MachInst>
class DelaySlotUPCState : public DelaySlotPCState<MachInst>
{
  protected:
    typedef DelaySlotPCState<MachInst> Base;

    MicroPC _upc;
    MicroPC _nupc;

  public:

    MicroPC upc() const { return _upc; }
    void upc(MicroPC val) { _upc = val; }

    MicroPC nupc() const { return _nupc; }
    void nupc(MicroPC val) { _nupc = val; }

    MicroPC
    microPC() const
    {
        return _upc;
    }

    void
    set(Addr val)
    {
        Base::set(val);
        upc(0);
        nupc(1);
    }

    DelaySlotUPCState() {}
    DelaySlotUPCState(Addr val) { set(val); }

    bool
    branching() const
    {
        return Base::branching() || this->nupc() != this->upc() + 1;
    }

    // Advance the upc within the instruction.
    void
    uAdvance()
    {
        _upc = _nupc;
        _nupc++;
    }

    // End the macroop by resetting the upc and advancing the regular pc.
    void
    uEnd()
    {
        this->advance();
        _upc = 0;
        _nupc = 1;
    }

    bool
    operator == (const DelaySlotUPCState<MachInst> &opc) const
    {
        return Base::_pc == opc._pc &&
               Base::_npc == opc._npc &&
               Base::_nnpc == opc._nnpc &&
               _upc == opc._upc && _nupc == opc._nupc;
    }

    bool
    operator != (const DelaySlotUPCState<MachInst> &opc) const
    {
        return !(*this == opc);
    }

    void
    serialize(CheckpointOut &cp) const override
    {
        Base::serialize(cp);
        SERIALIZE_SCALAR(_upc);
        SERIALIZE_SCALAR(_nupc);
    }

    void
    unserialize(CheckpointIn &cp) override
    {
        Base::unserialize(cp);
        UNSERIALIZE_SCALAR(_upc);
        UNSERIALIZE_SCALAR(_nupc);
    }
};

template <class MachInst>
std::ostream &
operator<<(std::ostream & os, const DelaySlotUPCState<MachInst> &pc)
{
    ccprintf(os, "(%#x=>%#x=>%#x).(%d=>%d)",
            pc.pc(), pc.npc(), pc.nnpc(), pc.upc(), pc.nupc());
    return os;
}

}

#endif
