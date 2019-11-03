/*
 * Copyright (c) 2017 Advanced Micro Devices, Inc.
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
 * Author: Brandon Potter
 */

#ifndef SRC_SIM_MEM_STATE_HH
#define SRC_SIM_MEM_STATE_HH

#include "sim/serialize.hh"

/**
 * This class holds the memory state for the Process class and all of its
 * derived, architecture-specific children.
 *
 * The fields held in this class dynamically change as the process object
 * is run in the simulator. They are updated by system calls and faults;
 * each change represents a modification to the process address space.
 * The stack, heap, and mmap boundaries are held with this class along with
 * the base of the next thread stack.
 *
 * The class is meant to be allocated dynamically and shared through a
 * pointer interface because two process can potentially share their virtual
 * address space if certain options are passed into the clone(2).
 */
class MemState : public Serializable
{
  public:
    MemState(Addr brk_point, Addr stack_base, Addr max_stack_size,
             Addr next_thread_stack_base, Addr mmap_end)
        : _brkPoint(brk_point), _stackBase(stack_base), _stackSize(0),
          _maxStackSize(max_stack_size), _stackMin(0),
          _nextThreadStackBase(next_thread_stack_base), _mmapEnd(mmap_end)
    { }

    MemState&
    operator=(const MemState &in)
    {
        if (this == &in)
            return *this;

        _brkPoint = in._brkPoint;
        _stackBase = in._stackBase;
        _stackSize = in._stackSize;
        _maxStackSize = in._maxStackSize;
        _stackMin = in._stackMin;
        _nextThreadStackBase = in._nextThreadStackBase;
        _mmapEnd = in._mmapEnd;
        return *this;
    }

    Addr getBrkPoint() const { return _brkPoint; }
    Addr getStackBase() const { return _stackBase; }
    Addr getStackSize() const { return _stackSize; }
    Addr getMaxStackSize() const { return _maxStackSize; }
    Addr getStackMin() const { return _stackMin; }
    Addr getNextThreadStackBase() const { return _nextThreadStackBase; }
    Addr getMmapEnd() const { return _mmapEnd; }

    void setBrkPoint(Addr brk_point) { _brkPoint = brk_point; }
    void setStackBase(Addr stack_base) { _stackBase = stack_base; }
    void setStackSize(Addr stack_size) { _stackSize = stack_size; }
    void setMaxStackSize(Addr max_stack) { _maxStackSize = max_stack; }
    void setStackMin(Addr stack_min) { _stackMin = stack_min; }
    void setNextThreadStackBase(Addr ntsb) { _nextThreadStackBase = ntsb; }
    void setMmapEnd(Addr mmap_end) { _mmapEnd = mmap_end; }

    void
    serialize(CheckpointOut &cp) const override
    {
        paramOut(cp, "brkPoint", _brkPoint);
        paramOut(cp, "stackBase", _stackBase);
        paramOut(cp, "stackSize", _stackSize);
        paramOut(cp, "maxStackSize", _maxStackSize);
        paramOut(cp, "stackMin", _stackMin);
        paramOut(cp, "nextThreadStackBase", _nextThreadStackBase);
        paramOut(cp, "mmapEnd", _mmapEnd);
    }
    void
    unserialize(CheckpointIn &cp) override
    {
        paramIn(cp, "brkPoint", _brkPoint);
        paramIn(cp, "stackBase", _stackBase);
        paramIn(cp, "stackSize", _stackSize);
        paramIn(cp, "maxStackSize", _maxStackSize);
        paramIn(cp, "stackMin", _stackMin);
        paramIn(cp, "nextThreadStackBase", _nextThreadStackBase);
        paramIn(cp, "mmapEnd", _mmapEnd);
    }

  private:
    Addr _brkPoint;
    Addr _stackBase;
    Addr _stackSize;
    Addr _maxStackSize;
    Addr _stackMin;
    Addr _nextThreadStackBase;
    Addr _mmapEnd;
};

#endif
