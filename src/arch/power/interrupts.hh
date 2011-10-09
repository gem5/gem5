/*
 * Copyright (c) 2011 Google
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

#ifndef __ARCH_POWER_INTERRUPT_HH__
#define __ARCH_POWER_INTERRUPT_HH__

#include "base/misc.hh"
#include "params/PowerInterrupts.hh"
#include "sim/sim_object.hh"

class ThreadContext;

namespace PowerISA {

class Interrupts : public SimObject
{
  private:
    BaseCPU * cpu;

  public:
    typedef PowerInterruptsParams Params;

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    Interrupts(Params * p) : SimObject(p), cpu(NULL)
    {}

    void
    setCPU(BaseCPU * _cpu)
    {
        cpu = _cpu;
    }

    void
    post(int int_num, int index)
    {
        panic("Interrupts::post not implemented.\n");
    }

    void
    clear(int int_num, int index)
    {
        panic("Interrupts::clear not implemented.\n");
    }

    void
    clearAll()
    {
        panic("Interrupts::clearAll not implemented.\n");
    }

    bool
    checkInterrupts(ThreadContext *tc) const
    {
        panic("Interrupts::checkInterrupts not implemented.\n");
    }

    Fault
    getInterrupt(ThreadContext *tc)
    {
        panic("Interrupts::getInterrupt not implemented.\n");
    }

    void
    updateIntrInfo(ThreadContext *tc)
    {
        panic("Interrupts::updateIntrInfo not implemented.\n");
    }
};

} // namespace PowerISA

#endif // __ARCH_POWER_INTERRUPT_HH__

