/*
 * Copyright 2019 Google, Inc.
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

#ifndef __ARCH_GENERIC_INTERRUPTS_HH__
#define __ARCH_GENERIC_INTERRUPTS_HH__

#include "base/logging.hh"
#include "params/BaseInterrupts.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class ThreadContext;
class BaseCPU;

class BaseInterrupts : public SimObject
{
  protected:
    ThreadContext *tc = nullptr;

  public:
    using Params = BaseInterruptsParams;

    BaseInterrupts(const Params &p) : SimObject(p) {}

    virtual void
    setThreadContext(ThreadContext *_tc)
    {
        tc = _tc;
    }

    /*
     * Functions for retrieving interrupts for the CPU to handle.
     */

    /*
     * Return whether there are any interrupts waiting to be recognized.
     */
    virtual bool checkInterrupts() const = 0;
    /*
     * Return an interrupt to process. This should return an interrupt exactly
     * when checkInterrupts returns true.
     */
    virtual Fault getInterrupt() = 0;
    /*
     * Update interrupt related state after an interrupt has been processed.
     */
    virtual void updateIntrInfo() = 0;

    /*
     * Old functions needed for compatability but which will be phased out
     * eventually.
     */
    virtual void
    post(int int_num, int index)
    {
        panic("Interrupts::post unimplemented!\n");
    }

    virtual void
    clear(int int_num, int index)
    {
        panic("Interrupts::clear unimplemented!\n");
    }

    virtual void
    clearAll()
    {
        panic("Interrupts::clearAll unimplemented!\n");
    }
};

} // namespace gem5

#endif // __ARCH_GENERIC_INTERRUPTS_HH__
