/*
 * Copyright (c) 2016 ARM Limited
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
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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

#ifndef __KERN_LINUX_EVENTS_HH__
#define __KERN_LINUX_EVENTS_HH__

#include <functional>
#include <string>

#include "base/compiler.hh"
#include "base/trace.hh"
#include "debug/DebugPrintf.hh"
#include "kern/linux/printk.hh"
#include "kern/system_events.hh"
#include "mem/se_translating_port_proxy.hh"
#include "sim/guest_abi.hh"

namespace gem5
{

class ThreadContext;

GEM5_DEPRECATED_NAMESPACE(Linux, linux);
namespace linux
{

template <typename ABI, typename Base>
class DebugPrintk : public Base
{
  public:
    using Base::Base;
    void
    process(ThreadContext *tc) override
    {
        if (debug::DebugPrintf) {
            std::string str;
            std::function<int(ThreadContext *, Addr, PrintkVarArgs)> func =
                [&str](ThreadContext *tc, Addr format_ptr,
                    PrintkVarArgs args) -> int {
                return printk(str, tc, format_ptr, args);
            };
            invokeSimcall<ABI>(tc, func);
            DPRINTFN("%s", str);
        }
        Base::process(tc);
    }
};

/**
 * Dump the guest kernel's dmesg buffer to a file in gem5's output
 * directory and print a warning.
 *
 * @warn This event uses linux::dumpDmesg() and comes with the same
 * limitations. Most importantly, the kernel's address mappings must
 * be available to the translating proxy.
 */
class DmesgDump : public PCEvent
{
  protected:
    std::string fname;

  public:
    DmesgDump(PCEventScope *s, const std::string &desc, Addr addr,
              const std::string &_fname) :
        PCEvent(s, desc, addr), fname(_fname)
    {}
    void process(ThreadContext *tc) override;
};

/**
 * Dump the guest kernel's dmesg buffer to a file in gem5's output
 * directory and panic.
 *
 * @warn This event uses linux::dumpDmesg() and comes with the same
 * limitations. Most importantly, the kernel's address mappings must
 * be available to the translating proxy.
 */
class KernelPanic : public PCEvent
{
  protected:
    std::string fname;

  public:
    KernelPanic(PCEventScope *s, const std::string &desc, Addr addr,
                const std::string &_fname) :
        PCEvent(s, desc, addr), fname(_fname)
    {}
    void process(ThreadContext *tc) override;
};

void onUDelay(ThreadContext *tc, uint64_t div, uint64_t mul, uint64_t time);

/**
 * A class to skip udelay() and related calls in the kernel.
 * This class has two additional parameters that take the argument to udelay
 * and manipulated it to come up with ns and eventually ticks to quiesce for.
 * See descriptions of argDivToNs and argMultToNs below.
 */
template <typename ABI, typename Base>
class SkipUDelay : public Base
{
  private:
    /**
     * Value to divide arg by to create ns. This is present beacues the linux
     * kernel code sometime precomputes the first multiply that is done in
     * udelay() if the parameter is a constant. We need to undo it so here is
     * how.
     */
    uint64_t argDivToNs;

    /**
     * Value to multiple arg by to create ns. Nominally, this is 1000 to
     * convert us to ns, but since linux can do some preprocessing of constant
     * values something else might be required.
     */
    uint64_t argMultToNs;

  public:
    SkipUDelay(PCEventScope *s, const std::string &desc, Addr addr,
            uint64_t mult, uint64_t div) :
        Base(s, desc, addr), argDivToNs(div), argMultToNs(mult)
    {}

    void
    process(ThreadContext *tc) override
    {
        // Use Addr since it's handled specially and will act as a natively
        // sized data type.
        std::function<void(ThreadContext *, Addr)> call_udelay =
            [this](ThreadContext *tc, Addr time) {
            onUDelay(tc, argDivToNs, argMultToNs, time);
        };
        invokeSimcall<ABI>(tc, call_udelay);
        Base::process(tc);
    }
};

} // namespace linux
} // namespace gem5

#endif // __KERN_LINUX_EVENTS_HH__
