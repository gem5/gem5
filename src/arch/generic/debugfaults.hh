/*
 * Copyright (c) 2010 Advanced Micro Devices, Inc.
 * All rights reserved.
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

#ifndef __ARCH_GENERIC_DEBUGFAULTS_HH__
#define __ARCH_GENERIC_DEBUGFAULTS_HH__

#include <string>

#include "base/logging.hh"
#include "cpu/null_static_inst.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "sim/faults.hh"

namespace gem5
{

namespace GenericISA
{

class M5DebugFault : public FaultBase
{
  protected:
    std::string _message;
    virtual void debugFunc() = 0;

    void
    advancePC(ThreadContext *tc, const StaticInstPtr &inst)
    {
        if (inst) {
            std::unique_ptr<PCStateBase> pc(tc->pcState().clone());
            inst->advancePC(*pc);
            tc->pcState(*pc);
        }
    }

  public:
    M5DebugFault(std::string _m) : _message(_m) {}

    template <class... Args>
    M5DebugFault(const std::string &format, const Args &...args)
        : _message(csprintf(format, args...))
    {}

    std::string
    message()
    {
        return _message;
    }

    void
    invoke(ThreadContext *tc,
           const StaticInstPtr &inst = nullStaticInstPtr) override
    {
        debugFunc();
        advancePC(tc, inst);
    }
};

// The "Flavor" template parameter is to keep warn, hack or inform messages
// with the same token from blocking each other.
template <class Flavor>
class M5DebugOnceFault : public M5DebugFault
{
  protected:
    bool &once;

    template <class F, class OnceToken>
    static bool &
    lookUpToken(const OnceToken &token)
    {
        static std::map<OnceToken, bool> tokenMap;
        return tokenMap[token];
    }

  public:
    template <class OnceToken, class... Args>
    M5DebugOnceFault(const OnceToken &token, const std::string &format,
                     const Args &...args)
        : M5DebugFault(format, args...), once(lookUpToken<Flavor>(token))
    {}

    void
    invoke(ThreadContext *tc,
           const StaticInstPtr &inst = nullStaticInstPtr) override
    {
        if (!once) {
            once = true;
            debugFunc();
        }
        advancePC(tc, inst);
    }
};

class M5PanicFault : public M5DebugFault
{
  public:
    using M5DebugFault::M5DebugFault;

    void
    debugFunc() override
    {
        panic(message());
    }

    FaultName
    name() const override
    {
        return "panic fault";
    }
};

class M5FatalFault : public M5DebugFault
{
  public:
    using M5DebugFault::M5DebugFault;

    void
    debugFunc() override
    {
        fatal(message());
    }

    FaultName
    name() const override
    {
        return "fatal fault";
    }
};

template <class Base>
class M5WarnFaultBase : public Base
{
  public:
    using Base::Base;

    void
    debugFunc() override
    {
        warn(this->message());
    }

    FaultName
    name() const override
    {
        return "warn fault";
    }
};

using M5WarnFault = M5WarnFaultBase<M5DebugFault>;
using M5WarnOnceFault = M5WarnFaultBase<M5DebugOnceFault<M5WarnFault>>;

template <class Base>
class M5HackFaultBase : public Base
{
  public:
    using Base::Base;

    void
    debugFunc() override
    {
        hack(this->message());
    }

    FaultName
    name() const override
    {
        return "hack fault";
    }
};

using M5HackFault = M5HackFaultBase<M5DebugFault>;
using M5HackOnceFault = M5HackFaultBase<M5DebugOnceFault<M5HackFault>>;

template <class Base>
class M5InformFaultBase : public Base
{
  public:
    using Base::Base;

    void
    debugFunc() override
    {
        inform(this->message());
    }

    FaultName
    name() const override
    {
        return "inform fault";
    }
};

using M5InformFault = M5InformFaultBase<M5DebugFault>;
using M5InformOnceFault = M5InformFaultBase<M5DebugOnceFault<M5InformFault>>;

} // namespace GenericISA
} // namespace gem5

#endif // __ARCH_GENERIC_DEBUGFAULTS_HH__
