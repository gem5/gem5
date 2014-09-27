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
 *
 * Authors: Gabe Black
 */

#ifndef __ARCH_GENERIC_DEBUGFAULTS_HH__
#define __ARCH_GENERIC_DEBUGFAULTS_HH__

#include <string>

#include "base/misc.hh"
#include "sim/faults.hh"

namespace GenericISA
{

class M5DebugFault : public FaultBase
{
  public:
    enum DebugFunc
    {
        PanicFunc,
        FatalFunc,
        WarnFunc,
        WarnOnceFunc
    };

  protected:
    std::string message;
    DebugFunc func;

  public:
    M5DebugFault(DebugFunc _func, std::string _message) :
        message(_message), func(_func)
    {}

    FaultName
    name() const
    {
        switch (func) {
          case PanicFunc:
            return "panic fault";
          case FatalFunc:
            return "fatal fault";
          case WarnFunc:
            return "warn fault";
          case WarnOnceFunc:
            return "warn_once fault";
          default:
            panic("unrecognized debug function number\n");
        }
    }

    void
    invoke(ThreadContext *tc, const StaticInstPtr &inst =
           StaticInst::nullStaticInstPtr)
    {
        switch (func) {
          case PanicFunc:
            panic(message);
            break;
          case FatalFunc:
            fatal(message);
            break;
          case WarnFunc:
            warn(message);
            break;
          case WarnOnceFunc:
            warn_once(message);
            break;
          default:
            panic("unrecognized debug function number\n");
        }
    }
};

template <int Func>
class M5VarArgsFault : public M5DebugFault
{
  public:
    template<typename ...Args>
    M5VarArgsFault(const std::string &format, const Args &...args) :
        M5DebugFault((DebugFunc)Func, csprintf(format, args...))
    {}
};

typedef M5VarArgsFault<M5DebugFault::PanicFunc> M5PanicFault;
typedef M5VarArgsFault<M5DebugFault::FatalFunc> M5FatalFault;
typedef M5VarArgsFault<M5DebugFault::WarnFunc> M5WarnFault;
typedef M5VarArgsFault<M5DebugFault::WarnOnceFunc> M5WarnOnceFault;

} // namespace GenericISA

#endif // __ARCH_GENERIC_DEBUGFAULTS_HH__
