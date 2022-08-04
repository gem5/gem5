/*
 * Copyright (c) 2021 Advanced Micro Devices, Inc.
 * All rights reserved.
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

#ifndef __ARCH_AMDGPU_VEGA_FAULTS_HH__
#define __ARCH_AMDGPU_VEGA_FAULTS_HH__

#include <string>

#include "arch/generic/mmu.hh"
#include "sim/faults.hh"

namespace gem5
{
namespace VegaISA
{

enum ExceptionCode : uint64_t
{
    INST_PAGE = 0,
    LOAD_PAGE = 1,
    STORE_PAGE = 2
};

class VegaFault : public FaultBase
{
  protected:
    const FaultName _name;
    const bool _interrupt;
    ExceptionCode _code;

    VegaFault(FaultName n, bool i, ExceptionCode c)
        : _name(n), _interrupt(i), _code(c)
    {}

    FaultName name() const override { return _name; }
    bool isInterrupt() const { return _interrupt; }
    ExceptionCode exception() const { return _code; }
    virtual RegVal trap_value() const { return 0; }

    void invoke(ThreadContext *tc, const StaticInstPtr &inst) override;
};

class PageFault : public VegaFault
{
  protected:
    Addr addr;

  public:
    PageFault(Addr _addr, ExceptionCode code, bool present,
              BaseMMU::Mode mode, bool user)
        : VegaFault("PageFault", false, code), addr(_addr)
    {
    }

    RegVal trap_value() const override { return addr; }
};

} // namespace VegaISA
} // namespace gem5

#endif // __ARCH_VEGA_FAULTS_HH__
