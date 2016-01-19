/*
 * Copyright (c) 2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
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
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
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
 *
 * Author: Anthony Gutierrez
 */

#ifndef __HSA_CODE_HH__
#define __HSA_CODE_HH__

#include <string>
#include <vector>

#include "arch/gpu_types.hh"
#include "config/the_gpu_isa.hh"

class HsaKernelInfo;

/* @class HsaCode
 * base code object for the set of HSA kernels associated
 * with a single application. this class provides the common
 * methods for creating, accessing, and storing information
 * about kernel and variable symbols, symbol name, memory
 * segment sizes, and instruction count, etc.
 */

class HsaCode
{
  public:
    HsaCode(const std::string &name) : readonly_data(nullptr), funcarg_size(0),
                                       _name(name)
    {
    }

    enum class MemorySegment {
        NONE,
        FLAT,
        GLOBAL,
        READONLY,
        KERNARG,
        GROUP,
        PRIVATE,
        SPILL,
        ARG,
        EXTSPACE0
    };

    const std::string& name() const { return _name; }
    int numInsts() const { return _insts.size(); }
    std::vector<TheGpuISA::RawMachInst>* insts() { return &_insts; }

    void
    setReadonlyData(uint8_t *_readonly_data)
    {
        readonly_data = _readonly_data;
    }

    virtual int getSize(MemorySegment segment) const = 0;
    virtual void generateHsaKernelInfo(HsaKernelInfo *hsaKernelInfo) const = 0;

    uint8_t *readonly_data;
    int funcarg_size;

  protected:
    // An array that stores instruction indices (0 through kernel size)
    // for a kernel passed to code object constructor as an argument.
    std::vector<TheGpuISA::RawMachInst> _insts;

  private:
    const std::string _name;
};

#endif // __HSA_CODE_HH__
