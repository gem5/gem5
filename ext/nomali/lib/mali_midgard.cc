/*
 * Copyright (c) 2014-2015 ARM Limited
 * All rights reserved
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Andreas Sandberg
 */

#include "mali_midgard.hh"

#include "regutils.hh"

namespace NoMali {

MaliMidgard::MaliMidgard(unsigned gpuType,
                         unsigned major, unsigned minor, unsigned status)
    : MaliMidgard(GPU_ID_MAKE(gpuType, major, minor, status))
{
}

MaliMidgard::MaliMidgard(uint32_t _gpuId)
    : GPU(gpuControl, jobControl, mmu),
      gpuControl(*this),
      jobControl(*this),
      mmu(*this),
      gpuId(_gpuId)
{
}

MaliMidgard::~MaliMidgard()
{
}

void
MaliMidgard::setupControlIdRegisters(RegVector &regs)
{
    regs[RegAddr(L2_FEATURES)] =
        (0x07 << 24) |  // lg2 ext bus width
        (0x13 << 16) |  // lg2 cache size
        (0x02 << 8) |   // lg2 associativity
        (0x06);         // lg2 line size

    regs[RegAddr(TILER_FEATURES)] =
        (0x8 << 8) |    // Maximum no active hierarchy levels
        0x09;           // lg2 bin size

    /* Coherent core group, but incoherent supergroup. 1 L2 slice. */
    regs[RegAddr(MEM_FEATURES)] = 0x1;

    regs[RegAddr(MMU_FEATURES)] = 0x2830;
    regs[RegAddr(AS_PRESENT)] = 0xff;
    regs[RegAddr(JS_PRESENT)] = 0x7;
    regs[RegAddr(JS0_FEATURES)] = 0x20e;
    regs[RegAddr(JS1_FEATURES)] = 0x1fe;
    regs[RegAddr(JS2_FEATURES)] = 0x7e;

    regs[RegAddr(TEXTURE_FEATURES_0)] = 0x00fe001e;
    regs[RegAddr(TEXTURE_FEATURES_1)] = 0xffff;
    regs[RegAddr(TEXTURE_FEATURES_2)] = 0x9f81ffff;

    regs[RegAddr(THREAD_MAX_THREADS)] = 0x100;
    regs[RegAddr(THREAD_MAX_WORKGROUP_SIZE)] = 0x100;
    regs[RegAddr(THREAD_MAX_BARRIER_SIZE)] = 0x100;
    regs[RegAddr(THREAD_FEATURES)] = 0x0a040400;

    regs.set64(RegAddr(SHADER_PRESENT_LO), 0xf);
    regs.set64(RegAddr(TILER_PRESENT_LO), 0x1);
    regs.set64(RegAddr(L2_PRESENT_LO), 0x1);
}

void
MaliMidgard::GPUControlSpec::reset()
{
    GPUControl::reset();

    regs[RegAddr(GPU_ID)] = midgard.gpuId;

    midgard.setupControlIdRegisters(regs);
}

};
