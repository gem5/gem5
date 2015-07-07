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

#include "mali_t7xx.hh"

#include "regutils.hh"

namespace NoMali {

MaliT7xxBase::MaliT7xxBase(unsigned gpuType,
                           unsigned major, unsigned minor, unsigned status)
    : MaliMidgard(gpuType, major, minor, status)
{
}

void
MaliT7xxBase::setupControlIdRegisters(RegVector &regs)
{
    MaliMidgard::setupControlIdRegisters(regs);

    regs[RegAddr(L2_FEATURES)] =
        (0x07 << 24) |  // lg2 ext bus width
        (0x13 << 16) |  // lg2 cache size
        (0x02 << 8) |   // lg2 associativity
        (0x06);         // lg2 line size
}


MaliT76x::MaliT76x(unsigned major, unsigned minor, unsigned status)
    : MaliT7xxBase(GPU_ID_PI_T76X, major, minor, status)
{
}

};
