/*
 * Copyright (c) 2014-2016 ARM Limited
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

#ifndef _LIBNOMALIMODEL_MMU_HH
#define _LIBNOMALIMODEL_MMU_HH

#include <vector>

#include "gpublock.hh"
#include "addrspace.hh"
#include "types.hh"

namespace NoMali {

class GPU;

/**
 * MMU dummy implementation.
 *
 * This is a dummy implementation of a Midgard GPU MMU block. The only
 * features supported by the block is interrupt delivery and registers
 * related to interrupt delivery. Writes to unimplemented registers
 * (most registers) are discarded and their values are read as zero.
 */
class MMU
    : public GPUBlockInt
{
  public:
    MMU(GPU &_gpu);
    virtual ~MMU();

    void reset() override;

    uint32_t readReg(RegAddr idx)  override;
    void writeReg(RegAddr idx, uint32_t value) override;

    uint32_t readRegRaw(RegAddr idx)  override;
    void writeRegRaw(RegAddr idx, uint32_t value) override;

  protected:
    void onInterrupt(int set) override;

    /** Address spaces belonging to this MMU block */
    std::vector<AddrSpace> spaces;
};

}

#endif // _LIBNOMALIMODEL_MMU_HH
