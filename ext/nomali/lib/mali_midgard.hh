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

#ifndef _LIBNOMALIMODEL_MALI_MIDGARD_HH
#define _LIBNOMALIMODEL_MALI_MIDGARD_HH

#include "gpu.hh"

#include "gpucontrol.hh"
#include "jobcontrol.hh"
#include "mmu.hh"

namespace NoMali {

/**
 * Base class for Midgard based GPUs
 */
class MaliMidgard
    : public GPU
{
  public:
    MaliMidgard(unsigned gpuType,
                unsigned major, unsigned minor, unsigned status);
    MaliMidgard(uint32_t gpuId);
    virtual ~MaliMidgard();


  protected:
    class GPUControlSpec
        : public GPUControl
    {
      public:
        GPUControlSpec(MaliMidgard &_gpu)
            : GPUControl(_gpu), midgard(_gpu) {}

        void reset() override;

      private:
        MaliMidgard &midgard;
    };

    virtual void setupControlIdRegisters(RegVector &regs);

    GPUControlSpec gpuControl;
    JobControl jobControl;
    MMU mmu;

  private:
    const uint32_t gpuId;
};

}

#endif // _LIBNOMALIMODEL_MALI_MIDGARD_HH
