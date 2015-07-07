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

#ifndef _LIBNOMALIMODEL_MALIT_T7XX_HH
#define _LIBNOMALIMODEL_MALIT_T7XX_HH

#include "mali_midgard.hh"

namespace NoMali {

class MaliT7xxBase
    : public MaliMidgard
{
  public:
    MaliT7xxBase(unsigned gpuType,
                 unsigned major, unsigned minor, unsigned status);

  protected:
    void setupControlIdRegisters(RegVector &regs) override;
};

/**
 * Simple NoMali implementation of the Mali T76x
 */
class MaliT76x
    : public MaliT7xxBase
{
  public:
    MaliT76x(unsigned major, unsigned minor, unsigned status);
};

}

#endif // _LIBNOMALIMODEL_MALI_T7XX_HH
