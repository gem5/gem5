/*
 * Copyright (c) 2022 Fraunhofer IESE
 * All rights reserved
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

#ifndef __MEM_DRAMSYS_H__
#define __MEM_DRAMSYS_H__

#include "DRAMSysConfiguration.h"
#include "mem/abstract_mem.hh"
#include "mem/dramsys_wrapper.hh"
#include "params/DRAMSys.hh"

namespace gem5
{

namespace memory
{

class DRAMSys : public AbstractMemory
{
    PARAMS(DRAMSys);
    sc_gem5::TlmTargetWrapper<32> tlmWrapper;

  public:
    DRAMSys(Params const &params)
        : AbstractMemory(params),
          tlmWrapper(dramSysWrapper.tSocket,
              params.name + ".tlm",
              InvalidPortID),
          config(DRAMSysConfiguration::from_path(
              params.configuration,
              params.resource_directory)),
          dramSysWrapper(params.name.c_str(),
            config,
            params.recordable,
            params.range)
    {
    }

    gem5::Port &getPort(const std::string &if_name, PortID idx) override
    {
        if (if_name != "tlm")
        {
            return AbstractMemory::getPort(if_name, idx);
        }

        return tlmWrapper;
    }

  private:
    DRAMSysConfiguration::Configuration config;
    DRAMSysWrapper dramSysWrapper;
};

} // namespace memory
} // namespace gem5

#endif // __MEM_DRAMSYS_HH__
