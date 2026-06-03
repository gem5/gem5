/*
 * Copyright (c) 2023 Fraunhofer IESE
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

#include "dramsys.hh"

namespace gem5
{

namespace memory
{

DRAMSys::DRAMSys(Params const &params)
    : AbstractMemory(params),
      tlmWrapper(dramSysWrapper.tSocket, params.name + ".tlm", InvalidPortID),
      config(::DRAMSys::Config::from_path(params.configuration)),
      dramSysWrapper(params.name.c_str(), config, params.range)
{
    dramSysWrapper.dramsys->registerIdleCallback([this] {
        if (dramSysWrapper.dramsys->idle()) {
            signalDrainDone();
        }
    });
}

void
DRAMSys::init()
{
    dramSysWrapper.dramsys->setBackingStore(pmemAddr);
    AbstractMemory::init();
}

gem5::Port &
DRAMSys::getPort(const std::string &if_name, PortID idx)
{
    if (if_name != "tlm") {
        return AbstractMemory::getPort(if_name, idx);
    }

    return tlmWrapper;
}

DrainState
DRAMSys::drain()
{
    return dramSysWrapper.dramsys->idle() ? DrainState::Drained
                                          : DrainState::Draining;
}

void
DRAMSys::serialize(CheckpointOut &cp) const
{
    std::filesystem::path checkpointPath = CheckpointIn::dir();
    dramSysWrapper.dramsys->serialize(checkpointPath);
}

void
DRAMSys::unserialize(CheckpointIn &cp)
{
    std::filesystem::path checkpointPath = CheckpointIn::dir();
    dramSysWrapper.dramsys->deserialize(checkpointPath);
}

} // namespace memory
} // namespace gem5
