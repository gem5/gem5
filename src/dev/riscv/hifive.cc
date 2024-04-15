/*
 * Copyright (c) 2021 Huawei International
 * Copyright (c) 2023 Google LLC
 * All rights reserved
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
 */

#include "dev/riscv/hifive.hh"

#include "dev/riscv/clint.hh"
#include "dev/riscv/plic.hh"
#include "params/HiFiveBase.hh"
#include "sim/system.hh"

namespace gem5
{

using namespace RiscvISA;

HiFiveBase::HiFiveBase(const Params &params)
    : Platform(params),
      clint(params.clint),
      plic(params.plic),
      uartIntID(params.uart_int_id)
{
    fatal_if(clint == nullptr, "CLINT should not be NULL");
    fatal_if(plic == nullptr, "PLIC should not be NULL");
}

void
HiFiveBase::postConsoleInt()
{
    plic->post(uartIntID);
}

void
HiFiveBase::clearConsoleInt()
{
    plic->clear(uartIntID);
}

void
HiFiveBase::postPciInt(int line)
{
    plic->post(line);
}

void
HiFiveBase::clearPciInt(int line)
{
    plic->clear(line);
}

void
HiFiveBase::serialize(CheckpointOut &cp) const
{}

void
HiFiveBase::unserialize(CheckpointIn &cp)
{}

} // namespace gem5
