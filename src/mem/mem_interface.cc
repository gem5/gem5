/*
 * Copyright (c) 2010-2020 ARM Limited
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
 * Copyright (c) 2013 Amin Farmahini-Farahani
 * All rights reserved.
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

#include "mem/mem_interface.hh"

#include "base/bitfield.hh"
#include "base/cprintf.hh"
#include "base/trace.hh"
#include "sim/system.hh"

namespace gem5
{

namespace memory
{

MemInterface::MemInterface(const MemInterfaceParams &_p)
    : AbstractMemory(_p),
      addrMapping(_p.addr_mapping),
      burstSize((_p.devices_per_rank * _p.burst_length * _p.device_bus_width) /
                8),
      deviceSize(_p.device_size),
      deviceRowBufferSize(_p.device_rowbuffer_size),
      devicesPerRank(_p.devices_per_rank),
      rowBufferSize(devicesPerRank * deviceRowBufferSize),
      burstsPerRowBuffer(rowBufferSize / burstSize),
      burstsPerStripe(range.interleaved() ? range.granularity() / burstSize :
                                            1),
      ranksPerChannel(_p.ranks_per_channel),
      banksPerRank(_p.banks_per_rank),
      rowsPerBank(0),
      tCK(_p.tCK),
      tCS(_p.tCS),
      tBURST(_p.tBURST),
      tRTW(_p.tRTW),
      tWTR(_p.tWTR),
      readBufferSize(_p.read_buffer_size),
      writeBufferSize(_p.write_buffer_size),
      numWritesQueued(0)
{}

void
MemInterface::setCtrl(MemCtrl *_ctrl, unsigned int command_window,
                      uint8_t pseudo_channel)
{
    ctrl = _ctrl;
    maxCommandsPerWindow = command_window / tCK;
    // setting the pseudo channel number for this interface
    pseudoChannel = pseudo_channel;
}

} // namespace memory
} // namespace gem5
