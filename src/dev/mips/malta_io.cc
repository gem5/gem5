/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

/** @file
 * Malta I/O including PIC, PIT, RTC, DMA
 */

#include "dev/mips/malta_io.hh"

#include <sys/time.h>

#include <deque>
#include <string>
#include <vector>

#include "base/time.hh"
#include "base/trace.hh"
#include "debug/Malta.hh"
#include "dev/mips/malta.hh"
#include "dev/mips/malta_cchip.hh"
#include "dev/mips/maltareg.h"
#include "dev/rtcreg.h"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "mem/port.hh"
#include "params/MaltaIO.hh"
#include "sim/system.hh"

namespace gem5
{

MaltaIO::RTC::RTC(const std::string &name, const MaltaIOParams &p)
    : MC146818(p.malta, name, p.time, p.year_is_bcd, p.frequency),
      malta(p.malta)
{}

MaltaIO::MaltaIO(const Params &p)
    : BasicPioDevice(p, 0x100),
      malta(p.malta),
      pitimer(this, p.name + "pitimer"),
      rtc(p.name + ".rtc", p)
{
    // set the back pointer from malta to myself
    malta->io = this;

    timerData = 0;
    picr = 0;
    picInterrupting = false;
}

Tick
MaltaIO::frequency() const
{
    return sim_clock::Frequency / params().frequency;
}

Tick
MaltaIO::read(PacketPtr pkt)
{
    panic("MaltaIO::read(...) not implemented inside malta_io.cc");
    return pioDelay;
}

Tick
MaltaIO::write(PacketPtr pkt)
{
    panic("MaltaIO::write(...) not implemented inside malta_io.cc");
    return pioDelay;
}

void
MaltaIO::postIntr(uint8_t interrupt)
{
    malta->cchip->postIntr(interrupt);
    DPRINTF(Malta, "posting pic interrupt to cchip\n");
}

void
MaltaIO::clearIntr(uint8_t interrupt)
{
    malta->cchip->clearIntr(interrupt);
    DPRINTF(Malta, "clear pic interrupt to cchip\n");
}

void
MaltaIO::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(timerData);
    SERIALIZE_SCALAR(mask1);
    SERIALIZE_SCALAR(mask2);
    SERIALIZE_SCALAR(mode1);
    SERIALIZE_SCALAR(mode2);
    SERIALIZE_SCALAR(picr);
    SERIALIZE_SCALAR(picInterrupting);

    // Serialize the timers
    pitimer.serialize("pitimer", cp);
    rtc.serialize("rtc", cp);
}

void
MaltaIO::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(timerData);
    UNSERIALIZE_SCALAR(mask1);
    UNSERIALIZE_SCALAR(mask2);
    UNSERIALIZE_SCALAR(mode1);
    UNSERIALIZE_SCALAR(mode2);
    UNSERIALIZE_SCALAR(picr);
    UNSERIALIZE_SCALAR(picInterrupting);

    // Unserialize the timers
    pitimer.unserialize("pitimer", cp);
    rtc.unserialize("rtc", cp);
}

void
MaltaIO::startup()
{
    rtc.startup();
    pitimer.startup();
}

} // namespace gem5
