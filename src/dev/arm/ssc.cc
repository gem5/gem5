/*
 * Copyright (c) 2022 Arm Limited
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

#include "dev/arm/ssc.hh"

namespace gem5
{

SysSecCtrl::SysSecCtrl(const Params &p)
    : BasicPioDevice(p, 0x1000),
      sscDbgcfgStat("ssc_dbgcfg_stat", p.ssc_dbgcfg_stat),
      sscDbgcfgSet("ssc_dbgcfg_set"),
      sscDbgcfgClr("ssc_dbgcfg_clr"),
      space0("space0", 0x28 - 0x1c),
      sscAuxDbgcfg("ssc_aux_dbgcfg"),
      space1("space1", 0x4),
      sscAuxGpretn("ssc_aux_gpretn"),
      space2("space2", 0x40 - 0x34),
      sscVersion("ssc_version", p.ssc_version),
      space3("space3", 0x100 - 0x44),
      sscSwScratch("ssc_sw_scratch"),
      space4("space4", 0x200 - 0x180),
      sscSwCap("ssc_sw_cap"),
      sscSwCapCtrl("ssc_sw_capctrl"),
      space5("space5", 0x500 - 0x304),
      sscChipIdSt("ssc_chipid_st"),
      space6("space6", 0xfd0 - 0x504),
      sscPid4("ssc_pid4", p.ssc_pid4),
      space7("space7", 0xfe0 - 0xfd4),
      sscPid0("ssc_pid0", p.ssc_pid0),
      sscPid1("ssc_pid1", p.ssc_pid1),
      sscPid2("ssc_pid2", p.ssc_pid2),
      space8("space8", 0xff0 - 0xfec),
      compid0("compid0", p.compid0),
      compid1("compid1", p.compid1),
      compid2("compid2", p.compid2),
      compid3("compid3", p.compid3),
      regBank("ssc", 0x0010)
{
    // RO registers
    sscDbgcfgStat.readonly();
    sscVersion.readonly();
    sscChipIdSt.readonly();
    sscPid0.readonly();
    sscPid1.readonly();
    sscPid2.readonly();
    sscPid4.readonly();
    compid0.readonly();
    compid1.readonly();
    compid2.readonly();
    compid3.readonly();

    /* clang-format off */
    regBank.addRegisters({
        sscDbgcfgStat, sscDbgcfgSet, sscDbgcfgClr,
        space0,
        sscAuxDbgcfg,
        space1,
        sscAuxGpretn,
        space2,
        sscVersion,
        space3,
        sscSwScratch,
        space4,
        sscSwCap, sscSwCapCtrl,
        space5,
        sscChipIdSt,
        space6,
        sscPid4,
        space7,
        sscPid0, sscPid1, sscPid2,
        space8,
        compid0, compid1, compid2, compid3,
    });
    /* clang-format on */
}

Tick
SysSecCtrl::read(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    Addr daddr = pkt->getAddr() - pioAddr;

    regBank.read(daddr, pkt->getPtr<void>(), pkt->getSize());

    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
SysSecCtrl::write(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    Addr daddr = pkt->getAddr() - pioAddr;

    regBank.write(daddr, pkt->getPtr<void>(), pkt->getSize());

    pkt->makeAtomicResponse();
    return pioDelay;
}

} // namespace gem5
