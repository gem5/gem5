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

#ifndef __DEV_ARM_SSC_H__
#define __DEV_ARM_SSC_H__

#include "dev/io_device.hh"
#include "dev/reg_bank.hh"
#include "params/SysSecCtrl.hh"

namespace gem5
{

/** System Security Control registers */
class SysSecCtrl : public BasicPioDevice
{
  public:
    PARAMS(SysSecCtrl);
    SysSecCtrl(const Params &p);

    /**
     * Handle a read to the device
     * @param pkt The memory request.
     * @param data Where to put the data.
     */
    Tick read(PacketPtr pkt) override;

    /**
     * All writes are simply ignored.
     * @param pkt The memory request.
     * @param data the data
     */
    Tick write(PacketPtr pkt) override;

  protected:
    using Register = RegisterBankLE::Register32LE;
    using Space = RegisterBankLE::RegisterRaz;
    template <size_t Size>
    using Block = RegisterBankLE::RegisterLBuf<Size>;

    Register sscDbgcfgStat;
    Register sscDbgcfgSet;
    Register sscDbgcfgClr;
    Space space0;
    Register sscAuxDbgcfg;
    Space space1;
    Register sscAuxGpretn;
    Space space2;
    Register sscVersion;
    Space space3;
    Block<0x80> sscSwScratch;
    Space space4;
    Block<0x100> sscSwCap;
    Register sscSwCapCtrl;
    Space space5;
    Register sscChipIdSt;
    Space space6;
    Register sscPid4;
    Space space7;
    Register sscPid0;
    Register sscPid1;
    Register sscPid2;
    Space space8;
    Register compid0;
    Register compid1;
    Register compid2;
    Register compid3;

    RegisterBankLE regBank;
};

} // namespace gem5

#endif
