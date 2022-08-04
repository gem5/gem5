/*
* Copyright (c) 2012, 2018 ARM Limited
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
 * Copyright (c) 2007-2008 The Florida State University
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

#ifndef __ARM_PROCESS_HH__
#define __ARM_PROCESS_HH__

#include <string>
#include <vector>

#include "arch/arm/regs/int.hh"
#include "base/loader/object_file.hh"
#include "mem/page_table.hh"
#include "sim/process.hh"
#include "sim/syscall_abi.hh"

namespace gem5
{

class ArmProcess : public Process
{
  protected:
    loader::Arch arch;
    ArmProcess(const ProcessParams &params, loader::ObjectFile *objFile,
               loader::Arch _arch);
    template<class IntType>
    void argsInit(int pageSize, const RegId &spId);

    template<class IntType>
    IntType
    armHwcap() const
    {
        return static_cast<IntType>(armHwcapImpl());
    }

    /**
     * AT_HWCAP is 32-bit wide on AArch64 as well so we can
     * safely return an uint32_t */
    virtual uint32_t armHwcapImpl() const = 0;
};

class ArmProcess32 : public ArmProcess
{
  public:
    ArmProcess32(const ProcessParams &params, loader::ObjectFile *objFile,
                 loader::Arch _arch);

  protected:
    void initState() override;

    /** AArch32 AT_HWCAP */
    uint32_t armHwcapImpl() const override;
};

class ArmProcess64 : public ArmProcess
{
  public:
    ArmProcess64(const ProcessParams &params, loader::ObjectFile *objFile,
                 loader::Arch _arch);

  protected:
    void initState() override;

    /** AArch64 AT_HWCAP */
    uint32_t armHwcapImpl() const override;
};

} // namespace gem5

#endif // __ARM_PROCESS_HH__
