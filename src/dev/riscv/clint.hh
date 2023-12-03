/*
 * Copyright (c) 2021 Huawei International
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

#ifndef __DEV_RISCV_CLINT_HH__
#define __DEV_RISCV_CLINT_HH__

#include "arch/riscv/interrupts.hh"
#include "dev/intpin.hh"
#include "dev/io_device.hh"
#include "dev/mc146818.hh"
#include "dev/reg_bank.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/Clint.hh"
#include "sim/system.hh"

namespace gem5
{

using namespace RiscvISA;

/**
 * NOTE:
 * This implementation of CLINT is based on
 * the SiFive U54MC datasheet:
 * https://sifive.cdn.prismic.io/sifive/fab000f6-
 * 0e07-48d0-9602-e437d5367806_sifive_U54MC_rtl_
 * full_20G1.03.00_manual.pdf
 */

/**
 * Future improvement of the model can check
 * the current privilege mode and enforce
 * access control.
 */
class Clint : public BasicPioDevice
{
    // Params
  protected:
    System *system;
    int nThread;
    IntSinkPin<Clint> signal;

  public:
    typedef ClintParams Params;
    Clint(const Params &params);

    // RTC Signal
  public:
    /**
     * Timer tick callback. Separated from RTC class
     * for easier implementation of a separate RTC
     * PioDevice.
     */
    void raiseInterruptPin(int id);

    void
    lowerInterruptPin(int id)
    {}

    // Register bank
  public:
    /**
     * MMIO Registers
     * 0x0000 - 0x3FFF: msip (write-through to misc reg file)
     * ...:             reserved[0]
     * 0x4000 - 0xBFF7: mtimecmp
     * ...:             reserved[1]
     * 0xBFF8:          mtime (read-only)
     */
    class ClintRegisters : public RegisterBankLE
    {
      public:
        const Addr mtimecmpStart = 0x4000;
        const Addr mtimeStart = 0xBFF8;
        const Addr maxBankSize = 0xC000;

        std::vector<Register32> msip;
        std::vector<Register64> mtimecmp;
        Register64 mtime = { "mtime", 0 };
        std::vector<RegisterRaz> reserved;

        ClintRegisters(const std::string &name, Addr base, Clint *clint)
            : RegisterBankLE(name, base), clint(clint)
        {}

        Clint *clint;

        void init();

    } registers;

    using Register32 = ClintRegisters::Register32;

    uint32_t readMSIP(Register32 &reg, const int thread_id);
    void writeMSIP(Register32 &reg, const uint32_t &data, const int thread_id);

    // External API
  public:
    /**
     * PioDevice interface functions
     */
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    /**
     * SimObject functions
     */
    void init() override;
    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

} // namespace gem5

#endif // __DEV_RISCV_CLINT_HH__
