/*
 * Copyright (c) 2014-2015 ARM Limited
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
 *
 * Authors: Andreas Sandberg
 */

#include "dev/arm/gpu_nomali.hh"

#include "debug/NoMali.hh"
#include "dev/arm/base_gic.hh"
#include "dev/arm/realview.hh"
#include "enums/MemoryMode.hh"
#include "mem/packet_access.hh"
#include "params/NoMaliGpu.hh"

NoMaliGpu::NoMaliGpu(const NoMaliGpuParams *p)
    : PioDevice(p),
      pioAddr(p->pio_addr),
      platform(p->platform),
      interruptMap{
          { NOMALI_INT_GPU, p->int_gpu },
          { NOMALI_INT_JOB, p->int_job },
          { NOMALI_INT_MMU, p->int_mmu },
      }
{
    if (nomali_api_version() != NOMALI_API_VERSION)
        panic("NoMali library API mismatch!\n");

    /* Setup the GPU configuration based on our param struct */
    nomali_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    switch (p->gpu_type) {
      case Enums::T60x:
        cfg.type = NOMALI_GPU_T60X;
        break;

      case Enums::T62x:
        cfg.type = NOMALI_GPU_T62X;
        break;

      case Enums::T760:
        cfg.type = NOMALI_GPU_T760;
        break;

      default:
        fatal("Unknown GPU type\n");
    }

    cfg.ver_maj = p->ver_maj;
    cfg.ver_min = p->ver_min;
    cfg.ver_status = p->ver_status;

    panicOnErr(
        nomali_create(&nomali, &cfg),
        "Failed to instantiate NoMali");


    /* Setup an interrupt callback */
    nomali_callback_t cbk_int;
    cbk_int.type = NOMALI_CALLBACK_INT;
    cbk_int.usr = (void *)this;
    cbk_int.func.interrupt = NoMaliGpu::_interrupt;
    panicOnErr(
        nomali_set_callback(nomali, &cbk_int),
        "Failed to setup interrupt callback");

    panicOnErr(
        nomali_get_info(nomali, &nomaliInfo),
        "Failed to get NoMali information struct");
}

NoMaliGpu::~NoMaliGpu()
{
    nomali_destroy(nomali);
}


void
NoMaliGpu::serialize(CheckpointOut &cp) const
{
    std::vector<uint32_t> regs(nomaliInfo.reg_size >> 2);

    for (int i = 0; i < nomaliInfo.reg_size; i += 4)
        regs[i >> 2] = readRegRaw(i);

    SERIALIZE_CONTAINER(regs);
}

void
NoMaliGpu::unserialize(CheckpointIn &cp)
{
    std::vector<uint32_t> regs(nomaliInfo.reg_size >> 2);

    UNSERIALIZE_CONTAINER(regs);

    for (int i = 0; i < nomaliInfo.reg_size; i += 4)
        writeRegRaw(i, regs[i >> 2]);
}

Tick
NoMaliGpu::read(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr);
    const Addr addr(pkt->getAddr() - pioAddr);
    const unsigned size(pkt->getSize());

    if (addr + size >= nomaliInfo.reg_size)
        panic("GPU register '0x%x' out of range!\n", addr);

    if (size != 4)
        panic("Unexpected GPU register read size: %i\n", size);
    else if (addr & 0x3)
        panic("Unaligned GPU read: %i\n", size);

    pkt->set<uint32_t>(readReg(addr));
    pkt->makeResponse();

    return 0;
}

Tick
NoMaliGpu::write(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr);
    const Addr addr(pkt->getAddr() - pioAddr);
    const unsigned size(pkt->getSize());

    if (addr + size >= nomaliInfo.reg_size)
        panic("GPU register '0x%x' out of range!\n", addr);

    if (size != 4)
        panic("Unexpected GPU register write size: %i\n", size);
    else if (addr & 0x3)
        panic("Unaligned GPU write: %i\n", size);

    writeReg(addr, pkt->get<uint32_t>());
    pkt->makeAtomicResponse();

    return 0;
}

AddrRangeList
NoMaliGpu::getAddrRanges() const
{
    return AddrRangeList({ RangeSize(pioAddr, nomaliInfo.reg_size) });
}

uint32_t
NoMaliGpu::readReg(nomali_addr_t reg)
{
    uint32_t value;

    panicOnErr(
        nomali_reg_read(nomali, &value, reg),
        "GPU register read failed");

    DPRINTF(NoMali, "readReg(0x%x): 0x%x\n",
            reg, value);

    return value;
}


void
NoMaliGpu::writeReg(nomali_addr_t reg, uint32_t value)
{
    DPRINTF(NoMali, "writeReg(0x%x, 0x%x)\n",
            reg, value);

    panicOnErr(
        nomali_reg_write(nomali, reg, value),
        "GPU register write failed");
}

uint32_t
NoMaliGpu::readRegRaw(nomali_addr_t reg) const
{
    uint32_t value;

    panicOnErr(
        nomali_reg_read_raw(nomali, &value, reg),
        "GPU raw register read failed");

    return value;
}


void
NoMaliGpu::writeRegRaw(nomali_addr_t reg, uint32_t value)
{
    panicOnErr(
        nomali_reg_write_raw(nomali, reg, value),
        "GPU raw register write failed");
}

void
NoMaliGpu::_interrupt(nomali_handle_t h, void *usr, nomali_int_t intno, int set)
{
    NoMaliGpu *_this(static_cast<NoMaliGpu *>(usr));

    _this->onInterrupt(h, intno, !!set);
}

void
NoMaliGpu::onInterrupt(nomali_handle_t h, nomali_int_t intno, bool set)
{
    const auto it_int(interruptMap.find(intno));
    if (it_int == interruptMap.end())
        panic("Unhandled interrupt from NoMali: %i\n", intno);

    DPRINTF(NoMali, "Interrupt %i->%i: %i\n",
            intno, it_int->second, set);

    assert(platform);
    assert(platform->gic);

    if (set)
        platform->gic->sendInt(it_int->second);
    else
        platform->gic->clearInt(it_int->second);
}

void
NoMaliGpu::gpuPanic(nomali_error_t err, const char *msg)
{
    panic("%s: %s\n", msg, nomali_errstr(err));
}

NoMaliGpu *
NoMaliGpuParams::create()
{
    return new NoMaliGpu(this);
}
