/*
 * Copyright (c) 2014-2016 ARM Limited
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

#include "dev/arm/gpu_nomali.hh"

#include "debug/NoMali.hh"
#include "dev/arm/base_gic.hh"
#include "dev/arm/realview.hh"
#include "enums/MemoryMode.hh"
#include "mem/packet_access.hh"
#include "nomali/lib/mali_midg_regmap.h"
#include "params/CustomNoMaliGpu.hh"
#include "params/NoMaliGpu.hh"

static const std::map<Enums::NoMaliGpuType, nomali_gpu_type_t> gpuTypeMap{
    { Enums::T60x, NOMALI_GPU_T60X },
    { Enums::T62x, NOMALI_GPU_T62X },
    { Enums::T760, NOMALI_GPU_T760 },
};

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

    const auto it_gpu(gpuTypeMap.find(p->gpu_type));
    if (it_gpu == gpuTypeMap.end()) {
        fatal("Unrecognized GPU type: %s (%i)\n",
              Enums::NoMaliGpuTypeStrings[p->gpu_type], p->gpu_type);
    }
    cfg.type = it_gpu->second;

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
    setCallback(cbk_int);

    /* Setup a reset callback */
    nomali_callback_t cbk_rst;
    cbk_rst.type = NOMALI_CALLBACK_RESET;
    cbk_rst.usr = (void *)this;
    cbk_rst.func.reset = NoMaliGpu::_reset;
    setCallback(cbk_rst);

    panicOnErr(
        nomali_get_info(nomali, &nomaliInfo),
        "Failed to get NoMali information struct");
}

NoMaliGpu::~NoMaliGpu()
{
    nomali_destroy(nomali);
}


void
NoMaliGpu::init()
{
    PioDevice::init();

    /* Reset the GPU here since the reset callback won't have been
     * installed when the GPU was reset at instantiation time.
     */
    reset();
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

    pkt->setLE<uint32_t>(readReg(addr));
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

    writeReg(addr, pkt->getLE<uint32_t>());
    pkt->makeAtomicResponse();

    return 0;
}

AddrRangeList
NoMaliGpu::getAddrRanges() const
{
    return AddrRangeList({ RangeSize(pioAddr, nomaliInfo.reg_size) });
}

void
NoMaliGpu::reset()
{
    DPRINTF(NoMali, "reset()\n");

    panicOnErr(
        nomali_reset(nomali),
        "Failed to reset GPU");
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

bool
NoMaliGpu::intState(nomali_int_t intno)
{
    int state = 0;
    panicOnErr(
        nomali_int_state(nomali, &state, intno),
        "Failed to get interrupt state");

    return !!state;
}

void
NoMaliGpu::gpuPanic(nomali_error_t err, const char *msg)
{
    panic("%s: %s\n", msg, nomali_errstr(err));
}


void
NoMaliGpu::onInterrupt(nomali_int_t intno, bool set)
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
NoMaliGpu::onReset()
{
    DPRINTF(NoMali, "Reset\n");
}

void
NoMaliGpu::setCallback(const nomali_callback_t &callback)
{
    DPRINTF(NoMali, "Registering callback %i\n",
            callback.type);

    panicOnErr(
        nomali_set_callback(nomali, &callback),
        "Failed to register callback");
}

void
NoMaliGpu::_interrupt(nomali_handle_t h, void *usr,
                      nomali_int_t intno, int set)
{
    NoMaliGpu *_this(static_cast<NoMaliGpu *>(usr));

    _this->onInterrupt(intno, !!set);
}

void
NoMaliGpu::_reset(nomali_handle_t h, void *usr)
{
    NoMaliGpu *_this(static_cast<NoMaliGpu *>(usr));

    _this->onReset();
}


CustomNoMaliGpu::CustomNoMaliGpu(const CustomNoMaliGpuParams *p)
    : NoMaliGpu(p),
      idRegs{
        { GPU_CONTROL_REG(GPU_ID), p->gpu_id },
        { GPU_CONTROL_REG(L2_FEATURES), p->l2_features },
        { GPU_CONTROL_REG(TILER_FEATURES), p->tiler_features },
        { GPU_CONTROL_REG(MEM_FEATURES), p->mem_features },
        { GPU_CONTROL_REG(MMU_FEATURES), p->mmu_features },
        { GPU_CONTROL_REG(AS_PRESENT), p->as_present },
        { GPU_CONTROL_REG(JS_PRESENT), p->js_present },

        { GPU_CONTROL_REG(THREAD_MAX_THREADS), p->thread_max_threads },
        { GPU_CONTROL_REG(THREAD_MAX_WORKGROUP_SIZE),
          p->thread_max_workgroup_size },
        { GPU_CONTROL_REG(THREAD_MAX_BARRIER_SIZE),
          p->thread_max_barrier_size },
        { GPU_CONTROL_REG(THREAD_FEATURES), p->thread_features },

        { GPU_CONTROL_REG(SHADER_PRESENT_LO), bits(p->shader_present, 31, 0) },
        { GPU_CONTROL_REG(SHADER_PRESENT_HI), bits(p->shader_present, 63, 32) },
        { GPU_CONTROL_REG(TILER_PRESENT_LO), bits(p->tiler_present, 31, 0) },
        { GPU_CONTROL_REG(TILER_PRESENT_HI), bits(p->tiler_present, 63, 32) },
        { GPU_CONTROL_REG(L2_PRESENT_LO), bits(p->l2_present, 31, 0) },
        { GPU_CONTROL_REG(L2_PRESENT_HI), bits(p->l2_present, 63, 32) },
      }
{
    fatal_if(p->texture_features.size() > 3,
             "Too many texture feature registers specified (%i)\n",
             p->texture_features.size());

    fatal_if(p->js_features.size() > 16,
             "Too many job slot feature registers specified (%i)\n",
             p->js_features.size());

    for (int i = 0; i < p->texture_features.size(); i++)
        idRegs[TEXTURE_FEATURES_REG(i)] = p->texture_features[i];

    for (int i = 0; i < p->js_features.size(); i++)
        idRegs[JS_FEATURES_REG(i)] = p->js_features[i];
}

CustomNoMaliGpu::~CustomNoMaliGpu()
{
}

void
CustomNoMaliGpu::onReset()
{
    NoMaliGpu::onReset();

    for (const auto &reg : idRegs)
        writeRegRaw(reg.first, reg.second);
}



NoMaliGpu *
NoMaliGpuParams::create()
{
    return new NoMaliGpu(this);
}

CustomNoMaliGpu *
CustomNoMaliGpuParams::create()
{
    return new CustomNoMaliGpu(this);
}
