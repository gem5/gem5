/*
 * Copyright (c) 2014-2015 ARM Limited
 * All rights reserved
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Andreas Sandberg
 */

#include "gpucontrol.hh"
#include "gpu.hh"
#include "regutils.hh"

namespace NoMali {

typedef void (GPUControl::*GpuCmdHandler)(uint32_t);

const std::vector<GpuCmdHandler> GPUControl::cmds {
    &GPUControl::cmdNop,                     // GPU_COMMAND_NOP
    &GPUControl::cmdSoftReset,               // GPU_COMMAND_SOFT_RESET
    &GPUControl::cmdHardReset,               // GPU_COMMAND_HARD_RESET
    &GPUControl::cmdPerfCntClear,            // GPU_COMMAND_PRFCNT_CLEAR
    &GPUControl::cmdPerfCntSample,           // GPU_COMMAND_PRFCNT_SAMPLE
    &GPUControl::cmdCycleCountStart,         // GPU_COMMAND_CYCLE_COUNT_START
    &GPUControl::cmdCycleCountStop,          // GPU_COMMAND_COUNT_STOP
    &GPUControl::cmdCleanCaches,             // GPU_COMMAND_CLEAN_CACHES
    &GPUControl::cmdCleanInvCaches,          // GPU_COMMAND_CLEAN_INV_CACHES
};

GPUControl::GPUControl(GPU &_gpu)
    : GPUBlockInt(_gpu,
                  RegAddr(GPU_IRQ_RAWSTAT),
                  RegAddr(GPU_IRQ_CLEAR),
                  RegAddr(GPU_IRQ_MASK),
                  RegAddr(GPU_IRQ_STATUS))
{
}

GPUControl::~GPUControl()
{
}

void
GPUControl::reset()
{
    GPUBlock::reset();
}

void
GPUControl::writeReg(RegAddr addr, uint32_t value)
{
    switch (addr.value) {
      case GPU_IRQ_RAWSTAT:
      case GPU_IRQ_CLEAR:
      case GPU_IRQ_MASK:
      case GPU_IRQ_STATUS:
        GPUBlockInt::writeReg(addr, value);
        break;

      case GPU_COMMAND:
        gpuCommand(value);
        break;

      case SHADER_PWRON_LO:
      case SHADER_PWRON_HI:
      case TILER_PWRON_LO:
      case TILER_PWRON_HI:
      case L2_PWRON_LO:
      case L2_PWRON_HI:
      case L3_PWRON_LO:
      case L3_PWRON_HI: {
          const RegAddr ready_reg(SHADER_READY_LO +
                                  (addr.value - SHADER_PWRON_LO));
          const RegAddr present_reg(SHADER_PRESENT_LO +
                                    (addr.value - SHADER_PWRON_LO));

          regs[ready_reg] |= value & regs[present_reg];
          raiseInterrupt(POWER_CHANGED_SINGLE | POWER_CHANGED_ALL);
      } break;

      case SHADER_PWROFF_LO:
      case SHADER_PWROFF_HI:
      case TILER_PWROFF_LO:
      case TILER_PWROFF_HI:
      case L2_PWROFF_LO:
      case L2_PWROFF_HI:
      case L3_PWROFF_LO:
      case L3_PWROFF_HI: {
          const RegAddr ready_reg(SHADER_READY_LO +
                                  (addr.value - SHADER_PWROFF_LO));

          regs[ready_reg] &= ~value;
          raiseInterrupt(POWER_CHANGED_SINGLE | POWER_CHANGED_ALL);
      } break;

      default:
        // Ignore writes by default
        break;
    };
}

void
GPUControl::onInterrupt(int set)
{
    gpu.intGPU(set);
}

void
GPUControl::gpuCommand(uint32_t cmd)
{
    if (cmd < cmds.size())
        (this->*cmds[cmd])(cmd);
}

void
GPUControl::cmdNop(uint32_t cmd)
{
}

void
GPUControl::cmdHardReset(uint32_t cmd)
{
    gpu.reset();
    raiseInterrupt(RESET_COMPLETED);
}

void
GPUControl::cmdSoftReset(uint32_t cmd)
{
    gpu.reset();
    raiseInterrupt(RESET_COMPLETED);
}

void
GPUControl::cmdPerfCntClear(uint32_t cmd)
{
}

void
GPUControl::cmdPerfCntSample(uint32_t cmd)
{
    raiseInterrupt(PRFCNT_SAMPLE_COMPLETED);
}

void
GPUControl::cmdCycleCountStart(uint32_t cmd)
{
}

void
GPUControl::cmdCycleCountStop(uint32_t cmd)
{
}

void
GPUControl::cmdCleanCaches(uint32_t cmd)
{
    raiseInterrupt(CLEAN_CACHES_COMPLETED);
}

void
GPUControl::cmdCleanInvCaches(uint32_t cmd)
{
    raiseInterrupt(CLEAN_CACHES_COMPLETED);
}

}
