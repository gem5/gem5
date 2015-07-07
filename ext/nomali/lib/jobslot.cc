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

#include "jobslot.hh"

#include <cassert>
#include <cstdlib>

#include "jobcontrol.hh"
#include "gpu.hh"
#include "regutils.hh"

namespace NoMali {

const std::vector<JobSlot::cmd_t> JobSlot::cmds {
    &JobSlot::cmdNop,                      // JSn_COMMAND_NOP
    &JobSlot::cmdStart,                    // JSn_COMMAND_START
    &JobSlot::cmdSoftStop,                 // JSn_COMMAND_SOFT_STOP
    &JobSlot::cmdHardStop,                 // JSn_COMMAND_HARD_STOP
    &JobSlot::cmdSoftStop0,                // JSn_COMMAND_SOFT_STOP_0
    &JobSlot::cmdHardStop0,                // JSn_COMMAND_HARD_STOP_0
    &JobSlot::cmdSoftStop1,                // JSn_COMMAND_SOFT_STOP_1
    &JobSlot::cmdHardStop1,                // JSn_COMMAND_HARD_STOP_1
};

JobSlot::JobSlot(GPU &_gpu, JobControl &_jc, uint8_t _id)
    : GPUBlock(_gpu, JSn_NO_REGS),
      id(_id),
      jc(_jc)
{
}

JobSlot::JobSlot(JobSlot &&rhs)
    : GPUBlock(std::move(rhs)),
      id(std::move(rhs.id)),
      jc(rhs.jc)
{
}

JobSlot::~JobSlot()
{
}

void
JobSlot::writeReg(RegAddr addr, uint32_t value)
{
    switch (addr.value) {
      case JSn_COMMAND:
        jobCommand(value);
        break;

      case JSn_COMMAND_NEXT:
        regs[addr] = value;
        tryStart();
        break;

      case JSn_HEAD_NEXT_LO:
      case JSn_HEAD_NEXT_HI:
      case JSn_AFFINITY_NEXT_LO:
      case JSn_AFFINITY_NEXT_HI:
      case JSn_CONFIG_NEXT:
        GPUBlock::writeReg(addr, value);
        break;

      default:
        // Ignore writes by default
        break;
    };
}

bool
JobSlot::active() const
{
    return false;
}

bool
JobSlot::activeNext() const
{
    return regs[RegAddr(JSn_COMMAND_NEXT)] == JSn_COMMAND_START;
}

void
JobSlot::tryStart()
{
    // Only actually start something if the next command is start
    if (regs[RegAddr(JSn_COMMAND_NEXT)] != JSn_COMMAND_START )
        return;

    // Reset the status register
    regs[RegAddr(JSn_STATUS)] = 0;

    // Transfer the next job configuration to the active job
    // configuration
    regs.set64(RegAddr(JSn_HEAD_LO), regs.get64(RegAddr(JSn_HEAD_NEXT_LO)));
    regs.set64(RegAddr(JSn_TAIL_LO), regs.get64(RegAddr(JSn_HEAD_NEXT_LO)));
    regs.set64(RegAddr(JSn_AFFINITY_LO),
               regs.get64(RegAddr(JSn_AFFINITY_NEXT_LO)));
    regs[RegAddr(JSn_CONFIG)] = regs[RegAddr(JSn_CONFIG_NEXT)];
    regs[RegAddr(JSn_COMMAND)] = regs[RegAddr(JSn_COMMAND_NEXT)];

    // Reset the next job configuration
    regs.set64(RegAddr(JSn_HEAD_NEXT_LO), 0);
    regs.set64(RegAddr(JSn_AFFINITY_NEXT_LO), 0);
    regs[RegAddr(JSn_CONFIG_NEXT)] = 0;
    regs[RegAddr(JSn_COMMAND_NEXT)] = 0;

    runJob();
}

void
JobSlot::runJob()
{
    exitJob(Status(Status::CLASS_NOFAULT, 0, 1), // JSn_STATUS_DONE
            0); // Time stamp counter value
}

void
JobSlot::exitJob(Status status, uint64_t fault_address)
{
    assert(status.statusClass() == Status::CLASS_NOFAULT ||
           status.statusClass() == Status::CLASS_JOB);

    regs[RegAddr(JSn_STATUS)] = status.value;

    if (status.statusClass() == Status::CLASS_NOFAULT) {
        jc.jobDone(id);
    } else {
        jc.jobFailed(id);
    }
}

void
JobSlot::jobCommand(uint32_t cmd)
{
    if (cmd < cmds.size())
        (this->*cmds[cmd])(cmd);
}

void
JobSlot::cmdNop(uint32_t cmd)
{
    assert(cmd == JSn_COMMAND_NOP);
}

void
JobSlot::cmdStart(uint32_t cmd)
{
    assert(cmd == JSn_COMMAND_START);
    // The JSn_COMMAND_START should never be issued through the
    // JSn_COMMAND register. It should use the JSn_COMMAND_NEXT
    // register instead.
    abort();
}

void
JobSlot::cmdSoftStop(uint32_t cmd)
{
    assert(cmd == JSn_COMMAND_SOFT_STOP ||
           cmd == JSn_COMMAND_SOFT_STOP_0 ||
           cmd == JSn_COMMAND_SOFT_STOP_1);
}

void
JobSlot::cmdHardStop(uint32_t cmd)
{
    assert(cmd == JSn_COMMAND_HARD_STOP ||
           cmd == JSn_COMMAND_HARD_STOP_0 ||
           cmd == JSn_COMMAND_HARD_STOP_1);
}

void
JobSlot::cmdSoftStop0(uint32_t cmd)
{
    if (!(regs[RegAddr(JSn_CONFIG)] & JSn_CONFIG_JOB_CHAIN_FLAG))
        cmdSoftStop(cmd);
}

void
JobSlot::cmdHardStop0(uint32_t cmd)
{
    if (!(regs[RegAddr(JSn_CONFIG)] & JSn_CONFIG_JOB_CHAIN_FLAG))
        cmdHardStop(cmd);
}

void
JobSlot::cmdSoftStop1(uint32_t cmd)
{
    if (regs[RegAddr(JSn_CONFIG)] & JSn_CONFIG_JOB_CHAIN_FLAG)
        cmdSoftStop(cmd);
}

void
JobSlot::cmdHardStop1(uint32_t cmd)
{
    if (regs[RegAddr(JSn_CONFIG)] & JSn_CONFIG_JOB_CHAIN_FLAG)
        cmdHardStop(cmd);
}

}
