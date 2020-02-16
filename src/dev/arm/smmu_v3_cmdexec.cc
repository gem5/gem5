/*
 * Copyright (c) 2013, 2018-2019 ARM Limited
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

#include "dev/arm/smmu_v3_cmdexec.hh"

#include "base/bitfield.hh"
#include "dev/arm/smmu_v3.hh"

void
SMMUCommandExecProcess::main(Yield &yield)
{
    SMMUAction a;
    a.type = ACTION_INITIAL_NOP;
    a.pkt = NULL;
    a.ifc = nullptr;
    a.delay = 0;
    yield(a);

    while (true) {
        busy = true;

        while (true) {
            // Masking depending on CMDQ_BASE.LOG2SIZE (log(number of
            // queue entries)). Example: a value of 0b101 (32 entries)
            // generates a 0b11111 mask.
            int size_mask = mask(
                smmu.regs.cmdq_base & Q_BASE_SIZE_MASK);

            // In this case the wrap bit is considered (+1)
            int size_mask_wrap = mask(
                (smmu.regs.cmdq_base & Q_BASE_SIZE_MASK) + 1);

            if ((smmu.regs.cmdq_cons & size_mask_wrap) ==
                    (smmu.regs.cmdq_prod & size_mask_wrap))
                break; // command queue empty

            Addr cmd_addr =
                (smmu.regs.cmdq_base & Q_BASE_ADDR_MASK) +
                (smmu.regs.cmdq_cons & size_mask) * sizeof(SMMUCommand);

            // This deliberately resets the error field in cmdq_cons!
            smmu.regs.cmdq_cons = (smmu.regs.cmdq_cons + 1) & size_mask_wrap;

            doRead(yield, cmd_addr, &cmd, sizeof(SMMUCommand));
            smmu.processCommand(cmd);
        }

        busy = false;
        // No more commands to process, signal the SMMU as drained
        smmu.signalDrainDone();

        doSleep(yield);
    }
}
