/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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
 *
 * Authors: Kevin Lim
 *          Gabe Black
 *          Steve Reinhardt
 */

#include "cpu/o3/free_list.hh"
#include "cpu/o3/regfile.hh"


PhysRegFile::PhysRegFile(unsigned _numPhysicalIntRegs,
                         unsigned _numPhysicalFloatRegs,
                         unsigned _numPhysicalCCRegs)
    : intRegFile(_numPhysicalIntRegs),
      floatRegFile(_numPhysicalFloatRegs),
      ccRegFile(_numPhysicalCCRegs),
      baseFloatRegIndex(_numPhysicalIntRegs),
      baseCCRegIndex(_numPhysicalIntRegs + _numPhysicalFloatRegs),
      totalNumRegs(_numPhysicalIntRegs
                   + _numPhysicalFloatRegs
                   + _numPhysicalCCRegs)
{
    if (TheISA::NumCCRegs == 0 && _numPhysicalCCRegs != 0) {
        // Just make this a warning and go ahead and allocate them
        // anyway, to keep from having to add checks everywhere
        warn("Non-zero number of physical CC regs specified, even though\n"
             "    ISA does not use them.\n");
    }
}


void
PhysRegFile::initFreeList(UnifiedFreeList *freeList)
{
    // Initialize the free lists.
    PhysRegIndex reg_idx = 0;

    // The initial batch of registers are the integer ones
    while (reg_idx < baseFloatRegIndex) {
        freeList->addIntReg(reg_idx++);
    }

    // The next batch of the registers are the floating-point physical
    // registers; put them onto the floating-point free list.
    while (reg_idx < baseCCRegIndex) {
        freeList->addFloatReg(reg_idx++);
    }

    // The rest of the registers are the condition-code physical
    // registers; put them onto the condition-code free list.
    while (reg_idx < totalNumRegs) {
        freeList->addCCReg(reg_idx++);
    }
}
