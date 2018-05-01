/*
 * Copyright (c) 2016 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Mark Wyse
 */

#ifndef __REGISTER_MANAGER_POLICY_HH__
#define __REGISTER_MANAGER_POLICY_HH__

#include <cstdint>

class ComputeUnit;
class HSAQueueEntry;
class Wavefront;

/**
 * Register Manager Policy abstract class
 *
 * A Register Manager Policy implements all of the functionality
 * of the Register Manager, including register mapping, allocation,
 * and freeing. Different policies may be implemented that support
 * different architectures or different methods of mapping and
 * allocation.
 */
class RegisterManagerPolicy
{
  public:
    virtual void setParent(ComputeUnit *_cu) { cu = _cu; }

    // Execute: called by RenameStage::execute()
    virtual void exec() = 0;

    // provide virtual to physical register mapping
    virtual int mapVgpr(Wavefront* w, int vgprIndex) = 0;
    virtual int mapSgpr(Wavefront* w, int sgprIndex) = 0;

    // check if requested number of vector registers can be allocated
    virtual bool canAllocateVgprs(int simdId, int nWfs, int demandPerWf) = 0;
    // check if requested number of scalar registers can be allocated
    // machine ISA only
    virtual bool canAllocateSgprs(int simdId, int nWfs, int demandPerWf) = 0;

    // allocate vector registers and reserve from register pool
    virtual void allocateRegisters(Wavefront *w, int vectorDemand,
        int scalarDemand) = 0;

    // free all remaining registers held by specified WF
    virtual void freeRegisters(Wavefront *w) = 0;

    // stats
    virtual void regStats() = 0;

  protected:
    ComputeUnit *cu;
};

#endif // __REGISTER_MANAGER_POLICY_HH__
