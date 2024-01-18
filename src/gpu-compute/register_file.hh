/*
 * Copyright (c) 2015-2017 Advanced Micro Devices, Inc.
 * All rights reserved.
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
 */

#ifndef __REGISTER_FILE_HH__
#define __REGISTER_FILE_HH__

#include <limits>
#include <vector>

#include "base/statistics.hh"
#include "base/types.hh"
#include "gpu-compute/misc.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class ComputeUnit;
class Shader;
class PoolManager;
class Wavefront;

struct RegisterFileParams;

// Abstract Register File
// This register file class can be inherited from to create both
// scalar and vector register files.
class RegisterFile : public SimObject
{
  public:
    RegisterFile(const RegisterFileParams &p);
    virtual ~RegisterFile();
    virtual void setParent(ComputeUnit *_computeUnit);
    int numRegs() const { return _numRegs; }

    // State functions

    // Scoreboard functions
    virtual bool operandsReady(Wavefront *w, GPUDynInstPtr ii) const;
    virtual bool regBusy(int idx) const;
    virtual void markReg(int regIdx, bool value);

    // Abstract Register Event
    class RegisterEvent : public Event
    {
      protected:
        RegisterFile *rf;
        int regIdx;

      public:
        RegisterEvent(RegisterFile *_rf, int _regIdx)
            : rf(_rf), regIdx(_regIdx) { setFlags(AutoDelete); }
    };

    // Register Event to mark a register as free in the scoreboard/busy vector
    class MarkRegFreeScbEvent : public RegisterEvent
    {
      public:
        MarkRegFreeScbEvent(RegisterFile *_rf, int _regIdx)
            : RegisterEvent(_rf, _regIdx) { }
        void process();
    };

    // Register Event to mark a register as busy in the scoreboard/busy vector
    class MarkRegBusyScbEvent : public RegisterEvent
    {
      public:
        MarkRegBusyScbEvent(RegisterFile *_rf, int _regIdx)
            : RegisterEvent(_rf, _regIdx) { }
        void process();
    };

    // Schedule an event to mark a register as free/busy in
    // the scoreboard/busy vector. Delay is already in Ticks
    virtual void enqRegFreeEvent(uint32_t regIdx, uint64_t delay);
    virtual void enqRegBusyEvent(uint32_t regIdx, uint64_t delay);

    // Schedule functions

    // The following functions are called by the SCH stage when attempting
    // to move a wave from the readyList to the schList.
    // canSchedule* checks if the RF is ready to provide operands for
    // the instruction, while schedule* requests the RF to begin reading
    // and writing of operands. Calling schedule* may only occur
    // immediately after canSchedule* was called and returned True
    virtual bool canScheduleReadOperands(Wavefront *w, GPUDynInstPtr ii);
    virtual bool canScheduleWriteOperands(Wavefront *w, GPUDynInstPtr ii);
    virtual void scheduleReadOperands(Wavefront *w, GPUDynInstPtr ii);
    virtual void scheduleWriteOperands(Wavefront *w, GPUDynInstPtr ii);

    // The following function is called to check if all operands
    // have been read for the given instruction
    virtual bool operandReadComplete(Wavefront *w, GPUDynInstPtr ii);

    // The following two functions are only called by returning loads to
    // check if the register file can support the incoming writes
    virtual bool canScheduleWriteOperandsFromLoad(Wavefront *w,
                                                  GPUDynInstPtr ii);
    // Queue the register writes. Assumes canScheduleWriteOperandsFromLoad
    // was called immediately prior and returned True
    virtual void scheduleWriteOperandsFromLoad(Wavefront *w,
                                               GPUDynInstPtr ii);

    // ExecRF is invoked every cycle by the compute unit and may be
    // used to model detailed timing of the register file.
    virtual void exec();

    // Called to inform RF that an instruction is executing
    // to schedule events for writeback, etc., as needed
    virtual void waveExecuteInst(Wavefront *w, GPUDynInstPtr ii);

    // Debug functions
    virtual std::string dump() const;

    virtual void dispatchInstruction(GPUDynInstPtr ii);

  protected:
    ComputeUnit* computeUnit;
    int simdId;

    // flag indicating if a register is busy
    std::vector<bool> busy;

    // numer of registers in this register file
    int _numRegs;

    struct RegisterFileStats : public statistics::Group
    {
        RegisterFileStats(statistics::Group *parent);

        // Total number of register reads per DWORD per thread
        statistics::Scalar registerReads;

        statistics::Scalar rfc_cache_read_hits;
        statistics::Scalar rfc_cache_write_hits;

        // Total number of register writes per DWORD per thread
        statistics::Scalar registerWrites;

        // Number of register file SRAM activations for reads.
        // The register file may be implemented with multiple SRAMs. This stat
        // tracks how many times the SRAMs are accessed for reads.
        statistics::Scalar sramReads;
        // Number of register file SRAM activations for writes
        statistics::Scalar sramWrites;
    } stats;
};

} // namespace gem5

#endif // __REGISTER_FILE_HH__
