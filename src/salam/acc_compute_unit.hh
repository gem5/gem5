/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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

#ifndef __SALAM_ACC_COMPUTE_UNIT_HH__
#define __SALAM_ACC_COMPUTE_UNIT_HH__

#include "params/AccComputeUnit.hh"
#include "salam/HWModeling/hw_interface.hh"
#include "salam/LLVMRead/debug_flags.hh"
#include "salam/LLVMRead/mem_request.hh"
#include "salam/comm_interface.hh"
#include "sim/sim_object.hh"

class AccComputeUnit : public SimObject
{
  private:
  protected:
    CommInterface *comm;
    HWInterface *hw;

    class TickEvent : public Event
    {
      private:
        AccComputeUnit *acc_comp_unit;

      public:
        TickEvent(AccComputeUnit *_acc_comp_unit)
            : Event(CPU_Tick_Pri), acc_comp_unit(_acc_comp_unit)
        {}
        void
        process()
        {
            acc_comp_unit->tick();
        }
        virtual const char *
        description() const
        {
            return "AccComputeUnit tick";
        }
    };

    TickEvent tickEvent;
    int clock_period;

  public:
    virtual void
    tick()
    {}
    AccComputeUnit(const AccComputeUnitParams &p);
    virtual void
    initialize()
    {}
    virtual void
    readCommit(MemoryRequest *req)
    {}
    virtual void
    writeCommit(MemoryRequest *req)
    {}
    CommInterface *
    getCommInterface()
    {
        return comm;
    }
    HWInterface *
    getHWInterface()
    {
        return hw;
    }
};

#endif //__SALAM_COMPUTE_UNIT_HH__
