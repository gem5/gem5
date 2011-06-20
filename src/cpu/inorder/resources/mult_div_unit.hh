/*
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
 * Authors: Korey Sewell
 *
 */

#ifndef __CPU_INORDER_MULT_DIV_UNIT_HH__
#define __CPU_INORDER_MULT_DIV_UNIT_HH__

#include <list>
#include <string>
#include <vector>

#include "cpu/inorder/first_stage.hh"
#include "cpu/inorder/inorder_dyn_inst.hh"
#include "cpu/inorder/resource.hh"
#include "cpu/func_unit.hh"
#include "cpu/op_class.hh"

class MDUEvent;

class MultDivUnit : public Resource {
  public:
    typedef ThePipeline::DynInstPtr DynInstPtr;

    enum Command {
        StartMultDiv,
        EndMultDiv,
        MultDiv
    };

  public:
    MultDivUnit(std::string res_name, int res_id, int res_width,
                int res_latency, InOrderCPU *_cpu,
                ThePipeline::Params *params);

  public:
    /** Override default Resource getSlot(). Will only getSlot if
     *  valid mult/div sequence is being maintained
     */
    int getSlot(DynInstPtr inst);
    
    void init();
    
    /** Get Operand Size For A Division Operation */
    int getDivOpSize(DynInstPtr inst);

    /** Override default Resource execute */
    void execute(int slot_num);

    void exeMulDiv(int slot_num);    
    
    /** Register extra resource stats */
    void regStats();

    void requestAgain(DynInstPtr inst, bool &try_request);

    void squash(DynInstPtr inst, int stage_num, InstSeqNum squash_seq_num,
                ThreadID tid);

  protected:
    /** Latency & Repeat Rate for Multiply Insts */
    unsigned multRepeatRate;
    unsigned multLatency;

    /** Latency & Repeat Rate for 8-bit Divide Insts */
    unsigned div8RepeatRate;
    unsigned div8Latency;

    /** Latency & Repeat Rate for 16-bit Divide Insts */
    unsigned div16RepeatRate;
    unsigned div16Latency;

    /** Latency & Repeat Rate for 24-bit Divide Insts */
    unsigned div24RepeatRate;
    unsigned div24Latency;

    /** Latency & Repeat Rate for 32-bit Divide Insts */
    unsigned div32RepeatRate;
    unsigned div32Latency;

    /** Last cycle that MDU was used */
    Tick lastMDUCycle;

    /** Last type of instruction MDU started processing */
    OpClass lastOpType;

    /** Last Division Operand of instruction MDU was processing */
    uint32_t lastDivSize;

    /** Last instruction name the MDU used */
    std::string lastInstName;

    /** Number of Multiplies */
    Stats::Scalar multiplies;

    /** Number of Divides */
    Stats::Scalar divides;

    MDUEvent *mduEvent;    
};

class MDUEvent : public ResourceEvent
{
  public:
    MDUEvent();
    ~MDUEvent() { }
    

    void process();
};


#endif //__CPU_INORDER_MULT_DIV_UNIT_HH__
