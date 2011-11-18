/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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
 */

#ifndef __CPU_INORDER_PARAMS_HH__
#define __CPU_INORDER_PARAMS_HH__

#include "cpu/base.hh"

//Forward declarations
class FunctionalMemory;
class Process;
class MemObject;
class MemInterface;

/**
 * This file defines the parameters that will be used for the InOrderCPU.
 * This must be defined externally so that the Impl can have a params class
 * defined that it can pass to all of the individual stages.
 */

class InOrderParams : public BaseCPU::Params
{
  public:

    // Workloads
    std::vector<Process *> workload;
    Process *process;

    //
    // Memory System/Caches
    //
    unsigned cachePorts;
    std::string fetchMemPort;
    std::string dataMemPort;

    //
    // Branch predictor (BP & BTB)
    //
    std::string predType;
    unsigned localPredictorSize;
    unsigned localCtrBits;
    unsigned localHistoryTableSize;
    unsigned localHistoryBits;
    unsigned globalPredictorSize;
    unsigned globalCtrBits;
    unsigned globalHistoryBits;
    unsigned choicePredictorSize;
    unsigned choiceCtrBits;
    unsigned BTBEntries;
    unsigned BTBTagSize;
    unsigned RASSize;

    // Pipeline Parameters
    unsigned stageWidth;

    // InOrderCPU Simulation Parameters
    unsigned instShiftAmt;
    unsigned activity;
    unsigned deferRegistration;

    //
    // Memory Parameters
    //
    unsigned memBlockSize;

    //
    // Multiply Divide Unit
    //
    // @NOTE: If >1 MDU is needed and each MDU is to use varying parametesr,
    // then MDU must be defined as its own SimObject so that an arbitrary # can
    // be defined with different parameters
    /** Latency & Repeat Rate for Multiply Insts */
    unsigned multLatency;
    unsigned multRepeatRate;

    /** Latency & Repeat Rate for 8-bit Divide Insts */
    unsigned div8Latency;
    unsigned div8RepeatRate;

    /** Latency & Repeat Rate for 16-bit Divide Insts */
    unsigned div16Latency;
    unsigned div16RepeatRate;

    /** Latency & Repeat Rate for 24-bit Divide Insts */
    unsigned div24Latency;
    unsigned div24RepeatRate;

    /** Latency & Repeat Rate for 32-bit Divide Insts */
    unsigned div32Latency;
    unsigned div32RepeatRate;


};

#endif // _CPU_INORDER_PARAMS_HH__
