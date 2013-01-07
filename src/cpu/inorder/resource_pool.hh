/*
 * Copyright (c) 2012 ARM Limited
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

#ifndef __CPU_INORDER_RESOURCE_POOL_HH__
#define __CPU_INORDER_RESOURCE_POOL_HH__

#include <string>
#include <vector>

#include "cpu/inorder/cpu.hh"
#include "cpu/inorder/inorder_dyn_inst.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/inorder/resource.hh"
#include "cpu/inst_seq.hh"
#include "params/InOrderCPU.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"

class CacheUnit;
class Event;
class FetchUnit;
class ResourceEvent;

class ResourcePool {
  public:
    typedef InOrderDynInst::DynInstPtr DynInstPtr;

  public:
    // List of Resource Pool Events that extends
    // the list started by the CPU
    // NOTE(1): Resource Pool also uses event list
    // CPUEventType defined in inorder/cpu.hh
    enum ResPoolEventType {
        InstGraduated = InOrderCPU::NumCPUEvents,
        SquashAll,
        UpdateAfterContextSwitch,
        Default
    };

    enum ResPoolEventPri {
        ResPool_Pri =  InOrderCPU::InOrderCPU_Pri - 5,
        ResGrad_Pri,
        ResSquash_Pri
    };

    class ResPoolEvent : public Event
    {
      protected:
        /** Resource Pool */
        ResourcePool *resPool;

      public:
        InOrderCPU::CPUEventType eventType;

        DynInstPtr inst;

        InstSeqNum seqNum;

        int stageNum;

        ThreadID tid;

      public:
        /** Constructs a resource event. */
        ResPoolEvent(ResourcePool *_resPool,
                     InOrderCPU::CPUEventType e_type,
                     DynInstPtr _inst,
                     int stage_num,
                     InstSeqNum seq_num,
                     ThreadID _tid,
                     ResPoolEventPri res_pri = ResPool_Pri);

        /** Set Type of Event To Be Scheduled */
        void setEvent(InOrderCPU::CPUEventType e_type,
                      DynInstPtr _inst,
                      int stage_num,
                      InstSeqNum seq_num,
                      ThreadID _tid)
        {
            eventType = e_type;
            inst = _inst;
            seqNum = seq_num;
            stageNum = stage_num;
            tid = _tid;
        }

        /** Processes a resource event. */
        void process();

        /** Returns the description of the resource event. */
        const char *description() const;

        /** Schedule Event */
        void scheduleEvent(Cycles delay);

        /** Unschedule This Event */
        void unscheduleEvent();
    };

  public:
    ResourcePool(InOrderCPU *_cpu, ThePipeline::Params *params);
    virtual ~ResourcePool();

    std::string name();

    std::string name(int res_idx) { return resources[res_idx]->name(); }

    void init();

    void print();

    /** Register Statistics in All Resources */
    void regStats();

    /** Returns a specific resource. */
    unsigned getResIdx(const ThePipeline::ResourceId &res_id);

    /** Returns a pointer to a resource */
    Resource* getResource(int res_idx) { return resources[res_idx]; }

    /** Request usage of this resource. Returns -1 if not granted and
     *  a positive request tag if granted.
     */
    ResReqPtr request(int res_idx, DynInstPtr inst);

    /** Squash The Resource */
    void squash(DynInstPtr inst, int res_idx, InstSeqNum done_seq_num,
                ThreadID tid);

    /** Squash All Resources in Pool after Done Seq. Num */
    void squashAll(DynInstPtr inst, int stage_num,
                   InstSeqNum done_seq_num, ThreadID tid);

    /** Squash Resources in Pool after a memory stall 
     *  NOTE: Only use during Switch-On-Miss Thread model
     */    
    void squashDueToMemStall(DynInstPtr inst, int stage_num,
                             InstSeqNum done_seq_num, ThreadID tid);

    /** Activate Thread in all resources */
    void activateThread(ThreadID tid);

    /** De-Activate Thread in all resources */
    void deactivateThread(ThreadID tid);

    /** Suspend Thread in all resources */
    void suspendThread(ThreadID tid);

    /** Broadcast Context Switch Update to all resources */
    void updateAfterContextSwitch(DynInstPtr inst, ThreadID tid);

    /** Broadcast graduation to all resources */
    void instGraduated(InstSeqNum seq_num, ThreadID tid);

    /** Broadcast trap to all resources */
    void trap(Fault fault, ThreadID tid, DynInstPtr inst);

    /** The number of instructions available that a resource can
     *  can still process.
     */
    int slotsAvail(int res_idx);

    /** The number of instructions using a resource */
    int slotsInUse(int res_idx);

    /** Schedule resource event, regardless of its current state. */
    void scheduleEvent(InOrderCPU::CPUEventType e_type, DynInstPtr inst = NULL,
                       Cycles delay = Cycles(0), int res_idx = 0,
                       ThreadID tid = 0);

   /** UnSchedule resource event, regardless of its current state. */
    void unscheduleEvent(int res_idx, DynInstPtr inst);

    /** Tasks to perform when simulation starts */
    virtual void startup() { }

    /** The CPU(s) that this resource interacts with */
    InOrderCPU *cpu;

    DynInstPtr dummyInst[ThePipeline::MaxThreads];

    /**
     * Get a pointer to the (always present) instruction fetch unit.
     *
     * @return the instruction unit
     */
    FetchUnit *getInstUnit() const { return instUnit; }

    /**
     * Get a pointer to the (always present) data load/store unit.
     *
     * @return the data cache unit
     */
    CacheUnit *getDataUnit() const { return dataUnit; }

  private:

    /** The instruction fetch unit. */
    FetchUnit *instUnit;

    /** The data load/store unit. */
    CacheUnit *dataUnit;

    std::vector<Resource *> resources;

    /** Resources that need to be updated on an inst. graduation */
    std::vector<int> gradObjects;
};

#endif //__CPU_INORDER_RESOURCE_HH__
