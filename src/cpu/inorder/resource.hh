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

#ifndef __CPU_INORDER_RESOURCE_HH__
#define __CPU_INORDER_RESOURCE_HH__

#include <list>
#include <string>
#include <vector>

#include "base/types.hh"
#include "cpu/inorder/inorder_dyn_inst.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/inst_seq.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"

class Event;
class InOrderCPU;
class ResourceEvent;
class ResourceRequest;

typedef ResourceRequest ResReq;
typedef ResourceRequest* ResReqPtr;

class CacheRequest;
typedef CacheRequest* CacheReqPtr;

class Resource {
  public:
    typedef ThePipeline::DynInstPtr DynInstPtr;

    friend class ResourceEvent;
    friend class ResourceRequest;

  public:
    Resource(std::string res_name, int res_id, int res_width,
             Cycles res_latency, InOrderCPU *_cpu);
    virtual ~Resource();


    /** Return name of this resource */
    virtual std::string name();

    /** Return ID for this resource */
    int getId() { return id; }

    /** Any extra initiliazation stuff can be set up using this function that
     * should get called before the simulation starts (tick 0)
     */
    virtual void init();    
    virtual void initSlots();

    /** Register Stats for this resource */
    virtual void regStats() { }

    /** Resources that care about thread activation override this. */
    virtual void activateThread(ThreadID tid) { }

    /** Deactivate Thread. Default action is to squash all instructions
     *  from deactivated thread.
     */
    virtual void deactivateThread(ThreadID tid);

    /** Resources that care about thread activation override this. */
    virtual void suspendThread(ThreadID tid) { }
    
    /** Will be called the cycle before a context switch. Any bookkeeping
     *  that needs to be kept for that, can be done here
     */
    virtual void updateAfterContextSwitch(DynInstPtr inst, ThreadID tid) { }    

    /** Resources that care when an instruction has been graduated
     *  can override this
     */
    virtual void instGraduated(InstSeqNum seq_num, ThreadID tid) { }

    /** Post-processsing for Trap Generated from this instruction */
    virtual void trap(Fault fault, ThreadID tid, DynInstPtr inst) { }

    /** Request usage of this resource. Returns a ResourceRequest object
     *  with all the necessary resource information
     */
    virtual ResourceRequest* request(DynInstPtr inst);

    /** Get the next available slot in this resource. Instruction is passed
     *  so that resources can check the instruction before allocating a slot
     *  if necessary.
     */
    virtual int getSlot(DynInstPtr inst);

    /** Find the slot that this instruction is using in a resource */
    virtual int findSlot(DynInstPtr inst);

    /** Free a resource slot */
    virtual void freeSlot(int slot_idx);

    /** Request usage of a resource for this instruction. If this instruction 
     *  already has made this request to this resource, and that request is 
     *  uncompleted this function will just return that request
     */
    virtual ResourceRequest* getRequest(DynInstPtr _inst, int stage_num,
                                        int res_idx, int slot_num,
                                        unsigned cmd);

    /** Schedule Execution of This Resource For A Given Slot*/
    void scheduleExecution(int slot_idx);

    /** Execute the function of this resource. The Default is action
     *  is to do nothing. More specific models will derive from this
     *  class and define their own execute function.
     */
    virtual void execute(int slot_idx);

    /** Fetch on behalf of an instruction. Will check to see
     *  if instruction is actually in resource before
     *  trying to fetch. Needs to be defined for derived units.
     */
    virtual Fault doFetchAccess(DynInstPtr inst)
    { panic("doFetchAccess undefined for %s", name()); return NoFault; }

    /** Read/Write on behalf of an instruction. Will check to see
     *  if instruction is actually in resource before
     *  trying to do access.Needs to be defined for derived units.
     */
    virtual void doCacheAccess(DynInstPtr inst, uint64_t *write_result = NULL,
                               CacheReqPtr split_req = NULL)
    { panic("doCacheAccess undefined for %s", name()); }

    /** Setup Squash to be sent out to pipeline and resource pool */
    void setupSquash(DynInstPtr inst, int stage_num, ThreadID tid);

    /** Squash All Requests After This Seq Num */
    virtual void squash(DynInstPtr inst, int stage_num,
                        InstSeqNum squash_seq_num, ThreadID tid);

    /** Squash Requests Due to a Memory Stall (By Default, same as "squash" */
    virtual void squashDueToMemStall(DynInstPtr inst, int stage_num,
                                     InstSeqNum squash_seq_num, ThreadID tid);

    /** Handle Squash & Trap that occured from an instruction in a resource */
    void squashThenTrap(int stage_num, DynInstPtr inst);

    /** The number of instructions available that this resource can
     *  can still process
     */
    int slotsAvail();

    /** The number of instructions using this resource */
    int slotsInUse();

    /** Schedule resource event, regardless of its current state. */
    void scheduleEvent(int slot_idx, Cycles delay);

    /** Find instruction in list, Schedule resource event, regardless of its 
     *  current state. */
    bool scheduleEvent(DynInstPtr inst, Cycles delay);

    /** Unschedule resource event, regardless of its current state. */
    void unscheduleEvent(int slot_idx);

    /** Unschedule resource event, regardless of its current state. */
    bool unscheduleEvent(DynInstPtr inst);

    /** Find the request that corresponds to this instruction */
    virtual ResReqPtr findRequest(DynInstPtr inst);

    /** */
    void rejectRequest(DynInstPtr inst);

    /** Request a Resource again. Some resources have to special process this
     *  in subsequent accesses.
     */
    virtual void requestAgain(DynInstPtr inst, bool &try_request);

    /** Return Latency of Resource */
    /*  Can be overridden for complex cases */
    virtual Cycles getLatency(int slot_num) { return latency; }

  protected:
    /** The name of this resource */
    std::string resName;

    /** ID of the resource. The Resource Pool uses this # to identify this
     *  resource.
     */
    int id;

    /** The number of instructions the resource can simultaneously
     *  process.
     */
    int width;

    /** Constant latency for this resource.
     *  Note: Dynamic latency resources set this to 0 and
     *  manage the latency themselves
     */
    const Cycles latency;

  public:
    /** List of all Requests the Resource is Servicing. Each request
        represents part of the resource's bandwidth
    */
    std::vector<ResReqPtr> reqs;

    /** A list of all the available execution slots for this resource.
     *  This correlates with the actual resource event idx.
     */
    std::vector<int> availSlots;

    /** The CPU(s) that this resource interacts with */
    InOrderCPU *cpu;

  protected:
    /** The resource event used for scheduling resource slots on the
     *  event queue
     */
    ResourceEvent *resourceEvent;

    /** Default denied resource request pointer*/
    ResReqPtr deniedReq;
};

class ResourceEvent : public Event
{
  public:
    /** Pointer to the Resource this is an event for */
    Resource *resource;


    /// Resource events that come before other associated CPU events
    /// (for InOrderCPU model).
    /// check src/sim/eventq.hh for more event priorities.
    enum InOrderPriority {
        Resource_Event_Pri = 45
    };

    /** The Resource Slot that this event is servicing */
    int slotIdx;

    /** Constructs a resource event. */
    ResourceEvent();
    ResourceEvent(Resource *res, int slot_idx);
    virtual ~ResourceEvent() { }

    /** Initialize data for this resource event. */
    virtual void init(Resource *res, int slot_idx);

    /** Processes a resource event. */
    virtual void process();

    /** Returns the description of the resource event. */
    const char *description() const;

    /** Set slot idx for event */
    void setSlot(int slot) { slotIdx = slot; }

    /** Schedule resource event, regardless of its current state. */
    void scheduleEvent(Cycles delay);

    /** Unschedule resource event, regardless of its current state. */
    void unscheduleEvent()
    {
        if (scheduled())
            squash();
    }

};

class ResourceRequest
{
  public:
    typedef ThePipeline::DynInstPtr DynInstPtr;

    static int resReqID;

    static int maxReqCount;
    
    friend class Resource;

  public:
    ResourceRequest(Resource *_res);
    
    virtual ~ResourceRequest();

    std::string name();
    
    int reqID;

    void setRequest(DynInstPtr _inst, int stage_num,
                    int res_idx, int slot_num, unsigned _cmd);

    virtual void clearRequest();

    /** Acknowledge that this is a request is done and remove
     *  from resource.
     */
    void done(bool completed = true);
    
    void freeSlot();

    /////////////////////////////////////////////
    //
    // GET RESOURCE REQUEST IDENTIFICATION / INFO
    //
    /////////////////////////////////////////////
    /** Get Resource Index */
    int getResIdx() { return resIdx; }
       
    /** Get Slot Number */
    int getSlot() { return slotNum; }
    bool hasSlot()  { return slotNum >= 0; }     

    /** Get Stage Number */
    int getStageNum() { return stageNum; }

    /** Set/Get Thread Ids */
    void setTid(ThreadID _tid) { tid = _tid; }
    ThreadID getTid() { return tid; }

    /** Instruction this request is for */
    DynInstPtr getInst() { return inst; }

    /** Data from this request. Overridden by Resource-Specific Request
     *  Objects
     */
    virtual PacketDataPtr getData() { return NULL; }

    /** Pointer to Resource that is being used */
    Resource *res;

    /** Instruction being used */
    DynInstPtr inst;

    /** Not guaranteed to be set, used for debugging */
    InstSeqNum seqNum;
    
    /** Command For This Resource */
    unsigned cmd;

    short stagePasses;

    bool valid;

    bool doneInResource;

    ////////////////////////////////////////
    //
    // GET RESOURCE REQUEST STATUS FROM VARIABLES
    //
    ////////////////////////////////////////
    /** Get/Set Completed variables */
    bool isCompleted() { return completed; }
    void setCompleted(bool cond = true) { completed = cond; }

    /** Get/Set Squashed variables */
    bool isSquashed() { return squashed; }
    void setSquashed() { squashed = true; }

    /** Get/Set IsProcessing variables */
    bool isProcessing() { return processing; }
    void setProcessing(bool cond = true) { processing = cond; }

    /** Get/Set IsWaiting variables */
    bool isMemStall() { return memStall; }
    void setMemStall(bool stall = true) { memStall = stall; }

  protected:
    /** Resource Identification */
    ThreadID tid;
    int stageNum;
    int resIdx;
    int slotNum;
    
    /** Resource Request Status */
    bool completed;
    bool squashed;
    bool processing;

    bool memStall;
};

#endif //__CPU_INORDER_RESOURCE_HH__
