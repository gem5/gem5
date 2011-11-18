/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 */

#ifndef __CPU_OZONE_INORDER_BACK_END_HH__
#define __CPU_OZONE_INORDER_BACK_END_HH__

#include <list>

#include "cpu/ozone/rename_table.hh"
#include "cpu/ozone/thread_state.hh"
#include "cpu/inst_seq.hh"
#include "cpu/thread_context.hh"
#include "cpu/timebuf.hh"
#include "mem/request.hh"
#include "sim/eventq.hh"
#include "sim/faults.hh"

template <class Impl>
class InorderBackEnd
{
  public:
    typedef typename Impl::Params Params;
    typedef typename Impl::DynInstPtr DynInstPtr;
    typedef typename Impl::FullCPU FullCPU;
    typedef typename Impl::FrontEnd FrontEnd;

    typedef typename FullCPU::OzoneTC OzoneTC;
    typedef typename Impl::FullCPU::CommStruct CommStruct;

    InorderBackEnd(Params *params);

    std::string name() const;

    void setCPU(FullCPU *cpu_ptr)
    { cpu = cpu_ptr; }

    void setFrontEnd(FrontEnd *front_end_ptr)
    { frontEnd = front_end_ptr; }

    void setCommBuffer(TimeBuffer<CommStruct> *_comm)
    { comm = _comm; }

    void setTC(ThreadContext *tc_ptr);

    void setThreadState(OzoneThreadState<Impl> *thread_ptr);

    void regStats() { }

    void checkInterrupts();

    void tick();
    void executeInsts();
    void squash(const InstSeqNum &squash_num, const Addr &next_PC);

    void squashFromXC();
    void generateXCEvent() { }

    bool robEmpty() { return instList.empty(); }

    bool isFull() { return false; }
    bool isBlocked() { return status == DcacheMissStoreStall ||
                           status == DcacheMissLoadStall ||
                           interruptBlocked; }

    void fetchFault(Fault &fault);

    void dumpInsts();

  private:
    void handleFault();

    void setSquashInfoFromTC();

    bool squashPending;
    InstSeqNum squashSeqNum;
    Addr squashNextPC;

    Fault faultFromFetch;

    bool interruptBlocked;

  public:
    template <class T>
    Fault read(Addr addr, T &data, unsigned flags);

    template <class T>
    Fault read(RequestPtr req, T &data, int load_idx);

    template <class T>
    Fault write(T data, Addr addr, unsigned flags, uint64_t *res);

    template <class T>
    Fault write(RequestPtr req, T &data, int store_idx);

    Addr readCommitPC() { return commitPC; }

    Addr commitPC;

    void switchOut() { panic("Not implemented!"); }
    void doSwitchOut() { panic("Not implemented!"); }
    void takeOverFrom(ThreadContext *old_tc = NULL) { panic("Not implemented!"); }

  public:
    FullCPU *cpu;

    FrontEnd *frontEnd;

    ThreadContext *tc;

    OzoneThreadState<Impl> *thread;

    RenameTable<Impl> renameTable;

  protected:
    enum Status {
        Running,
        Idle,
        DcacheMissLoadStall,
        DcacheMissStoreStall,
        DcacheMissComplete,
        Blocked
    };

    Status status;

    class DCacheCompletionEvent : public Event
    {
      private:
        InorderBackEnd *be;

      public:
        DCacheCompletionEvent(InorderBackEnd *_be);

        virtual void process();
        virtual const char *description() const;

        DynInstPtr inst;
    };

    friend class DCacheCompletionEvent;

    DCacheCompletionEvent cacheCompletionEvent;

//    MemInterface *dcacheInterface;

    RequestPtr memReq;

  private:
    typedef typename std::list<DynInstPtr>::iterator InstListIt;

    std::list<DynInstPtr> instList;

    // General back end width. Used if the more specific isn't given.
    int width;

    int latency;

    int squashLatency;

    TimeBuffer<int> numInstsToWB;
    TimeBuffer<int>::wire instsAdded;
    TimeBuffer<int>::wire instsToExecute;

    TimeBuffer<CommStruct> *comm;
    // number of cycles stalled for D-cache misses
    Stats::Scalar dcacheStallCycles;
    Counter lastDcacheStall;
};

template <class Impl>
template <class T>
Fault
InorderBackEnd<Impl>::read(Addr addr, T &data, unsigned flags)
{
    memReq->reset(addr, sizeof(T), flags);

    // translate to physical address
    Fault fault = cpu->dtb->translateAtomic(memReq, thread->getTC(), false);

    // if we have a cache, do cache access too
    if (fault == NoFault && dcacheInterface) {
        memReq->cmd = Read;
        memReq->completionEvent = NULL;
        memReq->time = curTick();
        MemAccessResult result = dcacheInterface->access(memReq);

        // Ugly hack to get an event scheduled *only* if the access is
        // a miss.  We really should add first-class support for this
        // at some point.
        if (result != MA_HIT) {
            // Fix this hack for keeping funcExeInst correct with loads that
            // are executed twice.
            memReq->completionEvent = &cacheCompletionEvent;
            lastDcacheStall = curTick();
//          unscheduleTickEvent();
            status = DcacheMissLoadStall;
            DPRINTF(IBE, "Dcache miss stall!\n");
        } else {
            // do functional access
            DPRINTF(IBE, "Dcache hit!\n");
        }
    }
    return fault;
}

template <class Impl>
template <class T>
Fault
InorderBackEnd<Impl>::write(T data, Addr addr, unsigned flags, uint64_t *res)
{
    memReq->reset(addr, sizeof(T), flags);

    // translate to physical address
    Fault fault = cpu->dtb->translateAtomic(memReq, thread->getTC(), true);

    if (fault == NoFault && dcacheInterface) {
        memReq->cmd = Write;
//      memcpy(memReq->data,(uint8_t *)&data,memReq->size);
        memReq->completionEvent = NULL;
        memReq->time = curTick();
        MemAccessResult result = dcacheInterface->access(memReq);

        // Ugly hack to get an event scheduled *only* if the access is
        // a miss.  We really should add first-class support for this
        // at some point.
        if (result != MA_HIT) {
            memReq->completionEvent = &cacheCompletionEvent;
            lastDcacheStall = curTick();
//          unscheduleTickEvent();
            status = DcacheMissStoreStall;
            DPRINTF(IBE, "Dcache miss stall!\n");
        } else {
            DPRINTF(IBE, "Dcache hit!\n");
        }
    }

    if (res && (fault == NoFault))
        *res = memReq->result;
    return fault;
}

template <class Impl>
template <class T>
Fault
InorderBackEnd<Impl>::read(MemReqPtr &req, T &data, int load_idx)
{
//    panic("Unimplemented!");
//    memReq->reset(addr, sizeof(T), flags);

    // translate to physical address
//    Fault fault = cpu->translateDataReadReq(req);
    req->cmd = Read;
    req->completionEvent = NULL;
    req->time = curTick();
    assert(!req->data);
    req->data = new uint8_t[64];
    Fault fault = cpu->read(req, data);
    memcpy(req->data, &data, sizeof(T));

    // if we have a cache, do cache access too
    if (dcacheInterface) {
        MemAccessResult result = dcacheInterface->access(req);

        // Ugly hack to get an event scheduled *only* if the access is
        // a miss.  We really should add first-class support for this
        // at some point.
        if (result != MA_HIT) {
            req->completionEvent = &cacheCompletionEvent;
            lastDcacheStall = curTick();
//          unscheduleTickEvent();
            status = DcacheMissLoadStall;
            DPRINTF(IBE, "Dcache miss load stall!\n");
        } else {
            DPRINTF(IBE, "Dcache hit!\n");

        }
    }

    return NoFault;
}

template <class Impl>
template <class T>
Fault
InorderBackEnd<Impl>::write(MemReqPtr &req, T &data, int store_idx)
{
//    req->reset(addr, sizeof(T), flags);

    // translate to physical address
//    Fault fault = cpu->translateDataWriteReq(req);

    req->cmd = Write;
    req->completionEvent = NULL;
    req->time = curTick();
    assert(!req->data);
    req->data = new uint8_t[64];
    memcpy(req->data, (uint8_t *)&data, req->size);

    switch(req->size) {
      case 1:
        cpu->write(req, (uint8_t &)data);
        break;
      case 2:
        cpu->write(req, (uint16_t &)data);
        break;
      case 4:
        cpu->write(req, (uint32_t &)data);
        break;
      case 8:
        cpu->write(req, (uint64_t &)data);
        break;
      default:
        panic("Unexpected store size!\n");
    }

    if (dcacheInterface) {
        req->cmd = Write;
        req->data = new uint8_t[64];
        memcpy(req->data,(uint8_t *)&data,req->size);
        req->completionEvent = NULL;
        req->time = curTick();
        MemAccessResult result = dcacheInterface->access(req);

        // Ugly hack to get an event scheduled *only* if the access is
        // a miss.  We really should add first-class support for this
        // at some point.
        if (result != MA_HIT) {
            req->completionEvent = &cacheCompletionEvent;
            lastDcacheStall = curTick();
//          unscheduleTickEvent();
            status = DcacheMissStoreStall;
            DPRINTF(IBE, "Dcache miss store stall!\n");
        } else {
            DPRINTF(IBE, "Dcache hit!\n");

        }
    }
/*
    if (req->isLLSC()) {
        if (req->isUncacheable()) {
            // Don't update result register (see stq_c in isa_desc)
            req->result = 2;
        } else {
            req->result = 1;
        }
    }
*/
/*
    if (res && (fault == NoFault))
        *res = req->result;
        */
    return NoFault;
}

#endif // __CPU_OZONE_INORDER_BACK_END_HH__
