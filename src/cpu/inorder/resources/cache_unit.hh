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

#ifndef __CPU_INORDER_CACHE_UNIT_HH__
#define __CPU_INORDER_CACHE_UNIT_HH__

#include <vector>
#include <list>
#include <string>

#include "arch/predecoder.hh"
#include "arch/tlb.hh"
#include "config/the_isa.hh"
#include "cpu/inorder/inorder_dyn_inst.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/inorder/resource.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "mem/port.hh"
#include "params/InOrderCPU.hh"
#include "sim/sim_object.hh"

class CacheRequest;
typedef CacheRequest* CacheReqPtr;

class CacheReqPacket;
typedef CacheReqPacket* CacheReqPktPtr;

class CacheUnit : public Resource
{
  public:
    typedef ThePipeline::DynInstPtr DynInstPtr;

  public:
    CacheUnit(std::string res_name, int res_id, int res_width,
              int res_latency, InOrderCPU *_cpu, ThePipeline::Params *params);

    enum Command {
        InitiateFetch,
        CompleteFetch,
        InitiateReadData,
        CompleteReadData,
        InitiateWriteData,
        CompleteWriteData,
        Fetch,
        ReadData,
        WriteData,
        InitSecondSplitRead,
        InitSecondSplitWrite,
        CompleteSecondSplitRead,
        CompleteSecondSplitWrite
    };

  public:
    /** CachePort class for the Cache Unit.  Handles doing the
     * communication with the cache/memory.
     */
    class CachePort : public Port
    {
      protected:
        /** Pointer to cache port unit */
        CacheUnit *cachePortUnit;

      public:
        /** Default constructor. */
        CachePort(CacheUnit *_cachePortUnit)
          : Port(_cachePortUnit->name() + "-cache-port",
                 (MemObject*)_cachePortUnit->cpu),
            cachePortUnit(_cachePortUnit)
        { }

        bool snoopRangeSent;

      protected:
        /** Atomic version of receive.  Panics. */
        Tick recvAtomic(PacketPtr pkt);

        /** Functional version of receive.  Panics. */
        void recvFunctional(PacketPtr pkt);

        /** Receives status change.  Other than range changing, panics. */
        void recvStatusChange(Status status);

        /** Returns the address ranges of this device. */
        void getDeviceAddressRanges(AddrRangeList &resp,
                                            AddrRangeList &snoop)
        { resp.clear(); snoop.clear(); }

        /** Timing version of receive. Handles setting fetch to the
         * proper status to start fetching. */
        bool recvTiming(PacketPtr pkt);

        /** Handles doing a retry of a failed fetch. */
        void recvRetry();
    };

    void init();

    ResourceRequest* getRequest(DynInstPtr _inst, int stage_num,
                                        int res_idx, int slot_num,
                                        unsigned cmd);

    ResReqPtr findRequest(DynInstPtr inst);
    ResReqPtr findSplitRequest(DynInstPtr inst, int idx);

    void requestAgain(DynInstPtr inst, bool &try_request);

    int getSlot(DynInstPtr inst);

    /** Execute the function of this resource. The Default is action
     *  is to do nothing. More specific models will derive from this
     *  class and define their own execute function.
     */
    void execute(int slot_num);

    void squash(DynInstPtr inst, int stage_num,
                InstSeqNum squash_seq_num, ThreadID tid);

    void squashDueToMemStall(DynInstPtr inst, int stage_num,
                             InstSeqNum squash_seq_num, ThreadID tid);

    /** Processes cache completion event. */
    void processCacheCompletion(PacketPtr pkt);

    void recvRetry();

    /** Align a PC to the start of an I-cache block. */
    Addr cacheBlockAlignPC(Addr addr)
    {
        return (addr & ~(cacheBlkMask));
    }

    /** Returns a specific port. */
    Port *getPort(const std::string &if_name, int idx);
    
    Fault read(DynInstPtr inst, Addr addr,
               uint8_t *data, unsigned size, unsigned flags);

    Fault write(DynInstPtr inst, uint8_t *data, unsigned size,
                Addr addr, unsigned flags, uint64_t *res);

    Fault doTLBAccess(DynInstPtr inst, CacheReqPtr cache_req, int acc_size,
                      int flags,  TheISA::TLB::Mode tlb_mode);

    /** Read/Write on behalf of an instruction.
     *  curResSlot needs to be a valid value in instruction.
     */
    Fault doCacheAccess(DynInstPtr inst, uint64_t *write_result=NULL,
                        CacheReqPtr split_req=NULL);

    void prefetch(DynInstPtr inst);

    void writeHint(DynInstPtr inst);

    uint64_t getMemData(Packet *packet);

    void setAddrDependency(DynInstPtr inst);
    void removeAddrDependency(DynInstPtr inst);
    
  protected:
    /** Cache interface. */
    CachePort *cachePort;

    bool cachePortBlocked;

    std::vector<Addr> addrList[ThePipeline::MaxThreads];

    std::map<Addr, InstSeqNum> addrMap[ThePipeline::MaxThreads];

  public:
    int cacheBlkSize;

    int cacheBlkMask;

    /** Align a PC to the start of the Cache block. */
    Addr cacheBlockAlign(Addr addr)
    {
        return (addr & ~(cacheBlkMask));
    }

    /** The mem line being fetched. */
    uint8_t *fetchData[ThePipeline::MaxThreads];

    /** @TODO: Move functionaly of fetching more than
        one instruction to 'fetch unit'*/
    /** The Addr of the cacheline that has been loaded. */
    //Addr cacheBlockAddr[ThePipeline::MaxThreads];
    //unsigned fetchOffset[ThePipeline::MaxThreads];

    TheISA::Predecoder predecoder;

    bool tlbBlocked[ThePipeline::MaxThreads];

    TheISA::TLB* tlb();

    TheISA::TLB *_tlb;
};

class CacheUnitEvent : public ResourceEvent {
  public:
    const std::string name() const
    {
        return "CacheUnitEvent";
    }


    /** Constructs a resource event. */
    CacheUnitEvent();
    virtual ~CacheUnitEvent() {}

    /** Processes a resource event. */
    void process();
};

class CacheRequest : public ResourceRequest
{
  public:
    CacheRequest(CacheUnit *cres, DynInstPtr inst, int stage_num, int res_idx,
                 int slot_num, unsigned cmd, int req_size,
                 MemCmd::Command pkt_cmd, unsigned flags, int cpu_id, int idx)
        : ResourceRequest(cres, inst, stage_num, res_idx, slot_num, cmd),
          pktCmd(pkt_cmd), memReq(NULL), reqData(NULL), dataPkt(NULL),
          retryPkt(NULL), memAccComplete(false), memAccPending(false),
          tlbStall(false), splitAccess(false), splitAccessNum(-1),
          split2ndAccess(false), instIdx(idx)
    { }


    virtual ~CacheRequest()
    {
        if (reqData && !splitAccess) {
            delete [] reqData;
        }
    }

    virtual PacketDataPtr getData()
    { 	return reqData; }

    void
    setMemAccCompleted(bool completed = true)
    {
        memAccComplete = completed;
    }

    bool is2ndSplit() 
    {
        return split2ndAccess;
    }
    
    bool isMemAccComplete() { return memAccComplete; }

    void setMemAccPending(bool pending = true) { memAccPending = pending; }
    bool isMemAccPending() { return memAccPending; }

    //Make this data private/protected!
    MemCmd::Command pktCmd;
    RequestPtr memReq;
    PacketDataPtr reqData;
    PacketPtr dataPkt;
    PacketPtr retryPkt;

    bool memAccComplete;
    bool memAccPending;
    bool tlbStall;

    bool splitAccess;
    int splitAccessNum;
    bool split2ndAccess;
    int instIdx;    
    
};

class CacheReqPacket : public Packet
{
  public:
    CacheReqPacket(CacheRequest *_req,
                   Command _cmd, short _dest, int _idx = 0)
        : Packet(_req->memReq, _cmd, _dest), cacheReq(_req), instIdx(_idx)
    {

    }

    CacheRequest *cacheReq;
    int instIdx;
    
};

#endif //__CPU_CACHE_UNIT_HH__
