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

#include <list>
#include <string>
#include <vector>

#include "arch/tlb.hh"
#include "base/hashmap.hh"
#include "config/the_isa.hh"
#include "cpu/inorder/inorder_dyn_inst.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/inorder/resource.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "mem/port.hh"
#include "params/InOrderCPU.hh"
#include "sim/sim_object.hh"

class CacheReqPacket;
typedef CacheReqPacket* CacheReqPktPtr;

class CacheUnit : public Resource
{
  public:
    typedef ThePipeline::DynInstPtr DynInstPtr;

  public:
    CacheUnit(std::string res_name, int res_id, int res_width,
              Cycles res_latency, InOrderCPU *_cpu,
              ThePipeline::Params *params);

    enum Command {
        InitiateReadData,
        CompleteReadData,
        InitiateWriteData,
        CompleteWriteData,
        InitSecondSplitRead,
        InitSecondSplitWrite,
        CompleteSecondSplitRead,
        CompleteSecondSplitWrite
    };

  public:

    void init();

    ResourceRequest* getRequest(DynInstPtr _inst, int stage_num,
                                int res_idx, int slot_num,
                                unsigned cmd);

    ResReqPtr findRequest(DynInstPtr inst);
    ResReqPtr findRequest(DynInstPtr inst, int idx);

    void requestAgain(DynInstPtr inst, bool &try_request);

    virtual int getSlot(DynInstPtr inst);

    /** Executes one of the commands from the "Command" enum */
    virtual void execute(int slot_num);

    virtual void squash(DynInstPtr inst, int stage_num,
                InstSeqNum squash_seq_num, ThreadID tid);

    void squashDueToMemStall(DynInstPtr inst, int stage_num,
                             InstSeqNum squash_seq_num, ThreadID tid);

    virtual void squashCacheRequest(CacheReqPtr req_ptr);

    /** After memory request is completedd in the cache, then do final
        processing to complete the request in the CPU.
    */
    virtual void processCacheCompletion(PacketPtr pkt);

    /** Create request that will interface w/TLB and Memory objects */
    virtual void setupMemRequest(DynInstPtr inst, CacheReqPtr cache_req,
                                 int acc_size, int flags);

    void finishCacheUnitReq(DynInstPtr inst, CacheRequest *cache_req);

    void buildDataPacket(CacheRequest *cache_req);

    bool processSquash(CacheReqPacket *cache_pkt);

    void trap(Fault fault, ThreadID tid, DynInstPtr inst);

    void recvRetry();
    
    Fault read(DynInstPtr inst, Addr addr,
               uint8_t *data, unsigned size, unsigned flags);

    Fault write(DynInstPtr inst, uint8_t *data, unsigned size,
                Addr addr, unsigned flags, uint64_t *res);

    void doTLBAccess(DynInstPtr inst, CacheReqPtr cache_req, int acc_size,
                      int flags,  TheISA::TLB::Mode tlb_mode);

    /** Read/Write on behalf of an instruction.
     *  curResSlot needs to be a valid value in instruction.
     */
    void doCacheAccess(DynInstPtr inst, uint64_t *write_result=NULL,
                        CacheReqPtr split_req=NULL);

    uint64_t getMemData(Packet *packet);

    void setAddrDependency(DynInstPtr inst);
    virtual void removeAddrDependency(DynInstPtr inst);
    
  protected:
    /** Cache interface. */
    MasterPort *cachePort;

    bool cachePortBlocked;

    std::list<Addr> addrList[ThePipeline::MaxThreads];

    m5::hash_map<Addr, InstSeqNum> addrMap[ThePipeline::MaxThreads];

  public:
    int cacheBlkSize;

    int cacheBlkMask;

    /** Align a PC to the start of the Cache block. */
    Addr cacheBlockAlign(Addr addr)
    {
        return (addr & ~(cacheBlkMask));
    }

    bool tlbBlocked[ThePipeline::MaxThreads];
    InstSeqNum tlbBlockSeqNum[ThePipeline::MaxThreads];

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

//@todo: Move into CacheUnit Class for private access to "valid" field
class CacheRequest : public ResourceRequest
{
  public:
    CacheRequest(CacheUnit *cres)
        :  ResourceRequest(cres), memReq(NULL), reqData(NULL),
           dataPkt(NULL), memAccComplete(false),
           memAccPending(false), tlbStall(false), splitAccess(false),
           splitAccessNum(-1), split2ndAccess(false),
           fetchBufferFill(false)
    { }

    virtual ~CacheRequest()
    {
        if (reqData && !splitAccess)
            delete [] reqData;
    }

    void setRequest(DynInstPtr _inst, int stage_num, int res_idx, int slot_num,
                    unsigned _cmd, MemCmd::Command pkt_cmd, int idx)
    {
        pktCmd = pkt_cmd;
        instIdx = idx;

        ResourceRequest::setRequest(_inst, stage_num, res_idx, slot_num, _cmd);
    }

    void clearRequest();

    virtual PacketDataPtr getData()
    { return reqData; }

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
    CacheReqPacket *dataPkt;

    bool memAccComplete;
    bool memAccPending;
    bool tlbStall;

    bool splitAccess;
    int splitAccessNum;
    bool split2ndAccess;
    int instIdx;    

    /** Should we expect block from cache access or fetch buffer? */
    bool fetchBufferFill;
};

class CacheReqPacket : public Packet
{
  public:
    CacheReqPacket(CacheRequest *_req,
                   Command _cmd, int _idx = 0)
        : Packet(&(*_req->memReq), _cmd), cacheReq(_req),
          instIdx(_idx), hasSlot(false), reqData(NULL), memReq(NULL)
    {

    }

    CacheRequest *cacheReq;
    int instIdx;
    bool hasSlot;
    PacketDataPtr reqData;
    RequestPtr memReq;
};

#endif //__CPU_CACHE_UNIT_HH__
