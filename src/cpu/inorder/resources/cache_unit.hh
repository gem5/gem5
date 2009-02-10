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

//#include "cpu/inorder/params.hh"

#include "cpu/inorder/resource.hh"
#include "cpu/inorder/inorder_dyn_inst.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "mem/port.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "sim/sim_object.hh"

#include "params/InOrderCPU.hh"

class CacheRequest;
typedef CacheRequest* CacheReqPtr;

class CacheReqPacket;
typedef CacheReqPacket* CacheReqPktPtr;

class CacheUnit : public Resource {
  public:
    typedef ThePipeline::DynInstPtr DynInstPtr;

  public:
    CacheUnit(std::string res_name, int res_id, int res_width,
              int res_latency, InOrderCPU *_cpu, ThePipeline::Params *params);
    virtual ~CacheUnit() {}

    enum Command {
        InitiateFetch,
        CompleteFetch,
        InitiateReadData,
        CompleteReadData,
        InitiateWriteData,
        CompleteWriteData,
        Fetch,
        ReadData,
        WriteData
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
          : Port(_cachePortUnit->name() + "-cache-port", (MemObject*)_cachePortUnit->cpu),
              cachePortUnit(_cachePortUnit)
        { }

        bool snoopRangeSent;

      protected:
        /** Atomic version of receive.  Panics. */
        virtual Tick recvAtomic(PacketPtr pkt);

        /** Functional version of receive.  Panics. */
        virtual void recvFunctional(PacketPtr pkt);

        /** Receives status change.  Other than range changing, panics. */
        virtual void recvStatusChange(Status status);

        /** Returns the address ranges of this device. */
        virtual void getDeviceAddressRanges(AddrRangeList &resp,
                                            AddrRangeList &snoop)
        { resp.clear(); snoop.clear(); }

        /** Timing version of receive. Handles setting fetch to the
         * proper status to start fetching. */
        virtual bool recvTiming(PacketPtr pkt);

        /** Handles doing a retry of a failed fetch. */
        virtual void recvRetry();
    };

    enum CachePortStatus {
        cacheWaitResponse,
        cacheWaitRetry,
        cacheAccessComplete
    };

    ///virtual void init();

    virtual ResourceRequest* getRequest(DynInstPtr _inst, int stage_num,
                                        int res_idx, int slot_num,
                                        unsigned cmd);

    void requestAgain(DynInstPtr inst, bool &try_request);

    int getSlot(DynInstPtr inst);

    void freeSlot(int slot_num);

    /** Execute the function of this resource. The Default is action
     *  is to do nothing. More specific models will derive from this
     *  class and define their own execute function.
     */
    void execute(int slot_num);

    void squash(DynInstPtr inst, int stage_num,
                InstSeqNum squash_seq_num, unsigned tid);

    /** Processes cache completion event. */
    void processCacheCompletion(PacketPtr pkt);

    void recvRetry();

    /** Align a PC to the start of an I-cache block. */
    Addr cacheBlockAlignPC(Addr addr)
    {
        //addr = TheISA::realPCToFetchPC(addr);
        return (addr & ~(cacheBlkMask));
    }

    /** Returns a specific port. */
    Port *getPort(const std::string &if_name, int idx);

    /** Fetch on behalf of an instruction. Will check to see
     *  if instruction is actually in resource before
     *  trying to fetch.
     */
    //Fault doFetchAccess(DynInstPtr inst);

    /** Read/Write on behalf of an instruction.
     *  curResSlot needs to be a valid value in instruction.
     */
    Fault doDataAccess(DynInstPtr inst);

    uint64_t getMemData(Packet *packet);

  protected:
    /** Cache interface. */
    CachePort *cachePort;

    CachePortStatus cacheStatus;

    CacheReqPtr retryReq;

    PacketPtr retryPkt;

    int retrySlot;

    bool cacheBlocked;

    std::vector<Addr> addrList;

    std::map<Addr, InstSeqNum> addrMap;

  public:
    int cacheBlkSize;

    int cacheBlkMask;

    /** Align a PC to the start of the Cache block. */
    Addr cacheBlockAlign(Addr addr)
    {
        return (addr & ~(cacheBlkMask));
    }

    /** THINGS USED FOR FETCH */
    // NO LONGER USED BY COMMENT OUT UNTIL FULL VERIFICATION
    /** The mem line being fetched. */
    //uint8_t *cacheData[ThePipeline::MaxThreads];

    /** The Addr of the cacheline that has been loaded. */
    //Addr cacheBlockAddr[ThePipeline::MaxThreads];

    //unsigned fetchOffset[ThePipeline::MaxThreads];

    /** @todo: Add Resource Stats Here */
};

struct CacheSchedEntry : public ThePipeline::ScheduleEntry {
    enum EntryType {
        FetchAccess,
        DataAccess
    };

    CacheSchedEntry(int stage_num, int _priority, int res_num, MemCmd::Command pkt_cmd,
                    EntryType _type = FetchAccess) :
        ScheduleEntry(stage_num, _priority, res_num), pktCmd(pkt_cmd),
        type(_type)
    { }

    MemCmd::Command pktCmd;
    EntryType type;
};

class CacheRequest : public ResourceRequest {
  public:
    CacheRequest(CacheUnit *cres, DynInstPtr inst, int stage_num, int res_idx,
                 int slot_num, unsigned cmd, int req_size,
                 MemCmd::Command pkt_cmd, unsigned flags, int cpu_id)
        : ResourceRequest(cres, inst, stage_num, res_idx, slot_num, cmd),
          pktCmd(pkt_cmd), memAccComplete(false), memAccPending(false)
    {
        memReq = inst->memReq;

        reqData = new uint8_t[req_size];
        retryPkt = NULL;
    }

    virtual ~CacheRequest()
    {
        /*
        delete reqData;

        Can get rid of packet and packet request now
        if (*dataPkt) {
            if (*dataPkt->req) {
                delete dataPkt->req;
            }
            delete dataPkt;
        }

        // Can get rid of packet and packet request now
        if (retryPkt) {
            if (retryPkt->req) {
                delete retryPkt->req;
            }
            delete retryPkt;
            }*/

        if (memReq) {
            delete memReq;
        }
    }

    virtual PacketDataPtr getData()
    { 	return reqData; }

    void setMemAccCompleted(bool completed = true) { memAccComplete = completed; }
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
};

class CacheReqPacket : public Packet {
  public:
    CacheReqPacket(CacheRequest *_req,
                   Command _cmd, short _dest)
        : Packet(_req->memReq, _cmd, _dest), cacheReq(_req)
    {

    }

    CacheRequest *cacheReq;
};

#endif //__CPU_CACHE_UNIT_HH__
