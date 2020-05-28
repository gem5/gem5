/*
 * Copyright (c) 2015-2018 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
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
 *
 * Authors: Eric van Tassell
 */

#ifndef __DEV_HSA_HSA_PACKET_PROCESSOR__
#define __DEV_HSA_HSA_PACKET_PROCESSOR__

#include <cstdint>

#include <queue>

#include "dev/dma_device.hh"
#include "dev/hsa/hsa.h"
#include "dev/hsa/hsa_queue.hh"
#include "params/HSAPacketProcessor.hh"

#define AQL_PACKET_SIZE 64
#define PAGE_SIZE 4096
#define NUM_DMA_BUFS 16
#define DMA_BUF_SIZE (AQL_PACKET_SIZE * NUM_DMA_BUFS)
// HSA runtime supports only 5 signals per barrier packet
#define NumSignalsPerBarrier 5

class HSADevice;
class HWScheduler;

// Our internal representation of an HSA queue
class HSAQueueDescriptor {
    public:
        uint64_t     basePointer;
        uint64_t     doorbellPointer;
        uint64_t     writeIndex;
        uint64_t     readIndex;
        uint32_t     numElts;
        uint64_t     hostReadIndexPtr;
        bool         stalledOnDmaBufAvailability;
        bool         dmaInProgress;

        HSAQueueDescriptor(uint64_t base_ptr, uint64_t db_ptr,
                           uint64_t hri_ptr, uint32_t size)
          : basePointer(base_ptr), doorbellPointer(db_ptr),
            writeIndex(0), readIndex(0),
            numElts(size), hostReadIndexPtr(hri_ptr),
            stalledOnDmaBufAvailability(false),
            dmaInProgress(false)
        {  }
        uint64_t spaceRemaining() { return numElts - (writeIndex - readIndex); }
        uint64_t spaceUsed() { return writeIndex - readIndex; }
        uint32_t objSize() { return AQL_PACKET_SIZE; }
        uint32_t numObjs() { return numElts; }
        bool isFull() { return spaceRemaining() == 0; }
        bool isEmpty() { return spaceRemaining() == numElts; }

        uint64_t ptr(uint64_t ix)
        {
            return basePointer +
                ((ix % numElts) * objSize());
        }
};

/**
 * Internal ring buffer which is used to prefetch/store copies of the
 * in-memory HSA ring buffer.  Each packet in the queue has three implicit
 * states tracked by a packet's relative location to the write, read, and
 * dispatch pointers.
 *
 * FREE: Entry is empty
 * ALLOCATED: Entry has been allocated for a packet, but the DMA has not
 *            yet completed
 * SUBMITTED: Packet has been submitted to the HSADevice, but has not
 *            yet completed
 */
class AQLRingBuffer
{
   private:
     std::vector<hsa_kernel_dispatch_packet_t> _aqlBuf;
     std::string _name;
     std::vector<Addr> _hostDispAddresses;
     std::vector<bool> _aqlComplete;
     uint64_t _wrIdx;   // Points to next write location
     uint64_t _rdIdx;   // Read pointer of AQL buffer
     uint64_t _dispIdx; // Dispatch pointer of AQL buffer

  public:
     std::string name() {return _name;}
     AQLRingBuffer(uint32_t size, const std::string name);
     int allocEntry(uint32_t nBufReq);
     bool freeEntry(void *pkt);

     /**
      * the kernel may try to read from the dispatch packet,
      * so we need to keep the host address that corresponds
      * to each of the dispatch packets this AQL buffer is
      * storing. when we call submitPkt(), we send along the
      * corresponding host address for the packet so the
      * wavefront can properly initialize its SGPRs - which
      * may include a pointer to the dispatch packet
      */
     void
     saveHostDispAddr(Addr host_pkt_addr, int num_pkts, int ix)
     {
         for (int i = 0; i < num_pkts; ++i) {
            _hostDispAddresses[ix % numObjs()] = host_pkt_addr + i * objSize();
            ++ix;
         }
     }

     Addr
     hostDispAddr() const
     {
         return _hostDispAddresses[dispIdx() % numObjs()];
     }

     bool
     dispPending() const
     {
         int packet_type = (_aqlBuf[_dispIdx % _aqlBuf.size()].header
             >> HSA_PACKET_HEADER_TYPE) &
             ((1 << HSA_PACKET_HEADER_WIDTH_TYPE) - 1);
         return (_dispIdx < _wrIdx) && packet_type != HSA_PACKET_TYPE_INVALID;
     }

     uint32_t nFree() const { return _aqlBuf.size() - (_wrIdx - _rdIdx); }
     void *ptr(uint32_t ix) { return _aqlBuf.data() + (ix % _aqlBuf.size()); }
     uint32_t numObjs() const { return _aqlBuf.size(); };
     uint32_t objSize() const { return AQL_PACKET_SIZE; }
     uint64_t dispIdx() const { return _dispIdx; }
     uint64_t wrIdx() const { return _wrIdx; }
     uint64_t rdIdx() const { return _rdIdx; }
     uint64_t* rdIdxPtr() { return &_rdIdx; }
     void incRdIdx(uint64_t value) { _rdIdx += value; }
     void incWrIdx(uint64_t value) { _wrIdx += value; }
     void incDispIdx(uint64_t value) { _dispIdx += value; }

};

typedef struct QueueContext {
    HSAQueueDescriptor* qDesc;
    AQLRingBuffer* aqlBuf;
    QueueContext(HSAQueueDescriptor* q_desc,
                 AQLRingBuffer* aql_buf)
                 : qDesc(q_desc), aqlBuf(aql_buf)
    {}
    QueueContext() : qDesc(NULL), aqlBuf(NULL) {}
} QCntxt;

class HSAPacketProcessor: public DmaDevice
{
    friend class HWScheduler;
  protected:
    typedef void (DmaDevice::*DmaFnPtr)(Addr, int, Event*, uint8_t*, Tick);
    HSADevice *hsa_device;
    HWScheduler *hwSchdlr;

    // Structure to store the read values of dependency signals
    // from shared memory. Also used for tracking the status of
    // those reads while they are in progress
    class SignalState
    {
      public:
        SignalState()
            : pendingReads(0), allRead(false), discardRead(false)
        {
            values.resize(NumSignalsPerBarrier);
        }
        void handleReadDMA();
        int pendingReads;
        bool allRead;
        // If this queue is unmapped when there are pending reads, then
        // the pending reads has to be discarded.
        bool discardRead;
        // values stores the value of already read dependency signal
        std::vector<hsa_signal_value_t> values;
        void
        resetSigVals()
        {
            std::fill(values.begin(), values.end(), 1);
        }
    };

    class QueueProcessEvent : public Event
    {
      private:
        HSAPacketProcessor *hsaPP;
        uint32_t rqIdx;
      public:
        QueueProcessEvent(HSAPacketProcessor *_hsaPP, uint32_t _rqIdx)
            : Event(Default_Pri), hsaPP(_hsaPP), rqIdx(_rqIdx)
        {}
        virtual void process();
        virtual const char *description() const;
    };

    // Registered queue list entry; each entry has one queueDescriptor and
    // associated AQL buffer
    class RQLEntry
    {
      public:
        RQLEntry(HSAPacketProcessor *hsaPP, uint32_t rqIdx)
            : aqlProcessEvent(hsaPP, rqIdx) {}
        QCntxt qCntxt;
        bool dispPending() { return qCntxt.aqlBuf->dispPending() > 0; }
        SignalState depSignalRdState;
        QueueProcessEvent aqlProcessEvent;
    };
    // Keeps track of queueDescriptors of registered queues
    std::vector<class RQLEntry *> regdQList;

    void translateOrDie(Addr vaddr, Addr &paddr);
    void dmaVirt(DmaFnPtr, Addr host_addr, unsigned size, Event *event,
                 void *data, Tick delay = 0);

    void dmaReadVirt(Addr host_addr, unsigned size, Event *event,
                     void *data, Tick delay = 0);

    void dmaWriteVirt(Addr host_addr, unsigned size, Event *event,
                      void *data, Tick delay = 0);
    bool processPkt(void* pkt, uint32_t rl_idx, Addr host_pkt_addr);
    void displayQueueDescriptor(int pid, uint32_t rl_idx);

  public:
    HSAQueueDescriptor*
    getQueueDesc(uint32_t queId)
    {
        return regdQList.at(queId)->qCntxt.qDesc;
    }
    class RQLEntry*
    getRegdListEntry(uint32_t queId)
    {
        return regdQList.at(queId);
    }

    int numHWQueues;
    Addr pioAddr;
    Addr pioSize;
    Tick pioDelay;
    const Tick pktProcessDelay;

    typedef HSAPacketProcessorParams Params;
    HSAPacketProcessor(const Params *p);
    ~HSAPacketProcessor();
    void setDeviceQueueDesc(uint64_t hostReadIndexPointer,
                            uint64_t basePointer,
                            uint64_t queue_id,
                            uint32_t size);
    void unsetDeviceQueueDesc(uint64_t queue_id);
    void setDevice(HSADevice * dev);
    void updateReadIndex(int, uint32_t);
    void getCommandsFromHost(int pid, uint32_t rl_idx);

    // PIO interface
    virtual Tick read(Packet*);
    virtual Tick write(Packet*);
    virtual AddrRangeList getAddrRanges() const;
    void finishPkt(void *pkt, uint32_t rl_idx);
    void finishPkt(void *pkt) { finishPkt(pkt, 0); }
    void schedAQLProcessing(uint32_t rl_idx);

    class DepSignalsReadDmaEvent : public Event
    {
      protected:
        SignalState *signalState;
      public:
        DepSignalsReadDmaEvent(SignalState *ss)
            : Event(Default_Pri, AutoDelete), signalState(ss)
        {}
        virtual void process() { signalState->handleReadDMA(); }
        virtual const char *description() const;
    };

    /**
     * this event is used to update the read_disp_id field (the read pointer)
     * of the MQD, which is how the host code knows the status of the HQD's
     * read pointer
     */
    class UpdateReadDispIdDmaEvent : public Event
    {
      public:
        UpdateReadDispIdDmaEvent();

        void process() override { }
        const char *description() const override;

    };

    /**
     * Calls getCurrentEntry once the queueEntry has been dmaRead.
     */
    struct dma_series_ctx {
        // deal with the fact dma ops can complete out of issue order
        uint32_t pkts_ttl;
        uint32_t pkts_2_go;
        uint32_t start_ix;
        uint32_t rl_idx;

        dma_series_ctx(uint32_t _pkts_ttl,
                       uint32_t _pkts_2_go,
                       uint32_t _start_ix,
                       uint32_t _rl_idx)
            : pkts_ttl(_pkts_2_go), pkts_2_go(_pkts_2_go),
              start_ix(_start_ix), rl_idx(_rl_idx)
        {};
        ~dma_series_ctx() {};
    };

    class CmdQueueCmdDmaEvent : public Event
    {
      protected:
        HSAPacketProcessor *hsaPP;
        int pid;
        bool isRead;
        uint32_t ix_start;
        uint num_pkts;
        dma_series_ctx *series_ctx;
        void *dest_4debug;

      public:
        CmdQueueCmdDmaEvent(HSAPacketProcessor *hsaPP, int pid, bool isRead,
                            uint32_t dma_buf_ix, uint num_bufs,
                            dma_series_ctx *series_ctx, void *dest_4debug);
        virtual void process();
        virtual const char *description() const;
    };
};

#endif // __DEV_HSA_HSA_PACKET_PROCESSOR__
