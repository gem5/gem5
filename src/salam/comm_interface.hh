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

#ifndef __SALAM_COMM_INTERFACE_HH__
#define __SALAM_COMM_INTERFACE_HH__

#include <list>
#include <queue>
#include <vector>

#include "dev/arm/base_gic.hh"
#include "dev/io_device.hh"
#include "params/CommInterface.hh"
#include "salam/LLVMRead/debug_flags.hh"
#include "salam/LLVMRead/mem_request.hh"
#include "salam/acc_compute_unit.hh"
#include "salam/scratchpad_memory.hh"
#include "salam/stream_port.hh"

class AccComputeUnit;

class CommInterface : public BasicPioDevice
{
  protected:
    Addr io_addr;
    Addr io_size;
    Addr flag_size;
    Addr config_size;
    Addr FLAG_OFFSET;
    Addr CONFIG_OFFSET;
    Addr VAR_OFFSET;
    std::string devname;
    BaseGic *gic;
    int32_t int_num;
    bool use_premap_data;
    std::vector<Addr> data_base_ptrs;
    ByteOrder endian;
    bool debugEnabled;

  public:
    bool
    debug()
    {
        return debugEnabled;
    }

  protected:
    class MemSidePort : public StreamRequestPort
    {
        friend class CommInterface;

      private:
        CommInterface *owner;
        std::queue<PacketPtr> outstandingPkts;
        MemoryRequest *readReq;
        MemoryRequest *writeReq;
        bool readActive;
        bool writeActive;

      public:
        MemSidePort(const std::string &name, CommInterface *owner,
                    PortID id = InvalidPortID)
            : StreamRequestPort(name, owner, id), owner(owner)
        {
            readActive = false;
            writeActive = false;
            readReq = NULL;
            writeReq = NULL;
        }
        MemoryRequest *
        getReadReq()
        {
            return readReq;
        }
        MemoryRequest *
        getWriteReq()
        {
            return writeReq;
        }
        void
        setReadReq(MemoryRequest *req = nullptr)
        {
            readReq = req;
        }
        void
        setWriteReq(MemoryRequest *req = nullptr)
        {
            writeReq = req;
        }

      protected:
        virtual bool recvTimingResp(PacketPtr pkt);
        virtual void recvReqRetry();
        virtual void recvRangeChange() {};
        virtual Tick
        recvAtomic(PacketPtr pkt)
        {
            return 0;
        }
        virtual void recvFunctional(PacketPtr pkt) {};
        void
        setStalled(PacketPtr pkt)
        {
            outstandingPkts.push(pkt);
        }
        bool
        isStalled()
        {
            return !outstandingPkts.empty();
        }
        void sendPacket(PacketPtr pkt);
        bool
        active()
        {
            return readActive || writeActive;
        }
        bool
        reading()
        {
            return readActive;
        }
        bool
        writing()
        {
            return writeActive;
        }
        bool
        debug()
        {
            return owner->debug();
        }
    };

    class SPMPort : public ScratchpadRequestPort
    {
        friend class CommInterface;

      private:
        CommInterface *owner;
        std::queue<PacketPtr> outstandingPkts;
        MemoryRequest *readReq;
        MemoryRequest *writeReq;
        bool readActive;
        bool writeActive;

      public:
        SPMPort(const std::string &name, CommInterface *owner,
                PortID id = InvalidPortID)
            : ScratchpadRequestPort(name, owner, id), owner(owner)
        {
            readActive = false;
            writeActive = false;
            readReq = NULL;
            writeReq = NULL;
        }
        MemoryRequest *
        getReadReq()
        {
            return readReq;
        }
        MemoryRequest *
        getWriteReq()
        {
            return writeReq;
        }
        void
        setReadReq(MemoryRequest *req = nullptr)
        {
            readReq = req;
        }
        void
        setWriteReq(MemoryRequest *req = nullptr)
        {
            writeReq = req;
        }

      protected:
        virtual bool recvTimingResp(PacketPtr pkt);
        virtual void recvReqRetry();
        virtual void recvRangeChange() {};
        virtual Tick
        recvAtomic(PacketPtr pkt)
        {
            return 0;
        }
        virtual void recvFunctional(PacketPtr pkt) {};
        void
        setStalled(PacketPtr pkt)
        {
            outstandingPkts.push(pkt);
        }
        bool
        isStalled()
        {
            return !outstandingPkts.empty();
        }
        void sendPacket(PacketPtr pkt);
        bool
        active()
        {
            return readActive || writeActive;
        }
        bool
        reading()
        {
            return readActive;
        }
        bool
        writing()
        {
            return writeActive;
        }
        bool
        debug()
        {
            return owner->debug();
        }
    };

    class RegPort : public RequestPort
    {
        friend class CommInterface;

      private:
        CommInterface *owner;
        MemoryRequest *readReq;
        MemoryRequest *writeReq;

      public:
        RegPort(const std::string &name, CommInterface *_owner,
                PortID id = InvalidPortID)
            : RequestPort(name), owner(_owner)
        {}
        void
        setReadReq(MemoryRequest *req = nullptr)
        {
            readReq = req;
        }
        void
        setWriteReq(MemoryRequest *req = nullptr)
        {
            writeReq = req;
        }

      protected:
        virtual bool recvTimingResp(PacketPtr pkt);
        virtual void recvReqRetry();
        virtual void recvRangeChange() {};
        virtual Tick
        recvAtomic(PacketPtr pkt)
        {
            return 0;
        }
        virtual void recvFunctional(PacketPtr pkt) {};
        void sendPacket(PacketPtr pkt);
        bool
        debug()
        {
            return owner->debug();
        }
    };

    class TickEvent : public Event
    {
      private:
        CommInterface *comm;

      public:
        TickEvent(CommInterface *_comm) : Event(CPU_Tick_Pri), comm(_comm) {}
        void
        process()
        {
            comm->tick();
        }
        virtual const char *
        description() const
        {
            return "CommInterface tick";
        }
        bool
        debug()
        {
            return comm->debug();
        }
    };

    std::list<MemoryRequest *> readQueue;
    std::list<MemoryRequest *> writeQueue;
    std::list<MemoryRequest *> accRdQ;
    std::list<MemoryRequest *> accWrQ;

    int requestsInQueues;

    std::vector<MemSidePort *> localPorts;
    std::vector<MemSidePort *> globalPorts;
    std::vector<MemSidePort *> streamPorts;
    std::vector<SPMPort *> spmPorts;
    std::vector<RegPort *> regPorts;

    bool
    localPortsStalled()
    {
        for (auto it = localPorts.begin(); it != localPorts.end(); ++it) {
            if (!((*it)->isStalled())) {
                return false;
            }
        }
        return true;
    }
    bool
    globalPortsStalled()
    {
        for (auto it = globalPorts.begin(); it != globalPorts.end(); ++it) {
            if (!((*it)->isStalled())) {
                return false;
            }
        }
        return true;
    }
    bool
    streamPortsStalled()
    {
        for (auto it = streamPorts.begin(); it != streamPorts.end(); ++it) {
            if (!((*it)->isStalled())) {
                return false;
            }
        }
        return true;
    }
    bool
    spmPortsStalled()
    {
        for (auto port : spmPorts) {
            if (!(port->isStalled())) {
                return false;
            }
        }
        return true;
    }
    bool
    allPortsStalled()
    {
        return localPortsStalled() && globalPortsStalled() &&
               streamPortsStalled() && spmPortsStalled();
    }
    bool inStreamRange(Addr add);
    bool inSPMRange(Addr add);
    bool inRegRange(Addr add);
    bool inLocalRange(Addr add);
    bool inGlobalRange(Addr add);
    MemSidePort *getValidLocalPort(Addr add, bool read);
    MemSidePort *getValidGlobalPort(Addr add, bool read);
    MemSidePort *getValidStreamPort(Addr add, size_t len, bool read);
    SPMPort *getValidSPMPort(Addr add, size_t len, bool read);
    RegPort *getValidRegPort(Addr add);

    CommInterface *comm;
    RequestorID masterId;
    TickEvent tickEvent;
    unsigned cacheLineSize;

    virtual void checkMMR();
    virtual void processMemoryRequests();
    virtual void tick();

    bool running;
    bool computationNeeded;
    bool int_flag;

    void tryRead(MemSidePort *port);
    void tryWrite(MemSidePort *port);

    void tryRead(SPMPort *port);
    void tryWrite(SPMPort *port);

    void tryRead(RegPort *port);
    void tryWrite(RegPort *port);

    Addr dataAddr;

    uint8_t *mmreg;

    bool processingDone;
    int processDelay;
    float clock_period;

    bool reset_spm;

    AccComputeUnit *cu;

  public:
    PARAMS(CommInterface);

    CommInterface(const CommInterfaceParams &p);

    void startup();

    virtual Tick read(PacketPtr pkt);

    virtual Tick write(PacketPtr pkt);

    Port &getPort(const std::string &if_name,
                  PortID idk = InvalidPortID) override;

    void recvPacket(PacketPtr pkt);

    // Addr value stored in gep register, length based on data type
    void enqueueRead(MemoryRequest *req);

    void enqueueWrite(MemoryRequest *req);

    bool
    isRunning()
    {
        return running;
    }
    bool
    isCompNeeded()
    {
        return computationNeeded;
    }

    uint64_t getGlobalVar(unsigned offset, unsigned size);
    int
    getProcessDelay()
    {
        return processDelay;
    }
    virtual int
    getReadPorts()
    {
        return 0;
    }
    virtual int
    getWritePorts()
    {
        return 0;
    }
    virtual int
    getReadBusWidth()
    {
        return 0;
    }
    virtual int
    getWriteBusWidth()
    {
        return 0;
    }
    virtual int
    getPmemRange()
    {
        return 0;
    }
    void
    registerCompUnit(AccComputeUnit *compunit)
    {
        cu = compunit;
    }
    virtual void finish();

    MemoryRequest *findMemRequest(PacketPtr pkt, bool isRead);
    void clearMemRequest(MemoryRequest *req, bool isRead);
    virtual void
    refreshMemPorts()
    {}
    std::string
    getName() const
    {
        return name();
    }

    virtual bool
    isBaseCommInterface()
    {
        return true;
    }

  protected:
};

#endif //__SALAM_COMM_INTERFACE_HH__

/*
 * MM Register Layout
 * | Location of Data 32bits | Compute Finished 1bit | Unused 30bits |
 * | Start Operation 1bit |
 */
