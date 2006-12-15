/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 * Authors: Steve Reinhardt
 */

#ifndef __CPU_SIMPLE_TIMING_HH__
#define __CPU_SIMPLE_TIMING_HH__

#include "cpu/simple/base.hh"

class TimingSimpleCPU : public BaseSimpleCPU
{
  public:

    struct Params : public BaseSimpleCPU::Params {
    };

    TimingSimpleCPU(Params *params);
    virtual ~TimingSimpleCPU();

    virtual void init();

  public:
    //
    enum Status {
        Idle,
        Running,
        IcacheRetry,
        IcacheWaitResponse,
        IcacheWaitSwitch,
        DcacheRetry,
        DcacheWaitResponse,
        DcacheWaitSwitch,
        SwitchedOut
    };

  protected:
    Status _status;

    Status status() const { return _status; }

    Event *drainEvent;

    Event *fetchEvent;

  private:

    class CpuPort : public Port
    {
      protected:
        TimingSimpleCPU *cpu;
        Tick lat;

      public:

        CpuPort(const std::string &_name, TimingSimpleCPU *_cpu, Tick _lat)
            : Port(_name, _cpu), cpu(_cpu), lat(_lat)
        { }

        bool snoopRangeSent;

      protected:

        virtual Tick recvAtomic(PacketPtr pkt);

        virtual void recvFunctional(PacketPtr pkt);

        virtual void recvStatusChange(Status status);

        virtual void getDeviceAddressRanges(AddrRangeList &resp,
            AddrRangeList &snoop)
        { resp.clear(); snoop.clear(); snoop.push_back(RangeSize(0,0)); }

        struct TickEvent : public Event
        {
            PacketPtr pkt;
            TimingSimpleCPU *cpu;

            TickEvent(TimingSimpleCPU *_cpu)
                :Event(&mainEventQueue), cpu(_cpu) {}
            const char *description() { return "Timing CPU clock event"; }
            void schedule(PacketPtr _pkt, Tick t);
        };

    };

    class IcachePort : public CpuPort
    {
      public:

        IcachePort(TimingSimpleCPU *_cpu, Tick _lat)
            : CpuPort(_cpu->name() + "-iport", _cpu, _lat), tickEvent(_cpu)
        { }

      protected:

        virtual bool recvTiming(PacketPtr pkt);

        virtual void recvRetry();

        struct ITickEvent : public TickEvent
        {

            ITickEvent(TimingSimpleCPU *_cpu)
                : TickEvent(_cpu) {}
            void process();
            const char *description() { return "Timing CPU clock event"; }
        };

        ITickEvent tickEvent;

    };

    class DcachePort : public CpuPort
    {
      public:

        DcachePort(TimingSimpleCPU *_cpu, Tick _lat)
            : CpuPort(_cpu->name() + "-dport", _cpu, _lat), tickEvent(_cpu)
        { }

      protected:

        virtual bool recvTiming(PacketPtr pkt);

        virtual void recvRetry();

        struct DTickEvent : public TickEvent
        {
            DTickEvent(TimingSimpleCPU *_cpu)
                : TickEvent(_cpu) {}
            void process();
            const char *description() { return "Timing CPU clock event"; }
        };

        DTickEvent tickEvent;

    };

    IcachePort icachePort;
    DcachePort dcachePort;

    PacketPtr ifetch_pkt;
    PacketPtr dcache_pkt;

    int cpu_id;
    Tick previousTick;

  public:

    virtual Port *getPort(const std::string &if_name, int idx = -1);

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

    virtual unsigned int drain(Event *drain_event);
    virtual void resume();

    void switchOut();
    void takeOverFrom(BaseCPU *oldCPU);

    virtual void activateContext(int thread_num, int delay);
    virtual void suspendContext(int thread_num);

    template <class T>
    Fault read(Addr addr, T &data, unsigned flags);

    template <class T>
    Fault write(T data, Addr addr, unsigned flags, uint64_t *res);

    void fetch();
    void completeIfetch(PacketPtr );
    void completeDataAccess(PacketPtr );
    void advanceInst(Fault fault);
  private:
    void completeDrain();
};

#endif // __CPU_SIMPLE_TIMING_HH__
