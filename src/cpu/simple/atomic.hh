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
 */

#ifndef __CPU_SIMPLE_ATOMIC_HH__
#define __CPU_SIMPLE_ATOMIC_HH__

#include "cpu/simple/base.hh"

class AtomicSimpleCPU : public BaseSimpleCPU
{
  public:

    struct Params : public BaseSimpleCPU::Params {
        int width;
        bool simulate_stalls;
    };

    AtomicSimpleCPU(Params *params);
    virtual ~AtomicSimpleCPU();

    virtual void init();

  public:
    //
    enum Status {
        Running,
        Idle,
        SwitchedOut
    };

  protected:
    Status _status;

    Status status() const { return _status; }

  private:

    struct TickEvent : public Event
    {
        AtomicSimpleCPU *cpu;

        TickEvent(AtomicSimpleCPU *c);
        void process();
        const char *description();
    };

    TickEvent tickEvent;

    const int width;
    const bool simulate_stalls;

    // main simulation loop (one cycle)
    void tick();

    class CpuPort : public Port
    {

        AtomicSimpleCPU *cpu;

      public:

        CpuPort(const std::string &_name, AtomicSimpleCPU *_cpu)
            : Port(_name), cpu(_cpu)
        { }

      protected:

        virtual bool recvTiming(Packet *pkt);

        virtual Tick recvAtomic(Packet *pkt);

        virtual void recvFunctional(Packet *pkt);

        virtual void recvStatusChange(Status status);

        virtual Packet *recvRetry();

        virtual void getDeviceAddressRanges(AddrRangeList &resp,
            AddrRangeList &snoop)
        { resp.clear(); snoop.clear(); }
    };

    CpuPort icachePort;
    CpuPort dcachePort;

    Request *ifetch_req;
    Packet  *ifetch_pkt;
    Request *data_read_req;
    Packet  *data_read_pkt;
    Request *data_write_req;
    Packet  *data_write_pkt;

    bool dcache_access;
    Tick dcache_complete;

  public:

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

    void switchOut(Sampler *s);
    void takeOverFrom(BaseCPU *oldCPU);

    virtual void activateContext(int thread_num, int delay);
    virtual void suspendContext(int thread_num);

    template <class T>
    Fault read(Addr addr, T &data, unsigned flags);

    template <class T>
    Fault write(T data, Addr addr, unsigned flags, uint64_t *res);
};

#endif // __CPU_SIMPLE_ATOMIC_HH__
