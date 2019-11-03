/*
 * Copyright (c) 2013, 2018-2019 ARM Limited
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
 * Authors: Stan Czerniawski
 */

#ifndef __DEV_ARM_SMMU_V3_PROC_HH__
#define __DEV_ARM_SMMU_V3_PROC_HH__

#include <list>
#include <queue>
#include <string>

#include "base/coroutine.hh"
#include "base/types.hh"
#include "mem/packet.hh"

class SMMUv3SlaveInterface;

/*
 * The meaning of these becomes apparent when you
 * look at runProcessAtomic()/runProcessTiming().
 */
enum SMMUActionType {
    ACTION_INITIAL_NOP,
    ACTION_SEND_REQ,
    ACTION_SEND_REQ_FINAL,
    ACTION_SEND_RESP,
    ACTION_SEND_RESP_ATS,
    ACTION_DELAY,
    ACTION_SLEEP,
    ACTION_TERMINATE,
};

struct SMMUAction
{
    SMMUActionType type;
    PacketPtr pkt;
    SMMUv3SlaveInterface *ifc;
    Tick delay;
};

class SMMUv3;
class SMMUProcess;

struct SMMUSemaphore
{
    explicit SMMUSemaphore(unsigned _max) :
        count(_max), max(_max)
    {}

    unsigned count;
    unsigned max;
    std::queue<SMMUProcess *> queue;
};

struct SMMUSignal
{
    std::list<SMMUProcess *> waiting;
};

class SMMUProcess : public Packet::SenderState
{
  private:
    typedef m5::Coroutine<PacketPtr, SMMUAction> Coroutine;

    Coroutine *coroutine;
    std::string myName;

    void wakeup();

  protected:
    typedef Coroutine::CallerType Yield;

    SMMUv3 &smmu;

    void reinit();

    virtual void main(Yield &yield) = 0;

    void doRead(Yield &yield, Addr addr, void *ptr, size_t size);
    void doWrite(Yield &yield, Addr addr, const void *ptr, size_t size);
    void doDelay(Yield &yield, Cycles cycles);
    void doSleep(Yield &yield);

    void doSemaphoreDown(Yield &yield, SMMUSemaphore &sem);
    void doSemaphoreUp(SMMUSemaphore &sem);

    void doWaitForSignal(Yield &yield, SMMUSignal &sig);
    void doBroadcastSignal(SMMUSignal &sig);

    void scheduleWakeup(Tick when);

 public:
    SMMUProcess(const std::string &name, SMMUv3 &_smmu);
    virtual ~SMMUProcess();

    SMMUAction run(PacketPtr pkt);

    const std::string name() const { return myName; };
};

#endif /* __DEV_ARM_SMMU_V3_PROC_HH__ */
