/*
 * Copyright (c) 2011-2015,2018 Advanced Micro Devices, Inc.
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
 * Authors: Brad Beckmann,
 *          Marc Orr,
 *          Anthony Gutierrez
 */

#ifndef __GPU_DISPATCHER_HH__
#define __GPU_DISPATCHER_HH__

#include <queue>
#include <vector>

#include "base/statistics.hh"
#include "dev/dma_device.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/ndrange.hh"
#include "gpu-compute/qstruct.hh"
#include "mem/port.hh"
#include "params/GpuDispatcher.hh"

class BaseCPU;
class Shader;

class GpuDispatcher : public DmaDevice
{
    public:
        typedef GpuDispatcherParams Params;

        MasterID masterId() { return _masterId; }

    protected:
        MasterID _masterId;

        // Base and length of PIO register space
        Addr pioAddr;
        Addr pioSize;
        Tick pioDelay;

        HsaQueueEntry curTask;

        std::unordered_map<int, NDRange> ndRangeMap;
        NDRange ndRange;

        // list of kernel_ids to launch
        std::queue<int> execIds;
        // list of kernel_ids that have finished
        std::queue<int> doneIds;

        uint64_t dispatchCount;
        // is there a kernel in execution?
        bool dispatchActive;

        BaseCPU *cpu;
        Shader *shader;
        ClDriver *driver;
        EventFunctionWrapper tickEvent;


        static GpuDispatcher *instance;

        // sycall emulation mode can have only 1 application running(?)
        // else we have to do some pid based tagging
        // unused
        typedef std::unordered_map<uint64_t, uint64_t> TranslationBuffer;
        TranslationBuffer tlb;

    public:
        /*statistics*/
        Stats::Scalar num_kernelLaunched;
        GpuDispatcher(const Params *p);

        ~GpuDispatcher() { }

        void exec();
        virtual void serialize(CheckpointOut &cp) const;
        virtual void unserialize(CheckpointIn &cp);
        void notifyWgCompl(Wavefront *w);
        void scheduleDispatch();
        void accessUserVar(BaseCPU *cpu, uint64_t addr, int val, int off);

        // using singleton so that glue code can pass pointer locations
        // to the dispatcher. when there are multiple dispatchers, we can
        // call something like getInstance(index)
        static void
         setInstance(GpuDispatcher *_instance)
        {
            instance = _instance;
        }

        static GpuDispatcher* getInstance() { return instance; }

        class TLBPort : public MasterPort
        {
          public:

            TLBPort(const std::string &_name, GpuDispatcher *_dispatcher)
                : MasterPort(_name, _dispatcher), dispatcher(_dispatcher) { }

          protected:
            GpuDispatcher *dispatcher;

            virtual bool recvTimingResp(PacketPtr pkt) { return true; }
            virtual Tick recvAtomic(PacketPtr pkt) { return 0; }
            virtual void recvFunctional(PacketPtr pkt) { }
            virtual void recvRangeChange() { }
            virtual void recvReqRetry() { }

        };

        TLBPort *tlbPort;

        Port &getPort(const std::string &if_name,
                      PortID idx=InvalidPortID) override;

        AddrRangeList getAddrRanges() const;
        Tick read(PacketPtr pkt);
        Tick write(PacketPtr pkt);

        // helper functions to retrieve/set GPU attributes
        int getNumCUs();
        int wfSize() const;
        void setFuncargsSize(int funcargs_size);

        /** Returns the size of the static hardware context of a wavefront */
        uint32_t getStaticContextSize() const;
};

#endif // __GPU_DISPATCHER_HH__
