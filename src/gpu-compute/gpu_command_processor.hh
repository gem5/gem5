/*
 * Copyright (c) 2018 Advanced Micro Devices, Inc.
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
 * Authors: Anthony Gutierrez
 */

/**
 * @file
 * The GPUCommandProcessor (CP) is responsible for accepting commands, in
 * the form of HSA AQL packets, from the HSA packet processor (HSAPP). The CP
 * works with several components, including the HSAPP and the dispatcher.
 * When the HSAPP sends a ready task to the CP, it will perform the necessary
 * operations to extract relevant data structures from memory, such as the
 * AQL queue descriptor and AQL packet, and initializes register state for the
 * task's wavefronts.
 */

#ifndef __DEV_HSA_GPU_COMMAND_PROCESSOR_HH__
#define __DEV_HSA_GPU_COMMAND_PROCESSOR_HH__

#include "dev/hsa/hsa_device.hh"
#include "gpu-compute/hsa_queue_entry.hh"

struct GPUCommandProcessorParams;
class GPUDispatcher;
class Shader;

class GPUCommandProcessor : public HSADevice
{
  public:
    typedef GPUCommandProcessorParams Params;

    GPUCommandProcessor() = delete;
    GPUCommandProcessor(const Params *p);

    void setShader(Shader *shader);
    Shader* shader();

    void submitDispatchPkt(void *raw_pkt, uint32_t queue_id,
                           Addr host_pkt_addr) override;
    void submitVendorPkt(void *raw_pkt, uint32_t queue_id,
                         Addr host_pkt_addr) override;
    void dispatchPkt(HSAQueueEntry *task);

    Tick write(PacketPtr pkt) override { return 0; }
    Tick read(PacketPtr pkt) override { return 0; }
    AddrRangeList getAddrRanges() const override;
    System *system();

  private:
    Shader *_shader;
    GPUDispatcher &dispatcher;

    void initABI(HSAQueueEntry *task);

    /**
     * Perform a DMA read of the read_dispatch_id_field_base_byte_offset
     * field, which follows directly after the read_dispatch_id (the read
     * pointer) in the amd_hsa_queue_t struct (aka memory queue descriptor
     * (MQD)), to find the base address of the MQD. The MQD is the runtime's
     * soft representation of a HW queue descriptor (HQD).
     *
     * Any fields below the read dispatch ID in the amd_hsa_queue_t should
     * not change according to the HSA standard, therefore we should be able
     * to get them based on their known relative position to the read dispatch
     * ID.
     */
    class ReadDispIdOffsetDmaEvent : public DmaCallback
    {
      public:
        ReadDispIdOffsetDmaEvent(GPUCommandProcessor &gpu_cmd_proc,
                                 HSAQueueEntry *task)
            : DmaCallback(), readDispIdOffset(0), gpuCmdProc(gpu_cmd_proc),
              _task(task)
        {
        }

        void
        process() override
        {
            /**
             * Now that the read pointer's offset from the base of
             * the MQD is known, we can use that to calculate the
             * the address of the MQD itself, the dispatcher will
             * DMA that into the HSAQueueEntry when a kernel is
             * launched.
             */
            _task->hostAMDQueueAddr
                = gpuCmdProc.hsaPP->getQueueDesc(_task->queueId())
                    ->hostReadIndexPtr - readDispIdOffset;

            /**
             * DMA a copy of the MQD into the task. Some fields of
             * the MQD will be used to initialize register state.
             */
            auto *mqdDmaEvent = new MQDDmaEvent(gpuCmdProc, _task);
            gpuCmdProc.dmaReadVirt(_task->hostAMDQueueAddr,
                                   sizeof(_amd_queue_t), mqdDmaEvent,
                                   &_task->amdQueue);
        }

        uint32_t readDispIdOffset;

      private:
        GPUCommandProcessor &gpuCmdProc;
        HSAQueueEntry *_task;
    };

    /**
     * Perform a DMA read of the MQD that corresponds to a hardware
     * queue descriptor (HQD). We store a copy of the MQD in the
     * HSAQueueEntry object so we can send a copy of it along with
     * a dispatch packet, which is needed to initialize register
     * state.
     */
    class MQDDmaEvent : public DmaCallback
    {
      public:
        MQDDmaEvent(GPUCommandProcessor &gpu_cmd_proc, HSAQueueEntry *task)
            : DmaCallback(), gpuCmdProc(gpu_cmd_proc), _task(task)
        {
        }

        void
        process() override
        {
            gpuCmdProc.dispatchPkt(_task);
        }

      private:
        GPUCommandProcessor &gpuCmdProc;
        HSAQueueEntry *_task;
    };
};

#endif // __DEV_HSA_GPU_COMMAND_PROCESSOR_HH__
