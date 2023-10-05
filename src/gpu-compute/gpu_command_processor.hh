/*
 * Copyright (c) 2018 Advanced Micro Devices, Inc.
 * All rights reserved.
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

#include <cstdint>
#include <functional>

#include "arch/amdgpu/vega/gpu_registers.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/GPUCommandProc.hh"
#include "dev/dma_virt_device.hh"
#include "dev/hsa/hsa_packet_processor.hh"
#include "dev/hsa/hsa_signal.hh"
#include "gpu-compute/dispatcher.hh"
#include "gpu-compute/gpu_compute_driver.hh"
#include "gpu-compute/hsa_queue_entry.hh"
#include "params/GPUCommandProcessor.hh"
#include "sim/full_system.hh"

namespace gem5
{

struct GPUCommandProcessorParams;
class GPUComputeDriver;
class GPUDispatcher;
class Shader;

class GPUCommandProcessor : public DmaVirtDevice
{
  public:
    typedef GPUCommandProcessorParams Params;
    typedef std::function<void(const uint64_t &)> HsaSignalCallbackFunction;

    GPUCommandProcessor() = delete;
    GPUCommandProcessor(const Params &p);

    HSAPacketProcessor& hsaPacketProc();
    RequestorID vramRequestorId();

    void setGPUDevice(AMDGPUDevice *gpu_device);
    void setShader(Shader *shader);
    Shader* shader();
    GPUComputeDriver* driver();

    enum AgentCmd
    {
      Nop = 0,
      Steal = 1
    };

    void submitAgentDispatchPkt(void *raw_pkt, uint32_t queue_id,
                           Addr host_pkt_addr);
    void submitDispatchPkt(void *raw_pkt, uint32_t queue_id,
                           Addr host_pkt_addr);
    void submitVendorPkt(void *raw_pkt, uint32_t queue_id,
                         Addr host_pkt_addr);
    void attachDriver(GPUComputeDriver *driver);

    void dispatchPkt(HSAQueueEntry *task);
    void signalWakeupEvent(uint32_t event_id);

    Tick write(PacketPtr pkt) override { return 0; }
    Tick read(PacketPtr pkt) override { return 0; }
    AddrRangeList getAddrRanges() const override;
    System *system();

    void sendCompletionSignal(Addr signal_handle);
    void updateHsaSignal(Addr signal_handle, uint64_t signal_value,
                         HsaSignalCallbackFunction function =
                            [] (const uint64_t &) { });
    void updateHsaSignalAsync(Addr signal_handle, int64_t diff);
    void updateHsaSignalData(Addr value_addr, int64_t diff,
                             uint64_t *prev_value);
    void updateHsaSignalDone(uint64_t *signal_value);
    void updateHsaMailboxData(Addr signal_handle, uint64_t *mailbox_value);
    void updateHsaEventData(Addr signal_handle, uint64_t *event_value);

    uint64_t functionalReadHsaSignal(Addr signal_handle);

    Addr getHsaSignalValueAddr(Addr signal_handle)
    {
        return signal_handle + offsetof(amd_signal_t, value);
    }

    Addr getHsaSignalMailboxAddr(Addr signal_handle)
    {
        return signal_handle + offsetof(amd_signal_t, event_mailbox_ptr);
    }

    Addr getHsaSignalEventAddr(Addr signal_handle)
    {
        return signal_handle + offsetof(amd_signal_t, event_id);
    }

  private:
    Shader *_shader;
    GPUDispatcher &dispatcher;
    GPUComputeDriver *_driver;
    AMDGPUDevice *gpuDevice;
    VegaISA::Walker *walker;

    // Typedefing dmaRead and dmaWrite function pointer
    typedef void (DmaDevice::*DmaFnPtr)(Addr, int, Event*, uint8_t*, Tick);
    void initABI(HSAQueueEntry *task);
    HSAPacketProcessor *hsaPP;
    TranslationGenPtr translate(Addr vaddr, Addr size) override;

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
    void
    ReadDispIdOffsetDmaEvent(HSAQueueEntry *task,
                             const uint32_t &readDispIdOffset)
    {
        /**
         * Now that the read pointer's offset from the base of
         * the MQD is known, we can use that to calculate the
         * the address of the MQD itself, the dispatcher will
         * DMA that into the HSAQueueEntry when a kernel is
         * launched.
         */
        task->hostAMDQueueAddr = hsaPP->getQueueDesc(
            task->queueId())->hostReadIndexPtr - readDispIdOffset;

        /**
         * DMA a copy of the MQD into the task. some fields of
         * the MQD will be used to initialize register state in VI
         */
        auto *mqdDmaEvent = new DmaVirtCallback<int>(
            [ = ] (const int &) { MQDDmaEvent(task); });

        dmaReadVirt(task->hostAMDQueueAddr,
                    sizeof(_amd_queue_t), mqdDmaEvent, &task->amdQueue);
    }

    /**
     * Perform a DMA read of the MQD that corresponds to a hardware
     * queue descriptor (HQD). We store a copy of the MQD in the
     * HSAQueueEntry object so we can send a copy of it along with
     * a dispatch packet, which is needed to initialize register
     * state.
     */
     void
     MQDDmaEvent(HSAQueueEntry *task)
     {
        /**
         *  dGPUs on any version of ROCm and APUs starting with ROCm 2.2
         *  can perform lazy allocation of private segment (scratch) memory,
         *  where the runtime will intentianally underallocate scratch
         *  resources to save framebuffer (or system on APU) memory.
         *  If we don't have enough scratch memory to launch this kernel,
         *  we need to raise a recoverable error code to the runtime by
         *  asserting queue_inactive_signal for the queue.  The runtime will
         *  then try to allocate more scratch and reset this signal.  When
         *  the signal is reset we should check that the runtime was
         *  successful and then proceed to launch the kernel.
         */
        if ((task->privMemPerItem() * VegaISA::NumVecElemPerVecReg) >
            task->amdQueue.compute_tmpring_size_wavesize * 1024) {
            // TODO: Raising this signal will potentially nuke scratch
            // space for in-flight kernels that were launched from this
            // queue.  We need to drain all kernels and deschedule the
            // queue before raising this signal. For now, just assert if
            // there are any in-flight kernels and tell the user that this
            // feature still needs to be implemented.
            fatal_if(hsaPP->inFlightPkts(task->queueId()) > 1,
                        "Needed more scratch, but kernels are in flight for "
                        "this queue and it is unsafe to reallocate scratch. "
                        "We need to implement additional intelligence in the "
                        "hardware scheduling logic to support CP-driven "
                        "queue draining and scheduling.");
            DPRINTF(GPUCommandProc, "Not enough scratch space to launch "
                    "kernel (%x available, %x requested bytes per "
                    "workitem). Asking host runtime to allocate more "
                    "space.\n",
                    task->amdQueue.compute_tmpring_size_wavesize * 1024,
                    task->privMemPerItem());

            updateHsaSignal(task->amdQueue.queue_inactive_signal.handle, 1,
                            [ = ] (const uint64_t &dma_buffer)
                                { WaitScratchDmaEvent(task, dma_buffer); });

        } else {
            DPRINTF(GPUCommandProc, "Sufficient scratch space, launching "
                    "kernel (%x available, %x requested bytes per "
                    "workitem).\n",
                    task->amdQueue.compute_tmpring_size_wavesize * 1024,
                    task->privMemPerItem());
            dispatchPkt(task);
        }
    }

    /**
     * Poll on queue_inactive signal until the runtime can get around to
     * taking care of our lack of scratch space.
     */
    void
    WaitScratchDmaEvent(HSAQueueEntry *task, const uint64_t &dmaBuffer)
    {
        if (dmaBuffer == 0) {
            DPRINTF(GPUCommandProc, "Host scratch allocation complete. "
                    "Attempting to re-read MQD\n");
            /**
            * Runtime will have updated the MQD to give us more scratch
            * space.  Read it out and continue to pester the runtime until
            * we get all that we need to launch.
            *
            * TODO: Technically only need to update private segment fields
            * since other MQD entries won't change since we last read them.
            */
            auto cb = new DmaVirtCallback<int>(
                [ = ] (const int &) { MQDDmaEvent(task); });

            dmaReadVirt(task->hostAMDQueueAddr, sizeof(_amd_queue_t), cb,
                        &task->amdQueue);
        } else {
            /**
            * Poll until runtime signals us that scratch space has been
            * allocated.
            */
            Addr value_addr = getHsaSignalValueAddr(
                task->amdQueue.queue_inactive_signal.handle);
            DPRINTF(GPUCommandProc, "Polling queue inactive signal at "
                    "%p.\n", value_addr);
            auto cb = new DmaVirtCallback<uint64_t>(
                [ = ] (const uint64_t &dma_buffer)
                { WaitScratchDmaEvent(task, dma_buffer); } );

            /**
             * Delay for a large amount of ticks to give the CPU time to
             * setup the scratch space. The delay should be non-zero to since
             * this method calls back itself and can cause an infinite loop
             * in the event queue if the allocation is not completed by the
             * first time this is called.
             */
            dmaReadVirt(value_addr, sizeof(Addr), cb, &cb->dmaBuffer, 1e9);
        }
    }
};

} // namespace gem5

#endif // __DEV_HSA_GPU_COMMAND_PROCESSOR_HH__
