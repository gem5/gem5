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
 */

#include "gpu-compute/gpu_command_processor.hh"

#include <cassert>

#include "base/chunk_generator.hh"
#include "debug/GPUCommandProc.hh"
#include "debug/GPUKernelInfo.hh"
#include "gpu-compute/dispatcher.hh"
#include "mem/se_translating_port_proxy.hh"
#include "mem/translating_port_proxy.hh"
#include "params/GPUCommandProcessor.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"
#include "sim/proxy_ptr.hh"
#include "sim/syscall_emul_buf.hh"

namespace gem5
{

GPUCommandProcessor::GPUCommandProcessor(const Params &p)
    : DmaVirtDevice(p), dispatcher(*p.dispatcher), _driver(nullptr),
      hsaPP(p.hsapp)
{
    assert(hsaPP);
    hsaPP->setDevice(this);
    dispatcher.setCommandProcessor(this);
}

HSAPacketProcessor&
GPUCommandProcessor::hsaPacketProc()
{
    return *hsaPP;
}

TranslationGenPtr
GPUCommandProcessor::translate(Addr vaddr, Addr size)
{
    // Grab the process and try to translate the virtual address with it; with
    // new extensions, it will likely be wrong to just arbitrarily grab context
    // zero.
    auto process = sys->threads[0]->getProcessPtr();

    return process->pTable->translateRange(vaddr, size);
}

/**
 * submitDispatchPkt() is the entry point into the CP from the HSAPP
 * and is only meant to be used with AQL kernel dispatch packets.
 * After the HSAPP receives and extracts an AQL packet, it sends
 * it to the CP, which is responsible for gathering all relevant
 * information about a task, initializing CU state, and sending
 * it to the dispatcher for WG creation and dispatch.
 *
 * First we need capture all information from the the AQL pkt and
 * the code object, then store it in an HSAQueueEntry. Once the
 * packet and code are extracted, we extract information from the
 * queue descriptor that the CP needs to perform state initialization
 * on the CU. Finally we call dispatch() to send the task to the
 * dispatcher. When the task completely finishes, we call finishPkt()
 * on the HSA packet processor in order to remove the packet from the
 * queue, and notify the runtime that the task has completed.
 */
void
GPUCommandProcessor::submitDispatchPkt(void *raw_pkt, uint32_t queue_id,
                                       Addr host_pkt_addr)
{
    static int dynamic_task_id = 0;
    _hsa_dispatch_packet_t *disp_pkt = (_hsa_dispatch_packet_t*)raw_pkt;

    /**
     * we need to read a pointer in the application's address
     * space to pull out the kernel code descriptor.
     */
    auto *tc = sys->threads[0];

    TranslatingPortProxy fs_proxy(tc);
    SETranslatingPortProxy se_proxy(tc);
    PortProxy &virt_proxy = FullSystem ? fs_proxy : se_proxy;

    /**
     * The kernel_object is a pointer to the machine code, whose entry
     * point is an 'amd_kernel_code_t' type, which is included in the
     * kernel binary, and describes various aspects of the kernel. The
     * desired entry is the 'kernel_code_entry_byte_offset' field,
     * which provides the byte offset (positive or negative) from the
     * address of the amd_kernel_code_t to the start of the machine
     * instructions.
     */
    AMDKernelCode akc;
    virt_proxy.readBlob(disp_pkt->kernel_object, (uint8_t*)&akc,
        sizeof(AMDKernelCode));

    DPRINTF(GPUCommandProc, "GPU machine code is %lli bytes from start of the "
        "kernel object\n", akc.kernel_code_entry_byte_offset);

    DPRINTF(GPUCommandProc,"GPUCommandProc: Sending dispatch pkt to %lu\n",
        (uint64_t)tc->cpuId());


    Addr machine_code_addr = (Addr)disp_pkt->kernel_object
        + akc.kernel_code_entry_byte_offset;

    DPRINTF(GPUCommandProc, "Machine code starts at addr: %#x\n",
        machine_code_addr);

    std::string kernel_name;

    /**
     * BLIT kernels don't have symbol names.  BLIT kernels are built-in compute
     * kernels issued by ROCm to handle DMAs for dGPUs when the SDMA
     * hardware engines are unavailable or explicitly disabled.  They can also
     * be used to do copies that ROCm things would be better performed
     * by the shader than the SDMA engines.  They are also sometimes used on
     * APUs to implement asynchronous memcopy operations from 2 pointers in
     * host memory.  I have no idea what BLIT stands for.
     * */
    if (akc.runtime_loader_kernel_symbol) {
        kernel_name = "Some kernel";
    } else {
        kernel_name = "Blit kernel";
    }

    DPRINTF(GPUKernelInfo, "Kernel name: %s\n", kernel_name.c_str());

    HSAQueueEntry *task = new HSAQueueEntry(kernel_name, queue_id,
        dynamic_task_id, raw_pkt, &akc, host_pkt_addr, machine_code_addr);

    DPRINTF(GPUCommandProc, "Task ID: %i Got AQL: wg size (%dx%dx%d), "
        "grid size (%dx%dx%d) kernarg addr: %#x, completion "
        "signal addr:%#x\n", dynamic_task_id, disp_pkt->workgroup_size_x,
        disp_pkt->workgroup_size_y, disp_pkt->workgroup_size_z,
        disp_pkt->grid_size_x, disp_pkt->grid_size_y,
        disp_pkt->grid_size_z, disp_pkt->kernarg_address,
        disp_pkt->completion_signal);

    DPRINTF(GPUCommandProc, "Extracted code object: %s (num vector regs: %d, "
        "num scalar regs: %d, code addr: %#x, kernarg size: %d, "
        "LDS size: %d)\n", kernel_name, task->numVectorRegs(),
        task->numScalarRegs(), task->codeAddr(), 0, 0);

    initABI(task);
    ++dynamic_task_id;
}

uint64_t
GPUCommandProcessor::functionalReadHsaSignal(Addr signal_handle)
{
    Addr value_addr = getHsaSignalValueAddr(signal_handle);
    auto tc = system()->threads[0];
    ConstVPtr<Addr> prev_value(value_addr, tc);
    return *prev_value;
}

void
GPUCommandProcessor::updateHsaSignal(Addr signal_handle, uint64_t signal_value,
                                     HsaSignalCallbackFunction function)
{
    // The signal value is aligned 8 bytes from
    // the actual handle in the runtime
    Addr value_addr = getHsaSignalValueAddr(signal_handle);
    Addr mailbox_addr = getHsaSignalMailboxAddr(signal_handle);
    Addr event_addr = getHsaSignalEventAddr(signal_handle);
    DPRINTF(GPUCommandProc, "Triggering completion signal: %x!\n", value_addr);

    auto cb = new DmaVirtCallback<uint64_t>(function, signal_value);

    dmaWriteVirt(value_addr, sizeof(Addr), cb, &cb->dmaBuffer, 0);

    auto tc = system()->threads[0];
    ConstVPtr<uint64_t> mailbox_ptr(mailbox_addr, tc);

    // Notifying an event with its mailbox pointer is
    // not supported in the current implementation. Just use
    // mailbox pointer to distinguish between interruptible
    // and default signal. Interruptible signal will have
    // a valid mailbox pointer.
    if (*mailbox_ptr != 0) {
        // This is an interruptible signal. Now, read the
        // event ID and directly communicate with the driver
        // about that event notification.
        ConstVPtr<uint32_t> event_val(event_addr, tc);

        DPRINTF(GPUCommandProc, "Calling signal wakeup event on "
                "signal event value %d\n", *event_val);
        signalWakeupEvent(*event_val);
    }
}

void
GPUCommandProcessor::attachDriver(GPUComputeDriver *gpu_driver)
{
    fatal_if(_driver, "Should not overwrite driver.");
    // TODO: GPU Driver inheritance hierarchy doesn't really make sense.
    // Should get rid of the base class.
    _driver = gpu_driver;
    assert(_driver);
}

GPUComputeDriver*
GPUCommandProcessor::driver()
{
    return _driver;
}

/**
 * submitVendorPkt() is for accepting vendor-specific packets from
 * the HSAPP. Vendor-specific packets may be used by the runtime to
 * send commands to the HSA device that are specific to a particular
 * vendor. The vendor-specific packets should be defined by the vendor
 * in the runtime.
 */

/**
 * TODO: For now we simply tell the HSAPP to finish the packet,
 *       however a future patch will update this method to provide
 *       the proper handling of any required vendor-specific packets.
 *       In the version of ROCm that is currently supported (1.6)
 *       the runtime will send packets that direct the CP to
 *       invalidate the GPUs caches. We do this automatically on
 *       each kernel launch in the CU, so this is safe for now.
 */
void
GPUCommandProcessor::submitVendorPkt(void *raw_pkt, uint32_t queue_id,
    Addr host_pkt_addr)
{
    hsaPP->finishPkt(raw_pkt, queue_id);
}

/**
 * submitAgentDispatchPkt() is for accepting agent dispatch packets.
 * These packets will control the dispatch of Wg on the device, and inform
 * the host when a specified number of Wg have been executed on the device.
 *
 * For now it simply finishes the pkt.
 */
void
GPUCommandProcessor::submitAgentDispatchPkt(void *raw_pkt, uint32_t queue_id,
    Addr host_pkt_addr)
{
    //Parse the Packet, see what it wants us to do
    _hsa_agent_dispatch_packet_t * agent_pkt =
        (_hsa_agent_dispatch_packet_t *)raw_pkt;

    if (agent_pkt->type == AgentCmd::Nop) {
        DPRINTF(GPUCommandProc, "Agent Dispatch Packet NOP\n");
    } else if (agent_pkt->type == AgentCmd::Steal) {
        //This is where we steal the HSA Task's completion signal
        int kid = agent_pkt->arg[0];
        DPRINTF(GPUCommandProc,
            "Agent Dispatch Packet Stealing signal handle for kernel %d\n",
            kid);

        HSAQueueEntry *task = dispatcher.hsaTask(kid);
        uint64_t signal_addr = task->completionSignal();// + sizeof(uint64_t);

        uint64_t return_address = agent_pkt->return_address;
        DPRINTF(GPUCommandProc, "Return Addr: %p\n",return_address);
        //*return_address = signal_addr;
        Addr *new_signal_addr = new Addr;
        *new_signal_addr  = (Addr)signal_addr;
        dmaWriteVirt(return_address, sizeof(Addr), nullptr, new_signal_addr, 0);

        DPRINTF(GPUCommandProc,
            "Agent Dispatch Packet Stealing signal handle from kid %d :" \
            "(%x:%x) writing into %x\n",
            kid,signal_addr,new_signal_addr,return_address);

    } else
    {
        panic("The agent dispatch packet provided an unknown argument in" \
        "arg[0],currently only 0(nop) or 1(return kernel signal) is accepted");
    }

    hsaPP->finishPkt(raw_pkt, queue_id);
}

/**
 * Once the CP has finished extracting all relevant information about
 * a task and has initialized the ABI state, we send a description of
 * the task to the dispatcher. The dispatcher will create and dispatch
 * WGs to the CUs.
 */
void
GPUCommandProcessor::dispatchPkt(HSAQueueEntry *task)
{
    dispatcher.dispatch(task);
}

void
GPUCommandProcessor::signalWakeupEvent(uint32_t event_id)
{
    _driver->signalWakeupEvent(event_id);
}

/**
 * The CP is responsible for traversing all HSA-ABI-related data
 * structures from memory and initializing the ABI state.
 * Information provided by the MQD, AQL packet, and code object
 * metadata will be used to initialze register file state.
 */
void
GPUCommandProcessor::initABI(HSAQueueEntry *task)
{
    auto cb = new DmaVirtCallback<uint32_t>(
        [ = ] (const uint32_t &readDispIdOffset)
            { ReadDispIdOffsetDmaEvent(task, readDispIdOffset); }, 0);

    Addr hostReadIdxPtr
        = hsaPP->getQueueDesc(task->queueId())->hostReadIndexPtr;

    dmaReadVirt(hostReadIdxPtr + sizeof(hostReadIdxPtr),
        sizeof(uint32_t), cb, &cb->dmaBuffer);
}

System*
GPUCommandProcessor::system()
{
    return sys;
}

AddrRangeList
GPUCommandProcessor::getAddrRanges() const
{
    AddrRangeList ranges;
    return ranges;
}

void
GPUCommandProcessor::setShader(Shader *shader)
{
    _shader = shader;
}

Shader*
GPUCommandProcessor::shader()
{
    return _shader;
}

} // namespace gem5
