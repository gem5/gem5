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

#include "gpu-compute/gpu_command_processor.hh"

#include "debug/GPUCommandProc.hh"
#include "debug/GPUKernelInfo.hh"
#include "gpu-compute/dispatcher.hh"
#include "params/GPUCommandProcessor.hh"

GPUCommandProcessor::GPUCommandProcessor(const Params *p)
    : HSADevice(p), dispatcher(*p->dispatcher)
{
    dispatcher.setCommandProcessor(this);
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
    auto &virt_proxy = tc->getVirtProxy();

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

    Addr machine_code_addr = (Addr)disp_pkt->kernel_object
        + akc.kernel_code_entry_byte_offset;

    DPRINTF(GPUCommandProc, "Machine code starts at addr: %#x\n",
        machine_code_addr);

    Addr kern_name_addr(0);
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
        virt_proxy.readBlob(akc.runtime_loader_kernel_symbol + 0x10,
            (uint8_t*)&kern_name_addr, 0x8);

        virt_proxy.readString(kernel_name, kern_name_addr);
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

/**
 * The CP is responsible for traversing all HSA-ABI-related data
 * structures from memory and initializing the ABI state.
 * Information provided by the MQD, AQL packet, and code object
 * metadata will be used to initialze register file state.
 */
void
GPUCommandProcessor::initABI(HSAQueueEntry *task)
{
    auto *readDispIdOffEvent = new ReadDispIdOffsetDmaEvent(*this, task);

    Addr hostReadIdxPtr
        = hsaPP->getQueueDesc(task->queueId())->hostReadIndexPtr;

    dmaReadVirt(hostReadIdxPtr + sizeof(hostReadIdxPtr),
        sizeof(readDispIdOffEvent->readDispIdOffset), readDispIdOffEvent,
            &readDispIdOffEvent->readDispIdOffset);
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

GPUCommandProcessor*
GPUCommandProcessorParams::create()
{
    return new GPUCommandProcessor(this);
}
