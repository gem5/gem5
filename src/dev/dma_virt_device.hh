/*
 * Copyright (c) 2021 Advanced Micro Devices, Inc.
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

#ifndef __DEV_DMA_VIRT_DEVICE_HH__
#define __DEV_DMA_VIRT_DEVICE_HH__

#include "dev/dma_device.hh"
#include "mem/translation_gen.hh"

namespace gem5
{

class DmaVirtDevice : public DmaDevice
{
  protected:
    /**
     * Wraps a std::function object in a DmaCallback.  Much cleaner than
     * defining a bunch of callback objects for each desired behavior when a
     * DMA completes.  Contains a built in templated buffer that can be used
     * for DMA temporary storage.
     */
    template <class T>
    class DmaVirtCallback : public DmaCallback
    {
        std::function<void(const T &)> _function;

        virtual void
        process() override
        {
            _function(dmaBuffer);
        }

      public:
        T dmaBuffer;

        DmaVirtCallback(const std::function<void(const T &)> &function,
                        T dma_buffer_value = 0)
            : DmaCallback(), _function(function), dmaBuffer(dma_buffer_value)
        {}
    };

  public:
    DmaVirtDevice(const Params &p) : DmaDevice(p) {}

    virtual ~DmaVirtDevice() {}

    /**
     * Initiate a DMA read from virtual address host_addr. Helper function
     * for dmaVirt method.
     *
     * @param host_addr Virtual starting address for DMA transfer
     * @param size Number of bytes to transfer
     * @param cb DmaCallback to call upon completition of transfer
     * @param data Pointer to the data to be transfered
     * @param delay Number of ticks to wait before scheduling callback
     */
    void dmaReadVirt(Addr host_addr, unsigned size, DmaCallback *cb,
                     void *data, Tick delay = 0);
    /**
     * Initiate a DMA write from virtual address host_addr. Helper function
     * for dmaVirt method.
     *
     * @param host_addr Virtual starting address for DMA transfer
     * @param size Number of bytes to transfer
     * @param cb DmaCallback to call upon completition of transfer
     * @param data Pointer to the data to be transfered
     * @param delay Number of ticks to wait before scheduling callback
     */
    void dmaWriteVirt(Addr host_addr, unsigned size, DmaCallback *b,
                      void *data, Tick delay = 0);

    // Typedefing dmaRead and dmaWrite function pointer
    typedef void (DmaDevice::*DmaFnPtr)(Addr, int, Event *, uint8_t *, Tick);

    /**
     * Initiate a call to DmaDevice using DmaFnPtr do a DMA starting from
     * virtual address host_addr for size number of bytes on the data. Upon
     * completion the DmaCallback cb is called if not nullptr.
     *
     * @param dmaFn Method in DmaDevice to call per transfer chunk
     * @param host_addr Virtual starting address for DMA transfer
     * @param size Number of bytes to transfer
     * @param cb DmaCallback to call upon completition of transfer
     * @param data Pointer to the data to be transfered
     * @param delay Number of ticks to wait before scheduling callback
     */
    void dmaVirt(DmaFnPtr dmaFn, Addr host_addr, unsigned size,
                 DmaCallback *cb, void *data, Tick delay = 0);

    /**
     * Function used to translate a range of addresses from virtual to
     * physical addresses. All classes inheriting from DmaVirtDevice must
     * define this.
     *
     * @param vaddr Virtual address of the start of the range
     * @param size Size of the range in bytes
     * @return A translation generator for this range
     */
    virtual TranslationGenPtr translate(Addr vaddr, Addr size) = 0;
};

} // namespace gem5

#endif // __DEV_DMA_VIRT_DEVICE_HH__
