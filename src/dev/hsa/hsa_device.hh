/*
 * Copyright (c) 2015-2018 Advanced Micro Devices, Inc.
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
 * Authors: Eric van Tassell
 *          Anthony Gutierrez
 *          Sooraj Puthoor
 *          Michael LeBeane
 */

#ifndef __DEV_HSA_HSA_DEVICE_HH__
#define __DEV_HSA_HSA_DEVICE_HH__

#include "dev/dma_device.hh"
#include "dev/hsa/hsa_packet_processor.hh"
#include "params/HSADevice.hh"

class HSADevice : public DmaDevice
{
  public:
    typedef HSADeviceParams Params;

    HSADevice(const Params *p) : DmaDevice(p), hsaPP(p->hsapp)
    {
        assert(hsaPP);
        hsaPP->setDevice(this);
    };

    HSAPacketProcessor& hsaPacketProc();

    /**
     * submitDispatchPkt() accepts AQL dispatch packets from the HSA packet
     * processor. Not all devices will accept AQL dispatch packets, so the
     * default implementation will fatal.
     */
    virtual void
    submitDispatchPkt(void *raw_pkt, uint32_t qID, Addr host_pkt_addr)
    {
        fatal("%s does not accept dispatch packets\n", name());
    }

    /**
     * submitVendorPkt() accepts vendor specific packets from the HSA
     * packet processor. This method should be overriden in any HSADevice
     * that acceptes vendor specific packets, and should interpret the
     * packet according to the vendor's specifications. Not all HSA
     * devices will accept vendor specific packets, so the default
     * implementation will fatal.
     */
    virtual void
    submitVendorPkt(void *raw_pkt, uint32_t queue_id, Addr host_pkt_addr)
    {
        fatal("%s does not accept vendor specific packets\n", name());
    }

    void dmaReadVirt(Addr host_addr, unsigned size, DmaCallback *cb,
                     void *data, Tick delay = 0);
    void dmaWriteVirt(Addr host_addr, unsigned size, DmaCallback *cb,
                      void *data, Tick delay = 0);

  protected:
    // Typedefing dmaRead and dmaWrite function pointer
    typedef void (DmaDevice::*DmaFnPtr)(Addr, int, Event*, uint8_t*, Tick);
    HSAPacketProcessor *hsaPP;
    void dmaVirt(DmaFnPtr, Addr host_addr, unsigned size, DmaCallback *cb,
                 void *data, Tick delay = 0);
    void translateOrDie(Addr vaddr, Addr &paddr);
};

#endif // __DEV_HSA_HSA_DEVICE_HH__
