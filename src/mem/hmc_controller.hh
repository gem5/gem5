/*
 * Copyright (c) 2011-2013 ARM Limited
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
 * Copyright (c) 2015 The University of Bologna
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


/**
 * @file
 * HMCController declaration
 */

#ifndef __MEM_HMC_CONTROLLER_HH__
#define __MEM_HMC_CONTROLLER_HH__

#include "mem/noncoherent_xbar.hh"
#include "mem/port.hh"
#include "params/HMCController.hh"

/**
 * HMC Controller, in general, is responsible for translating the host
 * protocol (AXI for example) to serial links protocol. Plus, it should have
 * large internal buffers to hide the access latency of the cube. It is also
 * inferred from the standard [1] and the literature [2] that serial links
 * share the same address range and packets can travel over any of them, so a
 * load distribution mechanism is required.
 * This model simply queues the incoming transactions (using a Bridge) and
 * schedules them to the serial links using a simple round robin mechanism to
 * balance the load among them. More advanced global scheduling policies and
 * reordering and steering of transactions can be added to this model if
 * required [3].
 * [1] http://www.hybridmemorycube.org/specification-download/
 * [2] Low-Power Hybrid Memory Cubes With Link Power Manageme and Two-Level
 * Prefetching (J. Ahn et. al)
 * [3] The Open-Silicon HMC Controller IP
 * http://www.open-silicon.com/open-silicon-ips/hmc/
 */

class HMCController : public NoncoherentXBar
{
public:

    HMCController(const HMCControllerParams *p);

private:

    // Receive range change only on one of the ports (because they all have
    //  the same range)
    virtual void recvRangeChange(PortID mem_side_port_id);

    // Receive a request and distribute it among response ports
    //  Simply forwards the packet to the next serial link based on a
    //  Round-robin counter
    virtual bool recvTimingReq(PacketPtr pkt, PortID cpu_side_port_id);

    int numMemSidePorts;

    // The round-robin counter
    int rr_counter;
    /**
     * Function for rotating the round robin counter
     * @return the next value of the counter
     */
    int rotate_counter();
};

#endif //__MEM_HMC_CONTROLLER_HH__
