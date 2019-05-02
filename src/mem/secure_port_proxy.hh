/*
 * Copyright (c) 2011-2013, 2018 ARM Limited
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
 * Authors: Andreas Hansson
 */

/**
 * @file
 * PortProxy Object Declaration.
 *
 * Port proxies are used when non-structural entities need access to
 * the memory system (or structural entities that want to peak into
 * the memory system without making a real memory access).
 *
 * Proxy objects replace the previous FunctionalPort, TranslatingPort
 * and VirtualPort objects, which provided the same functionality as
 * the proxies, but were instances of ports not corresponding to real
 * structural ports of the simulated system. Via the port proxies all
 * the accesses go through an actual port (either the system port,
 * e.g. for processes or initialisation, or a the data port of the
 * CPU, e.g. for threads) and thus are transparent to a potentially
 * distributed memory and automatically adhere to the memory map of
 * the system.
 */

#ifndef __MEM_SECURE_PORT_PROXY_HH__
#define __MEM_SECURE_PORT_PROXY_HH__

#include "mem/port_proxy.hh"

/**
 * This object is a proxy for a structural port, to be used for debug
 * accesses to secure memory.
 *
 * The addresses are interpreted as physical addresses to secure memory.
 */
class SecurePortProxy : public PortProxy
{
  public:
    SecurePortProxy(MasterPort &port, unsigned int cache_line_size)
        : PortProxy(port, cache_line_size) {}

    bool tryReadBlob(Addr addr, void *p, int size) const override;
    bool tryWriteBlob(Addr addr, const void *p, int size) const override;
    bool tryMemsetBlob(Addr addr, uint8_t val, int size) const override;
};

#endif // __MEM_SECURE_PORT_PROXY_HH__
