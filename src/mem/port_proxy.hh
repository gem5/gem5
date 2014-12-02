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

#ifndef __MEM_PORT_PROXY_HH__
#define __MEM_PORT_PROXY_HH__

#include "config/the_isa.hh"
#if THE_ISA != NULL_ISA
    #include "arch/isa_traits.hh"
#endif

#include "mem/port.hh"
#include "sim/byteswap.hh"

/**
 * This object is a proxy for a structural port, to be used for debug
 * accesses.
 *
 * This proxy object is used when non structural entities
 * (e.g. thread contexts, object file loaders) need access to the
 * memory system. It calls the corresponding functions on the underlying
 * structural port, and provides templatized convenience access functions.
 *
 * The addresses are interpreted as physical addresses.
 *
 * @sa SETranslatingProxy
 * @sa FSTranslatingProxy
 */
class PortProxy
{
  private:

    /** The actual physical port used by this proxy. */
    MasterPort &_port;

    /** Granularity of any transactions issued through this proxy. */
    const unsigned int _cacheLineSize;

  public:
    PortProxy(MasterPort &port, unsigned int cacheLineSize) :
        _port(port), _cacheLineSize(cacheLineSize) { }
    virtual ~PortProxy() { }

    /**
     * Read size bytes memory at address and store in p.
     */
    virtual void readBlob(Addr addr, uint8_t* p, int size) const;

    /**
     * Write size bytes from p to address.
     */
    virtual void writeBlob(Addr addr, const uint8_t* p, int size) const;

    /**
     * Fill size bytes starting at addr with byte value val.
     */
    virtual void memsetBlob(Addr addr, uint8_t v, int size) const;

    /**
     * Read sizeof(T) bytes from address and return as object T.
     */
    template <typename T>
    T read(Addr address) const;

    /**
     * Write object T to address. Writes sizeof(T) bytes.
     */
    template <typename T>
    void write(Addr address, T data) const;

#if THE_ISA != NULL_ISA
    /**
     * Read sizeof(T) bytes from address and return as object T.
     * Performs Guest to Host endianness transform.
     */
    template <typename T>
    T readGtoH(Addr address) const;

    /**
     * Write object T to address. Writes sizeof(T) bytes.
     * Performs Host to Guest endianness transform.
     */
    template <typename T>
    void writeHtoG(Addr address, T data) const;
#endif
};


template <typename T>
T
PortProxy::read(Addr address) const
{
    T data;
    readBlob(address, (uint8_t*)&data, sizeof(T));
    return data;
}

template <typename T>
void
PortProxy::write(Addr address, T data) const
{
    writeBlob(address, (uint8_t*)&data, sizeof(T));
}

#if THE_ISA != NULL_ISA
template <typename T>
T
PortProxy::readGtoH(Addr address) const
{
    T data;
    readBlob(address, (uint8_t*)&data, sizeof(T));
    return TheISA::gtoh(data);
}

template <typename T>
void
PortProxy::writeHtoG(Addr address, T data) const
{
    data = TheISA::htog(data);
    writeBlob(address, (uint8_t*)&data, sizeof(T));
}
#endif

#endif // __MEM_PORT_PROXY_HH__
