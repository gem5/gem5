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

#include <functional>
#include <limits>

#include "mem/protocol/functional.hh"
#include "sim/byteswap.hh"

namespace gem5
{

class RequestPort;
class ThreadContext;

/**
 * This object is a proxy for a port or other object which implements the
 * functional response protocol, to be used for debug accesses.
 *
 * This proxy object is used when non structural entities
 * (e.g. thread contexts, object file loaders) need access to the
 * memory system. It calls the corresponding functions on the underlying
 * protocol, and provides templatized convenience access functions.
 *
 * The addresses are interpreted as physical addresses.
 *
 * @sa SETranslatingProxy
 * @sa FSTranslatingProxy
 */
class PortProxy : FunctionalRequestProtocol
{
  public:
    typedef std::function<void(PacketPtr pkt)> SendFunctionalFunc;

  private:
    SendFunctionalFunc sendFunctional;

    /** Granularity of any transactions issued through this proxy. */
    const Addr _cacheLineSize;

    void
    recvFunctionalSnoop(PacketPtr pkt) override
    {
        // Since port proxies aren't anyone else's peer, they should never
        // receive snoops.
        panic("Port proxies should never receive snoops.");
    }

  public:
    PortProxy(SendFunctionalFunc func, Addr cache_line_size)
        : sendFunctional(func), _cacheLineSize(cache_line_size)
    {}

    // Helpers which create typical SendFunctionalFunc-s from other objects.
    PortProxy(ThreadContext *tc, Addr cache_line_size);
    PortProxy(const RequestPort &port, Addr cache_line_size);

    virtual ~PortProxy() {}

    /** Fixed functionality for use in base classes. */

    /**
     * Read size bytes memory at physical address and store in p.
     */
    void readBlobPhys(Addr addr, Request::Flags flags, void *p,
                      uint64_t size) const;

    /**
     * Write size bytes from p to physical address.
     */
    void writeBlobPhys(Addr addr, Request::Flags flags, const void *p,
                       uint64_t size) const;

    /**
     * Fill size bytes starting at physical addr with byte value val.
     */
    void memsetBlobPhys(Addr addr, Request::Flags flags, uint8_t v,
                        uint64_t size) const;

    /** Methods to override in base classes */

    /**
     * Read size bytes memory at address and store in p.
     * Returns true on success and false on failure.
     */
    virtual bool
    tryReadBlob(Addr addr, void *p, uint64_t size) const
    {
        readBlobPhys(addr, 0, p, size);
        return true;
    }

    /**
     * Write size bytes from p to address.
     * Returns true on success and false on failure.
     */
    virtual bool
    tryWriteBlob(Addr addr, const void *p, uint64_t size) const
    {
        writeBlobPhys(addr, 0, p, size);
        return true;
    }

    /**
     * Fill size bytes starting at addr with byte value val.
     * Returns true on success and false on failure.
     */
    virtual bool
    tryMemsetBlob(Addr addr, uint8_t val, uint64_t size) const
    {
        memsetBlobPhys(addr, 0, val, size);
        return true;
    }

    /** Higher level interfaces based on the above. */

    /**
     * Same as tryReadBlob, but insists on success.
     */
    void
    readBlob(Addr addr, void *p, uint64_t size) const
    {
        if (!tryReadBlob(addr, p, size))
            fatal("readBlob(%#x, ...) failed", addr);
    }

    /**
     * Same as tryWriteBlob, but insists on success.
     */
    void
    writeBlob(Addr addr, const void *p, uint64_t size) const
    {
        if (!tryWriteBlob(addr, p, size))
            fatal("writeBlob(%#x, ...) failed", addr);
    }

    /**
     * Same as tryMemsetBlob, but insists on success.
     */
    void
    memsetBlob(Addr addr, uint8_t v, uint64_t size) const
    {
        if (!tryMemsetBlob(addr, v, size))
            fatal("memsetBlob(%#x, ...) failed", addr);
    }

    /**
     * Read sizeof(T) bytes from address and return as object T.
     */
    template <typename T>
    T read(Addr address) const;

    /**
     * Write object T to address. Writes sizeof(T) bytes.
     */
    template <typename T>
    void write(Addr address, const T &data) const;

    /**
     * Read sizeof(T) bytes from address and return as object T.
     * Performs endianness conversion from the selected guest to host order.
     */
    template <typename T>
    T read(Addr address, ByteOrder guest_byte_order) const;

    /**
     * Write object T to address. Writes sizeof(T) bytes.
     * Performs endianness conversion from host to the selected guest order.
     */
    template <typename T>
    void write(Addr address, T data, ByteOrder guest_byte_order) const;

    /**
     * Write the string str into guest memory at address addr.
     * Returns true on success and false on failure.
     */
    bool tryWriteString(Addr addr, const char *str) const;

    /**
     * Same as tryWriteString, but insists on success.
     */
    void
    writeString(Addr addr, const char *str) const
    {
        if (!tryWriteString(addr, str))
            fatal("writeString(%#x, ...) failed", addr);
    }

    /**
     * Reads the string at guest address addr into the std::string str.
     * Returns true on success and false on failure.
     */
    bool tryReadString(std::string &str, Addr addr) const;

    /**
     * Same as tryReadString, but insists on success.
     */
    void
    readString(std::string &str, Addr addr) const
    {
        if (!tryReadString(str, addr))
            fatal("readString(%#x, ...) failed", addr);
    }

    /**
     * Reads the string at guest address addr into the char * str, reading up
     * to maxlen characters. The last character read is always a nul
     * terminator. Returns true on success and false on failure.
     */
    bool tryReadString(char *str, Addr addr, size_t maxlen) const;

    /**
     * Same as tryReadString, but insists on success.
     */
    void
    readString(char *str, Addr addr, size_t maxlen) const
    {
        if (!tryReadString(str, addr, maxlen))
            fatal("readString(%#x, ...) failed", addr);
    }
};

template <typename T>
T
PortProxy::read(Addr address) const
{
    T data;
    readBlob(address, &data, sizeof(T));
    return data;
}

template <typename T>
void
PortProxy::write(Addr address, const T &data) const
{
    writeBlob(address, &data, sizeof(T));
}

template <typename T>
T
PortProxy::read(Addr address, ByteOrder byte_order) const
{
    T data;
    readBlob(address, &data, sizeof(T));
    return gtoh(data, byte_order);
}

template <typename T>
void
PortProxy::write(Addr address, T data, ByteOrder byte_order) const
{
    data = htog(data, byte_order);
    writeBlob(address, &data, sizeof(T));
}

} // namespace gem5

#endif // __MEM_PORT_PROXY_HH__
