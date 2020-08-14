/*
 * Copyright (c) 2014 ARM Limited
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

#ifndef __DEV_VIRTIO_CONSOLE_HH__
#define __DEV_VIRTIO_CONSOLE_HH__

#include "dev/serial/serial.hh"
#include "dev/virtio/base.hh"

struct VirtIOConsoleParams;

/**
 * VirtIO console
 *
 * The VirtIO console uses the following queues:
 *  -# Port0 receive (from host to guest)
 *  -# Port0 transmit (from guest to host)
 *  -# Control receive (optional)
 *  -# Control transmit (optional)
 *  -# Port1 receive (optional)
 *  -# Port1 transmit (optional)
 *
 * This implementation implements the minimum of features for one
 * port, which means that the multiport feature is not announced and
 * only queue 0 and 1 are available.
 *
 * @see https://github.com/rustyrussell/virtio-spec
 * @see http://docs.oasis-open.org/virtio/virtio/v1.0/virtio-v1.0.html
 */
class VirtIOConsole : public VirtIODeviceBase
{
  public:
    typedef VirtIOConsoleParams Params;
    VirtIOConsole(Params *params);
    virtual ~VirtIOConsole();

    void readConfig(PacketPtr pkt, Addr cfgOffset);

  protected:
    /**
     * Console configuration structure
     *
     * @note This needs to be changed if the multiport feature is
     * announced!
     */
    struct Config {
        uint16_t cols;
        uint16_t rows;
    } M5_ATTR_PACKED;

    /** Currently active configuration (host byte order) */
    Config config;

    /** VirtIO device ID */
    static const DeviceId ID_CONSOLE = 0x03;

    /** @{
     * @name Feature bits
     */
    /** Provides the size information */
    static const FeatureBits F_SIZE = 0x01;
    /** Supports the multi-port interface */
    static const FeatureBits F_MULTIPORT = 0x02;
    /** @} */


  protected:
    /**
     * Virtqueue for data going from the host to the guest.
     */
    class TermRecvQueue
        : public VirtQueue
    {
      public:
        TermRecvQueue(PortProxy &proxy, ByteOrder bo,
                uint16_t size, VirtIOConsole &_parent)
            : VirtQueue(proxy, bo, size), parent(_parent) {}
        virtual ~TermRecvQueue() {}

        void onNotify() { trySend(); }

        /** Try to send data pending data from the terminal. */
        void trySend();

        std::string name() const { return parent.name() + ".qRecv"; }

      protected:
        VirtIOConsole &parent;
    };
    /** Receive queue for port 0 */
    TermRecvQueue qRecv;

    /**
     * Virtqueue for data going from the guest to the host.
     */
    class TermTransQueue
        : public VirtQueue
    {
      public:
        TermTransQueue(PortProxy &proxy, ByteOrder bo,
                uint16_t size, VirtIOConsole &_parent)
            : VirtQueue(proxy, bo, size), parent(_parent) {}
        virtual ~TermTransQueue() {}

        void onNotifyDescriptor(VirtDescriptor *desc);

        std::string name() const { return parent.name() + ".qTrans"; }

      protected:
        VirtIOConsole &parent;
    };
    /** Transmit queue for port 0 */
    TermTransQueue qTrans;

  protected:
    SerialDevice &device;
};

#endif // __DEV_VIRTIO_CONSOLE_HH__
