/*
 * Copyright (c) 2022  Institute of Computing Technology, Chinese Academy
 *                     of Sciences
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

#ifndef __DEV_VIRTIO_RNG_HH__
#define __DEV_VIRTIO_RNG_HH__

#include "base/compiler.hh"
#include "dev/virtio/base.hh"

namespace gem5
{

struct VirtIORngParams;

/**
 * VirtIO Rng
 *
 * @see https://github.com/rustyrussell/virtio-spec
 * @see http://docs.oasis-open.org/virtio/virtio/v1.0/virtio-v1.0.html
 */
class VirtIORng : public VirtIODeviceBase
{
  public:
    typedef VirtIORngParams Params;
    VirtIORng(const Params &params);
    virtual ~VirtIORng();

    void readConfig(PacketPtr pkt, Addr cfgOffset);

  protected:
    /** VirtIO device ID */
    static const DeviceId ID_RNG = 0x04;

  protected:
    /**
     * Virtqueue for data going from the host to the guest.
     */
    class RngQueue
        : public VirtQueue
    {
      public:
        RngQueue(PortProxy &proxy, ByteOrder bo, uint16_t size,
                 VirtIORng &_parent);
        virtual ~RngQueue() {}

        void onNotify() { trySend(); }

        /** Try to send data pending data from the terminal. */
        void trySend();

        std::string name() const { return parent.name() + ".qRecv"; }

      protected:
        VirtIORng &parent;
    };
    /** Receive queue for port 0 */
    RngQueue qReq;
};

} // namespace gem5

#endif // __DEV_VIRTIO_RNG_HH__
