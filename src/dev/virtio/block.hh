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

#ifndef __DEV_VIRTIO_BLOCK_HH__
#define __DEV_VIRTIO_BLOCK_HH__

#include "base/compiler.hh"
#include "dev/storage/disk_image.hh"
#include "dev/virtio/base.hh"

namespace gem5
{

struct VirtIOBlockParams;

/**
 * VirtIO block device
 *
 * The block device uses the following queues:
 *  -# Requests
 *
 * A guest issues a request by creating a descriptor chain that starts
 * with a BlkRequest. Immediately after the BlkRequest follows the
 * data for the request. The data buffer(s) are either input or output
 * descriptors depending on the request type. The last byte in the
 * descriptor chain should always be writable by the host and contains
 * the request status code (OK/Error).
 *
 * The protocol supports asynchronous request completion by returning
 * descriptor chains when they have been populated by the backing
 * store. However, there is little point in doing so here.
 *
 * @see https://github.com/rustyrussell/virtio-spec
 * @see http://docs.oasis-open.org/virtio/virtio/v1.0/virtio-v1.0.html
 */
class VirtIOBlock : public VirtIODeviceBase
{
  public:
    typedef VirtIOBlockParams Params;
    VirtIOBlock(const Params &params);
    virtual ~VirtIOBlock();

    void readConfig(PacketPtr pkt, Addr cfgOffset);

  protected:
    static const DeviceId ID_BLOCK = 0x02;

    /**
     * Block device configuration structure
     *
     * @note This needs to be changed if the supported feature set
     * changes!
     */
    struct GEM5_PACKED Config
    {
        uint64_t capacity;
    };

    Config config;

    /** @{
     * @name Feature bits
     */
    static const FeatureBits F_SIZE_MAX = (1 << 1);
    static const FeatureBits F_SEG_MAX = (1 << 2);
    static const FeatureBits F_GEOMETRY = (1 << 4);
    static const FeatureBits F_RO = (1 << 5);
    static const FeatureBits F_BLK_SIZE = (1 << 6);
    static const FeatureBits F_TOPOLOGY = (1 << 10);
    /** @} */

    /** @{
     * @name VirtIO block requests
     */
    typedef uint32_t RequestType;
    /** Read request */
    static const RequestType T_IN = 0;
    /** Write request */
    static const RequestType T_OUT = 1;
    /** Flush device buffers */
    static const RequestType T_FLUSH = 4;
    /** @} */

    /** @{
     * @name VirtIO block request status
     */
    typedef uint8_t Status;
    /** Request succeeded */
    static const Status S_OK = 0;
    /** Request failed due to a device error */
    static const Status S_IOERR = 1;
    /** Request not supported */
    static const Status S_UNSUPP = 2;

    /** @} */

    /** VirtIO block device request as sent by guest */
    struct GEM5_PACKED BlkRequest
    {
        RequestType type;
        uint32_t reserved;
        uint64_t sector;
    };

    /**
     * Device read request.
     *
     * @param req Disk request from guest.
     * @param desc_chain Request descriptor chain (from start of
     *                   request)
     * @param off_data Offset into the descriptor chain where data
     *                 should be written.
     * @param size Request data size.
     */
    Status read(const BlkRequest &req, VirtDescriptor *desc_chain,
                size_t off_data, size_t size);
    /**
     * Device write request.
     *
     * @param req Disk request from guest.
     * @param desc_chain Request descriptor chain (from start of
     *                   request)
     * @param off_data Offset into the descriptor chain where data
     *                 should be read.
     * @param size Request data size.
     */
    Status write(const BlkRequest &req, VirtDescriptor *desc_chain,
                 size_t off_data, size_t size);

  protected:
    /**
     * Virtqueue for disk requests.
     */
    class RequestQueue : public VirtQueue
    {
      public:
        RequestQueue(PortProxy &proxy, ByteOrder bo, uint16_t size,
                     VirtIOBlock &_parent)
            : VirtQueue(proxy, bo, size), parent(_parent)
        {}

        virtual ~RequestQueue() {}

        void onNotifyDescriptor(VirtDescriptor *desc);

        std::string
        name() const
        {
            return parent.name() + ".qRequests";
        }

      protected:
        VirtIOBlock &parent;
    };

    /** Device I/O request queue */
    RequestQueue qRequests;

    /** Image backing this device */
    DiskImage &image;
};

} // namespace gem5

#endif // __DEV_VIRTIO_BLOCK_HH__
