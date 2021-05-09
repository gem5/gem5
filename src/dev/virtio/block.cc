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

#include "dev/virtio/block.hh"

#include "debug/VIOBlock.hh"
#include "params/VirtIOBlock.hh"
#include "sim/system.hh"

namespace gem5
{

VirtIOBlock::VirtIOBlock(const Params &params)
    : VirtIODeviceBase(params, ID_BLOCK, sizeof(Config), 0),
      qRequests(params.system->physProxy, byteOrder,
                params.queueSize, *this),
      image(*params.image)
{
    registerQueue(qRequests);

    config.capacity = image.size();
}


VirtIOBlock::~VirtIOBlock()
{
}

void
VirtIOBlock::readConfig(PacketPtr pkt, Addr cfgOffset)
{
    Config cfg_out;
    cfg_out.capacity = htog(config.capacity, byteOrder);

    readConfigBlob(pkt, cfgOffset, (uint8_t *)&cfg_out);
}

VirtIOBlock::Status
VirtIOBlock::read(const BlkRequest &req, VirtDescriptor *desc_chain,
                  size_t off_data, size_t size)
{
    std::vector<uint8_t> data(size);
    uint64_t sector(req.sector);

    DPRINTF(VIOBlock, "Read request starting @ sector %i (size: %i)\n",
            sector, size);

    if (size % SectorSize != 0)
        panic("Unexpected request/sector size relationship\n");

    for (Addr offset = 0; offset < size; offset += SectorSize) {
        if (image.read(&data[offset], sector) != SectorSize) {
            warn("Failed to read sector %i\n", sector);
            return S_IOERR;
        }
        ++sector;
    }

    desc_chain->chainWrite(off_data, &data[0], size);

    return S_OK;
}

VirtIOBlock::Status
VirtIOBlock::write(const BlkRequest &req, VirtDescriptor *desc_chain,
                  size_t off_data, size_t size)
{
    std::vector<uint8_t> data(size);
    uint64_t sector(req.sector);

    DPRINTF(VIOBlock, "Write request starting @ sector %i (size: %i)\n",
            sector, size);

    if (size % SectorSize != 0)
        panic("Unexpected request/sector size relationship\n");


    desc_chain->chainRead(off_data, &data[0], size);

    for (Addr offset = 0; offset < size; offset += SectorSize) {
        if (image.write(&data[offset], sector) != SectorSize) {
            warn("Failed to write sector %i\n", sector);
            return S_IOERR;
        }
        ++sector;
    }

    return S_OK;

}

void
VirtIOBlock::RequestQueue::onNotifyDescriptor(VirtDescriptor *desc)
{
    DPRINTF(VIOBlock, "Got input data descriptor (len: %i)\n",
            desc->size());
    /*
     * Read the request structure and do endian conversion if
     * necessary.
     */
    BlkRequest req;
    desc->chainRead(0, (uint8_t *)&req, sizeof(req));
    req.type = htog(req.type, byteOrder);
    req.sector = htog(req.sector, byteOrder);

    Status status;
    const size_t data_size(desc->chainSize()
                           - sizeof(BlkRequest) - sizeof(Status));

    switch (req.type) {
      case T_IN:
        status = parent.read(req, desc, sizeof(BlkRequest), data_size);
        break;

      case T_OUT:
        status = parent.write(req, desc, sizeof(BlkRequest), data_size);
        break;

      case T_FLUSH:
        status = S_OK;
        break;

      default:
        warn("Unsupported IO request: %i\n", req.type);
        status = S_UNSUPP;
        break;
    }

    desc->chainWrite(sizeof(BlkRequest) + data_size,
                     &status, sizeof(status));

    // Tell the guest that we are done with this descriptor.
    produceDescriptor(desc, sizeof(BlkRequest) + data_size + sizeof(Status));
    parent.kick();
}

} // namespace gem5
