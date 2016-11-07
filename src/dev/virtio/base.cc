/*
 * Copyright (c) 2014, 2016 ARM Limited
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
 * Authors: Andreas Sandberg
 */

#include "dev/virtio/base.hh"

#include "debug/VIO.hh"
#include "params/VirtIODeviceBase.hh"
#include "params/VirtIODummyDevice.hh"

VirtDescriptor::VirtDescriptor(PortProxy &_memProxy, VirtQueue &_queue,
                               Index descIndex)
    : memProxy(&_memProxy), queue(&_queue), _index(descIndex),
      desc{0, 0, 0, 0}
{
}

VirtDescriptor::VirtDescriptor(VirtDescriptor &&other) noexcept
{
    *this = std::forward<VirtDescriptor>(other);
}

VirtDescriptor::~VirtDescriptor() noexcept
{
}

VirtDescriptor &
VirtDescriptor::operator=(VirtDescriptor &&rhs) noexcept
{
    memProxy = std::move(rhs.memProxy);
    queue = std::move(rhs.queue);
    _index = std::move(rhs._index);
    desc = std::move(rhs.desc);

    return *this;
}

void
VirtDescriptor::update()
{
    const Addr vq_addr(queue->getAddress());
    // Check if the queue has been initialized yet
    if (vq_addr == 0)
        return;

    assert(_index < queue->getSize());
    const Addr desc_addr(vq_addr + sizeof(desc) * _index);
    vring_desc guest_desc;
    memProxy->readBlob(desc_addr, (uint8_t *)&guest_desc, sizeof(guest_desc));
    desc = vtoh_legacy(guest_desc);
    DPRINTF(VIO,
            "VirtDescriptor(%i): Addr: 0x%x, Len: %i, Flags: 0x%x, "
            "Next: 0x%x\n",
            _index, desc.addr, desc.len, desc.flags, desc.next);
}

void
VirtDescriptor::updateChain()
{
    VirtDescriptor *desc(this);
    do {
        desc->update();
    } while ((desc = desc->next()) != NULL && desc != this);

    if (desc == this)
        panic("Loop in descriptor chain!\n");
}

void
VirtDescriptor::dump() const
{
    if (!DTRACE(VIO))
        return;

    DPRINTF(VIO, "Descriptor[%i]: "
            "Addr: 0x%x, Len: %i, Flags: 0x%x, Next: 0x%x\n",
            _index, desc.addr, desc.len, desc.flags, desc.next);

    if (isIncoming()) {
        uint8_t data[desc.len];
        read(0, data, desc.len);
        DDUMP(VIO, data, desc.len);
    }
}

void
VirtDescriptor::dumpChain() const
{
    if (!DTRACE(VIO))
        return;

    const VirtDescriptor *desc(this);
    do {
        desc->dump();
    } while ((desc = desc->next()) != NULL);
}

VirtDescriptor *
VirtDescriptor::next() const
{
    if (hasNext()) {
        return queue->getDescriptor(desc.next);
    } else {
        return NULL;
    }
}

void
VirtDescriptor::read(size_t offset, uint8_t *dst, size_t size) const
{
    DPRINTF(VIO, "VirtDescriptor(%p, 0x%x, %i)::read: offset: %i, dst: 0x%x, size: %i\n",
            this, desc.addr, desc.len, offset, (long)dst, size);
    assert(size <= desc.len - offset);
    if (!isIncoming())
        panic("Trying to read from outgoing buffer\n");

    memProxy->readBlob(desc.addr + offset, dst, size);
}

void
VirtDescriptor::write(size_t offset, const uint8_t *src, size_t size)
{
    DPRINTF(VIO, "VirtDescriptor(%p, 0x%x, %i)::write: offset: %i, src: 0x%x, size: %i\n",
            this, desc.addr, desc.len, offset, (long)src, size);
    assert(size <= desc.len - offset);
    if (!isOutgoing())
        panic("Trying to write to incoming buffer\n");

    memProxy->writeBlob(desc.addr + offset, const_cast<uint8_t *>(src), size);
}

void
VirtDescriptor::chainRead(size_t offset, uint8_t *dst, size_t size) const
{
    const VirtDescriptor *desc(this);
    const size_t full_size(size);
    do {
        if (offset < desc->size()) {
            const size_t chunk_size(std::min(desc->size() - offset, size));
            desc->read(offset, dst, chunk_size);
            dst += chunk_size;
            size -= chunk_size;
            offset = 0;
        } else {
            offset -= desc->size();
        }
    } while ((desc = desc->next()) != NULL && desc->isIncoming() && size > 0);

    if (size != 0) {
        panic("Failed to read %i bytes from chain of %i bytes @ offset %i\n",
              full_size, chainSize(), offset);
    }
}

void
VirtDescriptor::chainWrite(size_t offset, const uint8_t *src, size_t size)
{
    VirtDescriptor *desc(this);
    const size_t full_size(size);
    do {
        if (offset < desc->size()) {
            const size_t chunk_size(std::min(desc->size() - offset, size));
            desc->write(offset, src, chunk_size);
            src += chunk_size;
            size -= chunk_size;
            offset = 0;
        } else {
            offset -= desc->size();
        }
    } while ((desc = desc->next()) != NULL && size > 0);

    if (size != 0) {
        panic("Failed to write %i bytes into chain of %i bytes @ offset %i\n",
              full_size, chainSize(), offset);
    }
}

size_t
VirtDescriptor::chainSize() const
{
    size_t size(0);
    const VirtDescriptor *desc(this);
    do {
        size += desc->size();
    } while ((desc = desc->next()) != NULL);

    return size;
}



VirtQueue::VirtQueue(PortProxy &proxy, uint16_t size)
    : _size(size), _address(0), memProxy(proxy),
      avail(proxy, size), used(proxy, size),
      _last_avail(0)
{
    descriptors.reserve(_size);
    for (int i = 0; i < _size; ++i)
        descriptors.emplace_back(proxy, *this, i);
}

void
VirtQueue::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(_address);
    SERIALIZE_SCALAR(_last_avail);
}

void
VirtQueue::unserialize(CheckpointIn &cp)
{
    Addr addr_in;

    paramIn(cp, "_address", addr_in);
    UNSERIALIZE_SCALAR(_last_avail);

    // Use the address setter to ensure that the ring buffer addresses
    // are updated as well.
    setAddress(addr_in);
}

void
VirtQueue::setAddress(Addr address)
{
    const Addr addr_avail(address + _size * sizeof(struct vring_desc));
    const Addr addr_avail_end(addr_avail + sizeof(struct vring_avail) +
                              _size * sizeof(uint16_t));
    const Addr addr_used((addr_avail_end + sizeof(uint16_t) +
                          (ALIGN_SIZE - 1)) & ~(ALIGN_SIZE - 1));
    _address = address;
    avail.setAddress(addr_avail);
    used.setAddress(addr_used);
}

VirtDescriptor *
VirtQueue::consumeDescriptor()
{
    avail.read();
    DPRINTF(VIO, "consumeDescriptor: _last_avail: %i, avail.idx: %i (->%i)\n",
            _last_avail, avail.header.index,
            avail.ring[_last_avail % used.ring.size()]);
    if (_last_avail == avail.header.index)
        return NULL;

    VirtDescriptor::Index index(avail.ring[_last_avail % used.ring.size()]);
    ++_last_avail;

    VirtDescriptor *d(&descriptors[index]);
    d->updateChain();

    return d;
}

void
VirtQueue::produceDescriptor(VirtDescriptor *desc, uint32_t len)
{
    used.readHeader();
    DPRINTF(VIO, "produceDescriptor: dscIdx: %i, len: %i, used.idx: %i\n",
            desc->index(), len, used.header.index);

    struct vring_used_elem &e(used.ring[used.header.index % used.ring.size()]);
    e.id = desc->index();
    e.len = len;
    used.header.index += 1;
    used.write();
}

void
VirtQueue::dump() const
{
    if (!DTRACE(VIO))
        return;

    for (const VirtDescriptor &d : descriptors)
        d.dump();
}

void
VirtQueue::onNotify()
{
    DPRINTF(VIO, "onNotify\n");

    // Consume all pending descriptors from the input queue.
    VirtDescriptor *d;
    while ((d = consumeDescriptor()) != NULL)
        onNotifyDescriptor(d);
}


VirtIODeviceBase::VirtIODeviceBase(Params *params, DeviceId id,
                                   size_t config_size, FeatureBits features)
    : SimObject(params),
      guestFeatures(0),
      deviceId(id), configSize(config_size), deviceFeatures(features),
      _deviceStatus(0), _queueSelect(0),
      transKick(NULL)
{
}


VirtIODeviceBase::~VirtIODeviceBase()
{
}

void
VirtIODeviceBase::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(guestFeatures);
    SERIALIZE_SCALAR(_deviceStatus);
    SERIALIZE_SCALAR(_queueSelect);
    for (QueueID i = 0; i < _queues.size(); ++i)
        _queues[i]->serializeSection(cp, csprintf("_queues.%i", i));
}

void
VirtIODeviceBase::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(guestFeatures);
    UNSERIALIZE_SCALAR(_deviceStatus);
    UNSERIALIZE_SCALAR(_queueSelect);
    for (QueueID i = 0; i < _queues.size(); ++i)
        _queues[i]->unserializeSection(cp, csprintf("_queues.%i", i));
}

void
VirtIODeviceBase::reset()
{
    _queueSelect = 0;
    guestFeatures = 0;
    _deviceStatus = 0;

    for (QueueID i = 0; i < _queues.size(); ++i)
        _queues[i]->setAddress(0);
}

void
VirtIODeviceBase::onNotify(QueueID idx)
{
    DPRINTF(VIO, "onNotify: idx: %i\n", idx);
    if (idx >= _queues.size()) {
        panic("Guest tried to notify queue (%i), but only %i "
              "queues registered.\n",
              idx, _queues.size());
    }
    _queues[idx]->onNotify();
}

void
VirtIODeviceBase::setGuestFeatures(FeatureBits features)
{
    DPRINTF(VIO, "Setting guest features: 0x%x\n", features);
    if (~deviceFeatures & features) {
        panic("Guest tried to enable unsupported features:\n"
              "Device features: 0x%x\n"
              "Requested features: 0x%x\n",
              deviceFeatures, features);
    }
    guestFeatures = features;
}


void
VirtIODeviceBase::setDeviceStatus(DeviceStatus status)
{
    _deviceStatus = status;
    DPRINTF(VIO, "ACK: %i, DRIVER: %i, DRIVER_OK: %i, FAILED: %i\n",
            status.acknowledge, status.driver, status.driver_ok, status.failed);
    if (status == 0)
        reset();
}

void
VirtIODeviceBase::readConfig(PacketPtr pkt, Addr cfgOffset)
{
    panic("Unhandled device config read (offset: 0x%x).\n", cfgOffset);
}

void
VirtIODeviceBase::writeConfig(PacketPtr pkt, Addr cfgOffset)
{
    panic("Unhandled device config write (offset: 0x%x).\n", cfgOffset);
}

void
VirtIODeviceBase::readConfigBlob(PacketPtr pkt, Addr cfgOffset, const uint8_t *cfg)
{
    const unsigned size(pkt->getSize());

    if (cfgOffset + size > configSize)
        panic("Config read out of bounds.\n");

    pkt->makeResponse();
    pkt->setData(const_cast<uint8_t *>(cfg) + cfgOffset);
}

void
VirtIODeviceBase::writeConfigBlob(PacketPtr pkt, Addr cfgOffset, uint8_t *cfg)
{
    const unsigned size(pkt->getSize());

    if (cfgOffset + size > configSize)
        panic("Config write out of bounds.\n");

    pkt->makeResponse();
    pkt->writeData((uint8_t *)cfg + cfgOffset);
}


const VirtQueue &
VirtIODeviceBase::getCurrentQueue() const
{
    if (_queueSelect >= _queues.size())
        panic("Guest tried to access non-existing VirtQueue (%i).\n", _queueSelect);

    return *_queues[_queueSelect];
}

VirtQueue &
VirtIODeviceBase::getCurrentQueue()
{
    if (_queueSelect >= _queues.size())
        panic("Guest tried to access non-existing VirtQueue (%i).\n", _queueSelect);

    return *_queues[_queueSelect];
}

void
VirtIODeviceBase::setQueueAddress(uint32_t address)
{
    getCurrentQueue().setAddress(address * VirtQueue::ALIGN_SIZE);
}

uint32_t
VirtIODeviceBase::getQueueAddress() const
{
    Addr address(getCurrentQueue().getAddress());
    assert(!(address & ((1 >> VirtQueue::ALIGN_BITS) - 1)));
    return address >> VirtQueue::ALIGN_BITS;
}

void
VirtIODeviceBase::registerQueue(VirtQueue &queue)
{
    _queues.push_back(&queue);
}


VirtIODummyDevice::VirtIODummyDevice(VirtIODummyDeviceParams *params)
    : VirtIODeviceBase(params, ID_INVALID, 0, 0)
{
}

VirtIODummyDevice *
VirtIODummyDeviceParams::create()
{
    return new VirtIODummyDevice(this);
}
