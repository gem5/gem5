/*
 * Copyright (c) 2014, 2016-2017 ARM Limited
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

#ifndef __DEV_VIRTIO_BASE_HH__
#define __DEV_VIRTIO_BASE_HH__

#include "arch/isa_traits.hh"
#include "base/bitunion.hh"
#include "base/callback.hh"
#include "dev/virtio/virtio_ring.h"
#include "mem/port_proxy.hh"
#include "sim/sim_object.hh"

struct VirtIODeviceBaseParams;
struct VirtIODummyDeviceParams;

class VirtQueue;

/** @{
 * @name VirtIO endian conversion helpers
 *
 * VirtIO prior to version 1.0 (legacy versions) normally send values
 * to the host in the guest systems native byte order. This is going
 * to change in version 1.0 which mandates little endian. We currently
 * only support the legacy version of VirtIO (the new and shiny
 * standard is still in a draft state and not implemented by the
 * kernel). Once we support the new standard, we should negotiate the
 * VirtIO version with the guest and automatically use the right type
 * of byte swapping.
 */

/** Convert legacy VirtIO endianness to host endianness. */
template <typename T> inline T
vtoh_legacy(T v) {
    return TheISA::gtoh(v);
}

/** Convert host endianness to legacy VirtIO endianness. */
template <typename T> inline T
htov_legacy(T v) {
    return TheISA::htog(v);
}


template <> inline vring_used_elem
vtoh_legacy(vring_used_elem v) {
    v.id = vtoh_legacy(v.id);
    v.len = vtoh_legacy(v.len);
    return v;
}

template <> inline vring_used_elem
htov_legacy(vring_used_elem v) {
    v.id = htov_legacy(v.id);
    v.len = htov_legacy(v.len);
    return v;
}

template <> inline vring_desc
vtoh_legacy(vring_desc v) {
    v.addr = vtoh_legacy(v.addr);
    v.len = vtoh_legacy(v.len);
    v.flags = vtoh_legacy(v.flags);
    v.next = vtoh_legacy(v.next);
    return v;
}

template <> inline vring_desc
htov_legacy(vring_desc v) {
    v.addr = htov_legacy(v.addr);
    v.len = htov_legacy(v.len);
    v.flags = htov_legacy(v.flags);
    v.next = htov_legacy(v.next);
    return v;
}

/** @} */

/**
 * VirtIO descriptor (chain) wrapper
 *
 * Communication in VirtIO takes place by sending and receiving chains
 * of so called descriptors using device queues. The queue is
 * responsible for sending a descriptor chain from the guest to the
 * host and later sending it back to the guest. The descriptor chain
 * itself can be thought of as a linked list of buffers (descriptors)
 * that are read only (isIncoming() is true) or write only
 * (isOutgoing() is true). A single chain may contain any mix of input
 * and output buffers.
 *
 * The descriptor wrapper is normally <i>only</i> instantiated by the
 * virtqueue wrapper (VirtQueue) and should never be instantiated in
 * device models. The VirtQueue also ensures that the descriptor
 * wrapper is re-populated with new data from the guest by calling
 * updateChain() whenever a new descriptor chain is passed to the host
 * (VirtQueue::consumeDescriptor()). The updateChain() method
 * automatically does some sanity checks on the descriptor chain to
 * detect loops.
 */
class VirtDescriptor
{
  public:
    /** Descriptor index in virtqueue */
    typedef uint16_t Index;

    /** @{
     * @name VirtIO Descriptor <-> Queue Interface
     */
    /**
     * Create a descriptor wrapper.
     *
     * @param memProxy Proxy to the guest physical memory.
     * @param queue Queue owning this descriptor.
     * @param index Index within the queue.
     */
    VirtDescriptor(PortProxy &memProxy, VirtQueue &queue, Index index);
    // WORKAROUND: The noexcept declaration works around a bug where
    // gcc 4.7 tries to call the wrong constructor when emplacing
    // something into a vector.
    VirtDescriptor(VirtDescriptor &&other) noexcept;
    ~VirtDescriptor() noexcept;

    VirtDescriptor &operator=(VirtDescriptor &&rhs) noexcept;

    /** Get the descriptor's index into the virtqueue. */
    Index index() const { return _index; }

    /** Populate this descriptor with data from the guest. */
    void update();

    /** Populate this descriptor chain with data from the guest. */
    void updateChain();
    /** @} */

    /** @{
     * @name Debug interfaces
     */
    /**
     * Dump the contents of a descriptor
     */
    void dump() const;
    /**
     * Dump the contents of a descriptor chain starting at this
     * descriptor.
     */
    void dumpChain() const;
    /** @} */


    /** @{
     * @name Device Model Interfaces
     */
    /**
     * Read the contents of a descriptor.
     *
     * This method copies the contents of a descriptor into a buffer
     * within gem5. Devices should typically use chainRead() instead
     * as it automatically follows the descriptor chain to read the
     * desired number of bytes.
     *
     * @see chainRead
     *
     * @param offset Offset into the descriptor.
     * @param dst Destination buffer.
     * @param size Amount of data to read (in bytes).
     */
    void read(size_t offset, uint8_t *dst, size_t size) const;
    /**
     * Write to the contents of a descriptor.
     *
     * This method copies the contents of a descriptor into a buffer
     * within gem5. Devices should typically use chainWrite() instead
     * as it automatically follows the descriptor chain to read the
     * desired number of bytes.
     *
     * @see chainWrite
     *
     * @param offset Offset into the descriptor.
     * @param src Source buffer.
     * @param size Amount of data to read (in bytes).
     */
    void write(size_t offset, const uint8_t *src, size_t size);
    /**
     * Retrieve the size of this descriptor.
     *
     * This method gets the size of a single descriptor. For incoming
     * data, it corresponds to the amount of data that can be read
     * from the descriptor. For outgoing data, it corresponds to the
     * amount of data that can be written to it.
     *
     * @see chainSize
     *
     * @return Size of descriptor in bytes.
     */
    size_t size() const { return desc.len; }

    /**
     * Is this descriptor chained to another descriptor?
     *
     * @return true if there is a next pointer, false otherwise.
     */
    bool hasNext() const { return desc.flags & VRING_DESC_F_NEXT; }
    /**
     * Get the pointer to the next descriptor in a chain.
     *
     * @return Pointer to the next descriptor or NULL if this is the
     * last element in a chain.
     */
    VirtDescriptor *next() const;

    /** Check if this is a read-only descriptor (incoming data). */
    bool isIncoming() const { return !isOutgoing(); }
    /** Check if this is a write-only descriptor (outgoing data). */
    bool isOutgoing() const { return desc.flags & VRING_DESC_F_WRITE; }


    /**
     * Read the contents of a descriptor chain.
     *
     * This method reads the specified number of bytes from a
     * descriptor chain starting at the this descriptor plus an offset
     * in bytes. The method automatically follows the links in the
     * descriptor chain.
     *
     * @param offset Offset into the chain (in bytes).
     * @param dst Pointer to destination buffer.
     * @param size Size (in bytes).
     */
    void chainRead(size_t offset, uint8_t *dst, size_t size) const;
    /**
     * Write to a descriptor chain.
     *
     * This method writes the specified number of bytes to a
     * descriptor chain starting at the this descriptor plus an offset
     * in bytes. The method automatically follows the links in the
     * descriptor chain.
     *
     * @param offset Offset into the chain (in bytes).
     * @param src Pointer to source buffer.
     * @param size Size (in bytes).
     */
    void chainWrite(size_t offset, const uint8_t *src, size_t size);
    /**
     * Retrieve the size of this descriptor chain.
     *
     * This method gets the size of a descriptor chain starting at
     * this descriptor.
     *
     * @return Size of descriptor chain in bytes.
     */
    size_t chainSize() const;
    /** @} */

  private:
    // Remove default constructor
    VirtDescriptor();
    // Prevent copying
    VirtDescriptor(const VirtDescriptor &other);

    /** Pointer to memory proxy */
    PortProxy *memProxy;
    /** Pointer to virtqueue owning this descriptor */
    VirtQueue *queue;

    /** Index in virtqueue */
    Index _index;

    /** Underlying descriptor */
    vring_desc desc;
};

/**
 * Base wrapper around a virtqueue.
 *
 * VirtIO device models typically need to extend this class to
 * implement their own device queues.
 *
 * @note Queues must be registered with
 * VirtIODeviceBase::registerQueue() to be active.
 */
class VirtQueue : public Serializable {
public:
    virtual ~VirtQueue() {};

    /** @{
     * @name Checkpointing Interface
     */
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    /** @{
     * @name Low-level Device Interface
     */
    /**
     * Set the base address of this queue.
     *
     * @param address Guest physical base address of the queue.
     */
    void setAddress(Addr address);
    /**
     * Get the guest physical address of this queue.
     *
     * @return Physical address in guest where this queue resides.
     */
    Addr getAddress() const { return _address; }

    /**
     * Get the number of descriptors available in this queue.
     *
     * @return Size of queue in descriptors.
     */
     uint16_t getSize() const { return _size; }

    /**
     * Get a pointer to a specific descriptor in the queue.
     *
     * @note This interfaces is normally only used by VirtDescriptor
     * to follow descriptor chains. Device models typically don't need
     * to use it.
     *
     * @return Pointer to a VirtDescriptor.
     */
    VirtDescriptor *getDescriptor(VirtDescriptor::Index index) {
        return &descriptors[index];
    }
    /** @} */

    /** @{
     * @name Device Model Interfaces
     */
    /**
     * Get an incoming descriptor chain from the queue.
     *
     * @return Pointer to descriptor on success, NULL if no pending
     * descriptors are available.
     */
    VirtDescriptor *consumeDescriptor();
    /**
     * Send a descriptor chain to the guest.
     *
     * This method posts a descriptor chain to the guest after a
     * device model has finished processing it. The device model
     * typically needs to call VirtIODeviceBase::kick() to deliver
     * notify tell the guest that the queue has been updated.
     *
     * @note The desc parameter must refer to the first descriptor in
     * a chain that has been retrieved using consumeDescriptor().
     *
     * @note The len parameter specified the amount of data produced
     * by the device model. It seems to be ignored by Linux and it is
     * not well defined.
     *
     * @param desc Start of descriptor chain.
     * @param len Length of the produced data.
     */
    void produceDescriptor(VirtDescriptor *desc, uint32_t len);
    /** @} */

    /** @{
     * @name Device Model Callbacks
     */
    /**
     * Notify queue of pending events.
     *
     * This method is called by VirtIODeviceBase::onNotify() to notify
     * the device model of pending data in a virtqueue. The default
     * implementation of this method iterates over the available
     * descriptor chains and calls onNotifyDescriptor() for every new
     * incoming chain.
     *
     * Device models should normally overload one of onNotify() and
     * onNotifyDescriptor().
     */
    virtual void onNotify();
    /**
     * Notify queue of pending incoming descriptor.
     *
     * This method is called by the default implementation of
     * onNotify() to notify the device model of pending data in a
     * descriptor chain.
     *
     * Device models should normally overload one of onNotify() and
     * onNotifyDescriptor().
     */
    virtual void onNotifyDescriptor(VirtDescriptor *desc) {};
    /** @} */

    /** @{
     * @name Debug interfaces
     */
    /** Dump the contents of a queue */
    void dump() const;
    /** @} */

    /** @{ */
    /**
     * Page size used by VirtIO.\ It's hard-coded to 4096 bytes in
     * the spec for historical reasons.
     */
    static const Addr ALIGN_BITS = 12;
    static const Addr ALIGN_SIZE = 1 << ALIGN_BITS;
    /** @} */

  protected:
    /**
     * Instantiate a new virtqueue.
     *
     * Instantiate a virtqueue with a fixed size. The size is
     * specified in descriptors which are defined as 4096 bytes each.
     *
     * @param proxy Proxy to the guest physical memory.
     * @param size Size in descriptors/pages.
     */
    VirtQueue(PortProxy &proxy, uint16_t size);

  private:
    VirtQueue();

    /** Queue size in terms of number of descriptors */
    const uint16_t _size;
    /** Base address of the queue */
    Addr _address;
    /** Guest physical memory proxy */
    PortProxy &memProxy;

  private:
    /**
     * VirtIO ring buffer wrapper.
     *
     * This class wraps a VirtIO ring buffer. The template parameter T
     * is used to select the data type for the items in the ring (used
     * or available descriptors).
     */
    template<typename T>
    class VirtRing
    {
      public:
        typedef uint16_t Flags;
        typedef uint16_t Index;

        struct Header {
            Flags flags;
            Index index;
        } M5_ATTR_PACKED;

        VirtRing<T>(PortProxy &proxy, uint16_t size)
        : header{0, 0}, ring(size), _proxy(proxy), _base(0) {}

        /**
         * Set the base address of the VirtIO ring buffer.
         *
         * @param addr New host physical address
         */
        void setAddress(Addr addr) { _base = addr; }

        /** Update the ring buffer header with data from the guest. */
        void readHeader() {
            assert(_base != 0);
            _proxy.readBlob(_base, &header, sizeof(header));
            header.flags = vtoh_legacy(header.flags);
            header.index = vtoh_legacy(header.index);
        }

        void writeHeader() {
            Header out;
            assert(_base != 0);
            out.flags = htov_legacy(header.flags);
            out.index = htov_legacy(header.index);
            _proxy.writeBlob(_base, &out, sizeof(out));
        }

        void read() {
            readHeader();

            /* Read and byte-swap the elements in the ring */
            T temp[ring.size()];
            _proxy.readBlob(_base + sizeof(header),
                            temp, sizeof(T) * ring.size());
            for (int i = 0; i < ring.size(); ++i)
                ring[i] = vtoh_legacy(temp[i]);
        }

        void write() {
            assert(_base != 0);
            /* Create a byte-swapped copy of the ring and write it to
             * guest memory. */
            T temp[ring.size()];
            for (int i = 0; i < ring.size(); ++i)
                temp[i] = htov_legacy(ring[i]);
            _proxy.writeBlob(_base + sizeof(header),
                             temp, sizeof(T) * ring.size());
            writeHeader();
        }

        /** Ring buffer header in host byte order */
        Header header;
        /** Elements in ring in host byte order */
        std::vector<T> ring;

      private:
        // Remove default constructor
        VirtRing<T>();

        /** Guest physical memory proxy */
        PortProxy &_proxy;
        /** Guest physical base address of the ring buffer */
        Addr _base;
    };

    /** Ring of available (incoming) descriptors */
    VirtRing<VirtDescriptor::Index> avail;
    /** Ring of used (outgoing) descriptors */
    VirtRing<struct vring_used_elem> used;

    /** Offset of last consumed descriptor in the VirtQueue::avail
     * ring */
    uint16_t _last_avail;

    /** Vector of pre-created descriptors indexed by their index into
     * the queue. */
    std::vector<VirtDescriptor> descriptors;
};

/**
 * Base class for all VirtIO-based devices.
 *
 * This class implements the functionality of the VirtIO 0.9.5
 * specification. This version of VirtIO is also known as "legacy" in
 * the VirtIO 1.0 specification from OASIS.
 *
 * @see https://github.com/rustyrussell/virtio-spec
 * @see http://docs.oasis-open.org/virtio/virtio/v1.0/virtio-v1.0.html
 */
class VirtIODeviceBase : public SimObject
{
  public:
    typedef uint16_t QueueID;
    typedef uint32_t FeatureBits;
    /** This is a VirtQueue address as exposed through the low-level
     * interface.\ The address needs to be multiplied by the page size
     * (seems to be hardcoded to 4096 in the spec) to get the real
     * physical address.
     */
    typedef uint16_t VirtAddress;
    /** Device Type (sometimes known as subsystem ID) */
    typedef uint16_t DeviceId;

    BitUnion8(DeviceStatus)
        Bitfield<7> failed;
        Bitfield<2> driver_ok;
        Bitfield<1> driver;
        Bitfield<0> acknowledge;
    EndBitUnion(DeviceStatus)

    typedef VirtIODeviceBaseParams Params;
    VirtIODeviceBase(Params *params, DeviceId id, size_t config_size,
                     FeatureBits features);
    virtual ~VirtIODeviceBase();

  public:
    /** @{
     * @name SimObject Interfaces
     */
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
    /** @} */


  protected:
    /** @{
     * @name Device Model Interfaces
     */

    /**
     * Inform the guest of available buffers.
     *
     * When a device model has finished processing incoming buffers
     * (after onNotify has been called), it typically needs to inform
     * the guest that there are new pending outgoing buffers. The
     * method used to inform the guest is transport dependent, but is
     * typically through an interrupt. Device models call this method
     * to tell the transport interface to notify the guest.
     */
    void kick() {
        assert(transKick);
        transKick->process();
    };

    /**
     * Register a new VirtQueue with the device model.
     *
     * Devices typically register at least one VirtQueue to use for
     * communication with the guest. This <i>must</i> be done from the
     * constructor since the number of queues are assumed to be
     * constant throughout the lifetime of the device.
     *
     * @warning This method may only be called from the device model
     * constructor.
     */
    void registerQueue(VirtQueue &queue);


    /**
     * Feature set accepted by the guest.
     *
     * When the guest starts the driver for the device, it starts by
     * negotiating features. The device first offers a set of features
     * (see deviceFeatures), the driver then notifies the device of
     * which features it accepted. The base class will automatically
     * accept any feature set that is a subset of the features offered
     * by the device.
     */
    FeatureBits guestFeatures;
    /** @} */

  public:
    /** @{
     * @name Optional VirtIO Interfaces
     */
    /**
     * Read from the configuration space of a device.
     *
     * This method is called by the transport interface to read data
     * from a device model's configuration space. The device model
     * should use the cfgOffset parameter as the offset into its
     * configuration space.
     *
     * @warning The address in the packet should not be used to
     * determine the offset into a device's configuration space.
     *
     * @param pkt Read request packet.
     * @param cfgOffset Offset into the device's configuration space.
     */
    virtual void readConfig(PacketPtr pkt, Addr cfgOffset);
    /**
     * Write to the configuration space of a device.
     *
     * This method is called by the transport interface to write data
     * into a device model's configuration space. The device model
     * should use the cfgOffset parameter as the offset into its
     * configuration space.
     *
     * @warning The address in the packet should not be used to
     * determine the offset into a device's configuration space.
     *
     * @param pkt Write request packet.
     * @param cfgOffset Offset into the device's configuration space.
     */
    virtual void writeConfig(PacketPtr pkt, Addr cfgOffset);

    /**
     * Driver-request device reset.
     *
     * The device driver may reset a device by writing zero to the
     * device status register (using setDeviceStatus()), which causes
     * this method to be called. Device models overriding this method
     * <i>must</i> ensure that the reset method of the base class is
     * called when the device is reset.
     *
     * @note Always call the reset method of the base class from
     * device-specific reset methods.
     */
    virtual void reset();
    /** @} */

  protected:
    /** @{
     * @name Device Model Helpers
     */

    /**
     * Read configuration data from a device structure.
     *
     * @param pkt Read request packet.
     * @param cfgOffset Offset into the device's configuration space.
     * @param cfg Device configuration
     */
    void readConfigBlob(PacketPtr pkt, Addr cfgOffset, const uint8_t *cfg);

    /**
     * Write configuration data to a device structure.
     *
     * @param pkt Write request packet.
     * @param cfgOffset Offset into the device's configuration space.
     * @param cfg Device configuration
     */
    void writeConfigBlob(PacketPtr pkt, Addr cfgOffset, uint8_t *cfg);

    /** @} */

  public:
    /** @{
     * @name VirtIO Transport Interfaces
     */
     /**
      * Register a callback to kick the guest through the transport
      * interface.
      *
      * @param c Callback into transport interface.
      */
    void registerKickCallback(Callback *c) {
        assert(!transKick);
        transKick = c;
    }


    /**
     * Driver is requesting service.
     *
     * This method is called by the underlying hardware interface
     * (e.g., PciVirtIO or MmmioVirtIO) to notify a device of pending
     * incoming descriptors.
     *
     * @param index ID of the queue with pending actions.
     */
    void onNotify(QueueID index);


    /**
     * Change currently active queue.
     *
     * The transport interface works on a queue at a time. The
     * currently active queue is decided by the value of the queue
     * select field in a device.
     *
     * @param idx ID of the queue to select.
     */
    void setQueueSelect(QueueID idx) { _queueSelect = idx; }
    /**
     * Get the currently active queue.
     *
     * The transport interface works on a queue at a time. The
     * currently active queue is decided by the value of the queue
     * select field in a device.
     *
     * @return The ID of the currently active queue.
     */
    QueueID getQueueSelect() const { return _queueSelect; }

    /**
     * Change the host physical address of the currently active queue.
     *
     * @note The new address is specified in multiples of the page
     * size (fixed to 4096 bytes in the standard). For example, if the
     * address 10 is selected, the actual host physical address will
     * be 40960.
     *
     * @see setQueueSelect
     * @see getQueueSelect
     *
     * @param address New address of the currently active queue (in
     * pages).
     */
    void setQueueAddress(uint32_t address);
    /**
     * Get the host physical address of the currently active queue.
     *
     * @note The new address is specified in multiples of the page
     * size (fixed to 4096 bytes in the standard). For example, if the
     * address 10 is selected, the actual host physical address will
     * be 40960.
     *
     * @see setQueueSelect
     * @see getQueueSelect
     *
     * @return Address of the currently active queue (in pages).
     */
    uint32_t getQueueAddress() const;

    /**
     * Get the size (descriptors) of the currently active queue.
     *
     * @return Size of the currently active queue in number of
     * descriptors.
     */
    uint16_t getQueueSize() const { return getCurrentQueue().getSize(); }

    /**
     * Update device status and optionally reset device.
     *
     * The special device status of 0 is used to reset the device by
     * calling reset().
     *
     * @param status New device status.
     */
    void setDeviceStatus(DeviceStatus status);

    /**
     * Retrieve the device status.
     *
     * @return Device status.
     */
    DeviceStatus getDeviceStatus() const { return _deviceStatus; }

    /**
     * Set feature bits accepted by the guest driver.
     *
     * This enables a subset of the features offered by the device
     * model through the getGuestFeatures() interface.
     */
    void setGuestFeatures(FeatureBits features);

    /**
     * Get features accepted by the guest driver.
     *
     * @return Currently active features.
     */
    FeatureBits getGuestFeatures() const { return guestFeatures; }

    /** Device ID (sometimes known as subsystem ID) */
    const DeviceId deviceId;

    /** Size of the device's configuration space */
    const size_t configSize;

    /** Feature set offered by the device */
    const FeatureBits deviceFeatures;
    /** @} */

  private:
    /** Convenience method to get the currently selected queue */
    const VirtQueue &getCurrentQueue() const;
    /** Convenience method to get the currently selected queue */
    VirtQueue &getCurrentQueue();

    /**
     * Status of the device
     *
     * @see getDeviceStatus
     * @see setDeviceStatus
     */
    DeviceStatus _deviceStatus;

    /** Queue select register (set by guest) */
    QueueID _queueSelect;

    /** List of virtual queues supported by this device */
    std::vector<VirtQueue *> _queues;

    /** Callbacks to kick the guest through the transport layer  */
    Callback *transKick;
};

class VirtIODummyDevice : public VirtIODeviceBase
{
  public:
    VirtIODummyDevice(VirtIODummyDeviceParams *params);

  protected:
    /** VirtIO device ID */
    static const DeviceId ID_INVALID = 0x00;
};

#endif // __DEV_VIRTIO_BASE_HH__
