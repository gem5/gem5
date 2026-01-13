/*
 * Copyright (c) 2012-2013, 2015, 2017, 2019 ARM Limited
 * All rights reserved.
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
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#ifndef __DEV_DMA_DEVICE_HH__
#define __DEV_DMA_DEVICE_HH__

#include <deque>
#include <optional>
#include <memory>

#include "base/addr_range_map.hh"
#include "base/chunk_generator.hh"
#include "base/circlebuf.hh"
#include "dev/io_device.hh"
#include "mem/backdoor.hh"
#include "params/DmaDevice.hh"
#include "sim/drain.hh"
#include "sim/system.hh"

namespace gem5
{

class ClockedObject;

class DmaPort : public RequestPort, public Drainable
{
  private:
    AddrRangeMap<MemBackdoorPtr, 1> memBackdoors;

    /**
     * Take the first request on the transmit list and attempt to send a timing
     * packet from it. If it is successful, schedule the sending of the next
     * packet. Otherwise remember that we are waiting for a retry.
     */
    void trySendTimingReq();

    /**
     * For timing, attempt to send the first item on the transmit
     * list, and if it is successful and there are more packets
     * waiting, then schedule the sending of the next packet. For
     * atomic, simply send and process everything on the transmit
     * list.
     */
    void sendDma();

    struct DmaReqState : public Packet::SenderState
    {
        /** Event to call on the device when this transaction (all packets)
         * complete. */
        Event *completionEvent;

        /** Event to call on the device when this transaction is aborted. */
        Event *abortEvent;

        /** Whether this request was aborted. */
        bool aborted = false;

        /** Total number of bytes that this transaction involves. */
        const Addr totBytes;

        /** Number of bytes that have been acked for this transaction. */
        Addr numBytes = 0;

        /** Amount to delay completion of dma by */
        const Tick delay;

        /** Object to track what chunks of bytes to send at a time. */
        ChunkGenerator gen;

        /** Pointer to a buffer for the data. */
        uint8_t *const data = nullptr;

        /** The flags to use for requests. */
        const Request::Flags flags;

        /** The requestor ID to use for requests. */
        const RequestorID id;

        /** Stream IDs. */
        const std::optional<uint32_t> sid;
        const std::optional<uint32_t> ssid;

        /** Command for the request. */
        const Packet::Command cmd;

        DmaReqState(Packet::Command _cmd, Addr addr, Addr chunk_sz, Addr tb,
                    uint8_t *_data, Request::Flags _flags, RequestorID _id,
                    std::optional<uint32_t> _sid, std::optional<uint32_t> _ssid,
                    Event *ce, Tick _delay, Event *ae=nullptr)
            : completionEvent(ce), abortEvent(ae), totBytes(tb), delay(_delay),
              gen(addr, tb, chunk_sz), data(_data), flags(_flags), id(_id),
              sid(_sid), ssid(_ssid), cmd(_cmd)
        {}

        PacketPtr createPacket();
    };

    /** Send the next packet from a DMA request in atomic mode. */
    bool sendAtomicReq(DmaReqState *state);
    /**
     * Send the next packet from a DMA request in atomic mode, and request
     * and/or use memory backdoors if possible.
     */
    bool sendAtomicBdReq(DmaReqState *state);

    /**
     * Handle a response packet by updating the corresponding DMA
     * request state to reflect the bytes received, and also update
     * the pending request counter. If the DMA request that this
     * packet is part of is complete, then signal the completion event
     * if present, potentially with a delay added to it.
     *
     * @param pkt Response packet to handler
     * @param delay Additional delay for scheduling the completion event
     */
    void handleRespPacket(PacketPtr pkt, Tick delay=0);
    void handleResp(DmaReqState *state, Addr addr, Addr size, Tick delay=0);

  public:
    /** The device that owns this port. */
    ClockedObject *const device;

    /** The system that device/port are in. This is used to select which mode
     * we are currently operating in. */
    System *const sys;

    /** Id for all requests */
    const RequestorID requestorId;

  protected:
    /** Use a deque as we never do any insertion or removal in the middle */
    std::deque<DmaReqState *> transmitList;

    /** Event used to schedule a future sending from the transmit list. */
    EventFunctionWrapper sendEvent;

    /** Number of outstanding packets the dma port has. */
    uint32_t pendingCount = 0;

    /** The packet (if any) waiting for a retry to send. */
    PacketPtr inRetry = nullptr;
    /**
     * Whether the other side expects us to wait for a retry. We may have
     * decided not to actually send the packet by the time we get the retry.
     */
    bool retryPending = false;

    /** Default streamId */
    const std::optional<uint32_t> defaultSid;

    /** Default substreamId */
    const std::optional<uint32_t> defaultSSid;

    const Addr cacheLineSize;

  protected:

    bool recvTimingResp(PacketPtr pkt) override;
    void recvReqRetry() override;

  public:

    DmaPort(ClockedObject *dev, System *s, std::optional<uint32_t> sid=0,
            std::optional<uint32_t> ssid=0);

    void
    dmaAction(Packet::Command cmd, Addr addr, int size, Event *event,
              uint8_t *data, Tick delay, Request::Flags flag=0);

    void
    dmaAction(Packet::Command cmd, Addr addr, int size, Event *event,
              uint8_t *data, std::optional<uint32_t> sid,
              std::optional<uint32_t> ssid, Tick delay, Request::Flags flag=0);

    // Abort and remove any pending DMA transmissions.
    void abortPending();

    bool dmaPending() const { return pendingCount > 0; }

    DrainState drain() override;
};

class DmaDevice : public PioDevice
{
   protected:
    DmaPort dmaPort;

  public:
    typedef DmaDeviceParams Params;
    DmaDevice(const Params &p);
    virtual ~DmaDevice() = default;

    void
    dmaWrite(Addr addr, int size, Event *event, uint8_t *data,
             std::optional<uint32_t> sid, std::optional<uint32_t> ssid,
             Tick delay=0)
    {
        dmaPort.dmaAction(MemCmd::WriteReq, addr, size, event, data,
                          sid, ssid, delay);
    }

    void
    dmaWrite(Addr addr, int size, Event *event, uint8_t *data, Tick delay=0)
    {
        dmaPort.dmaAction(MemCmd::WriteReq, addr, size, event, data, delay);
    }

    void
    dmaRead(Addr addr, int size, Event *event, uint8_t *data,
            std::optional<uint32_t> sid, std::optional<uint32_t> ssid,
            Tick delay=0)
    {
        dmaPort.dmaAction(MemCmd::ReadReq, addr, size, event, data,
                          sid, ssid, delay);
    }

    void
    dmaRead(Addr addr, int size, Event *event, uint8_t *data, Tick delay=0)
    {
        dmaPort.dmaAction(MemCmd::ReadReq, addr, size, event, data, delay);
    }

    bool dmaPending() const { return dmaPort.dmaPending(); }

    void init() override;

    Addr cacheBlockSize() const { return sys->cacheLineSize(); }

    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

};

/**
 * DMA callback class.
 *
 * Allows one to register for a callback event after a sequence of (potentially
 * non-contiguous) DMA transfers on a DmaPort completes.  Derived classes must
 * implement the process() method and use getChunkEvent() to allocate a
 * callback event for each participating DMA.
 */
class DmaCallback : public Drainable
{
  public:
    virtual const std::string name() const { return "DmaCallback"; }

    /**
     * DmaPort ensures that all oustanding DMA accesses have completed before
     * it finishes draining.  However, DmaChunkEvents scheduled with a delay
     * might still be sitting on the event queue.  Therefore, draining is not
     * complete until count is 0, which ensures that all outstanding
     * DmaChunkEvents associated with this DmaCallback have fired.
     */
    DrainState
    drain() override
    {
        return count ? DrainState::Draining : DrainState::Drained;
    }

  protected:
    int count = 0;

    virtual ~DmaCallback() = default;

    /**
     * Callback function invoked on completion of all chunks.
     */
    virtual void process() = 0;

  private:
    /**
     * Called by DMA engine completion event on each chunk completion.
     * Since the object may delete itself here, callers should not use
     * the object pointer after calling this function.
     */
    void
    chunkComplete()
    {
        if (--count == 0) {
            process();
            // Need to notify DrainManager that this object is finished
            // draining, even though it is immediately deleted.
            signalDrainDone();
            delete this;
        }
    }

  public:

    /**
     * Request a chunk event.  Chunks events should be provided to each DMA
     * request that wishes to participate in this DmaCallback.
     */
    Event *
    getChunkEvent()
    {
        ++count;
        return new EventFunctionWrapper([this]{ chunkComplete(); }, name(),
                                        true);
    }
};

/**
 * Buffered DMA engine helper class
 *
 * This class implements a simple DMA engine that feeds a FIFO
 * buffer. The size of the buffer, the maximum number of pending
 * requests and the maximum request size are all set when the engine
 * is instantiated.
 *
 * An <i>asynchronous</i> transfer of a <i>block</i> of data
 * (designated by a start address and a size) is started by calling
 * the startFill() method. The DMA engine will aggressively try to
 * keep the internal FIFO full. As soon as there is room in the FIFO
 * for more data <i>and</i> there are free request slots, a new fill
 * will be started.
 *
 * Data in the FIFO can be read back using the get() and tryGet()
 * methods. Both request a block of data from the FIFO. However, get()
 * panics if the block cannot be satisfied, while tryGet() simply
 * returns false. The latter call makes it possible to implement
 * custom buffer underrun handling.
 *
 * A simple use case would be something like this:
 * \code{.cpp}
 *     // Create a DMA engine with a 1KiB buffer. Issue up to 8 concurrent
 *     // uncacheable 64 byte (maximum) requests.
 *     DmaReadFifo *dma = new DmaReadFifo(port, 1024, 64, 8,
 *                                        Request::UNCACHEABLE);
 *
 *     // Start copying 4KiB data from 0xFF000000
 *     dma->startFill(0xFF000000, 0x1000);
 *
 *     // Some time later when there is data in the FIFO.
 *     uint8_t data[8];
 *     dma->get(data, sizeof(data))
 * \endcode
 *
 *
 * The DMA engine allows new blocks to be requested as soon as the
 * last request for a block has been sent (i.e., there is no need to
 * wait for pending requests to complete). This can be queried with
 * the atEndOfBlock() method and more advanced implementations may
 * override the onEndOfBlock() callback.
 */
class DmaReadFifo : public Drainable, public Serializable
{
  public:
    DmaReadFifo(DmaPort &port, size_t size,
                unsigned max_req_size,
                unsigned max_pending,
                Request::Flags flags=0);

    ~DmaReadFifo();

  public: // Serializable
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  public: // Drainable
    DrainState drain() override;

  public: // FIFO access
    /**
     * @{
     * @name FIFO access
     */
    /**
     * Try to read data from the FIFO.
     *
     * This method reads len bytes of data from the FIFO and stores
     * them in the memory location pointed to by dst. The method
     * fails, and no data is written to the buffer, if the FIFO
     * doesn't contain enough data to satisfy the request.
     *
     * @param dst Pointer to a destination buffer
     * @param len Amount of data to read.
     * @return true on success, false otherwise.
     */
    bool tryGet(uint8_t *dst, size_t len);

    template<typename T>
    bool
    tryGet(T &value)
    {
        return tryGet(static_cast<T *>(&value), sizeof(T));
    };

    /**
     * Read data from the FIFO and panic on failure.
     *
     * @see tryGet()
     *
     * @param dst Pointer to a destination buffer
     * @param len Amount of data to read.
     */
    void get(uint8_t *dst, size_t len);

    template<typename T>
    T
    get()
    {
        T value;
        get(static_cast<uint8_t *>(&value), sizeof(T));
        return value;
    };

    /** Get the amount of data stored in the FIFO */
    size_t size() const { return buffer.size(); }
    /** Flush the FIFO */
    void flush() { buffer.flush(); }

    /** @} */
  public: // FIFO fill control
    /**
     * @{
     * @name FIFO fill control
     */
    /**
     * Start filling the FIFO.
     *
     * @warn It's considered an error to call start on an active DMA
     * engine unless the last request from the active block has been
     * sent (i.e., atEndOfBlock() is true).
     *
     * @param start Physical address to copy from.
     * @param size Size of the block to copy.
     */
    void startFill(Addr start, size_t size);

    /**
     * Stop the DMA engine.
     *
     * Stop filling the FIFO and ignore incoming responses for pending
     * requests. The onEndOfBlock() callback will not be called after
     * this method has been invoked. However, once the last response
     * has been received, the onIdle() callback will still be called.
     */
    void stopFill();

    /**
     * Has the DMA engine sent out the last request for the active
     * block?
     */
    bool atEndOfBlock() const { return nextAddr == endAddr; }

    /**
     * Is the DMA engine active (i.e., are there still in-flight
     * accesses)?
     */
    bool
    isActive() const
    {
        return !(pendingRequests.empty() && atEndOfBlock());
    }

    /** @} */
  protected: // Callbacks
    /**
     * @{
     * @name Callbacks
     */
    /**
     * End of block callback
     *
     * This callback is called <i>once</i> after the last access in a
     * block has been sent. It is legal for a derived class to call
     * startFill() from this method to initiate a transfer.
     */
    virtual void onEndOfBlock() {};

    /**
     * Last response received callback
     *
     * This callback is called when the DMA engine becomes idle (i.e.,
     * there are no pending requests).
     *
     * It is possible for a DMA engine to reach the end of block and
     * become idle at the same tick. In such a case, the
     * onEndOfBlock() callback will be called first. This callback
     * will <i>NOT</i> be called if that callback initiates a new DMA transfer.
     */
    virtual void onIdle() {};

    /** @} */
  private: // Configuration
    /** Maximum request size in bytes */
    const Addr maxReqSize;
    /** Maximum FIFO size in bytes */
    const size_t fifoSize;
    /** Request flags */
    const Request::Flags reqFlags;

    DmaPort &port;

    const Addr cacheLineSize;

  private:
    class DmaDoneEvent : public Event
    {
      public:
        DmaDoneEvent(DmaReadFifo *_parent, size_t max_size);

        void kill();
        void cancel();
        bool canceled() const { return _canceled; }
        void reset(size_t size);
        void process();

        bool done() const { return _done; }
        size_t requestSize() const { return _requestSize; }
        const uint8_t *data() const { return _data.data(); }
        uint8_t *data() { return _data.data(); }

      private:
        DmaReadFifo *parent;
        bool _done = false;
        bool _canceled = false;
        size_t _requestSize;
        std::vector<uint8_t> _data;
    };

    typedef std::unique_ptr<DmaDoneEvent> DmaDoneEventUPtr;

    /**
     * DMA request done, handle incoming data and issue new
     * request.
     */
    void dmaDone();

    /** Handle pending requests that have been flagged as done. */
    void handlePending();

    /** Try to issue new DMA requests or bypass DMA requests*/
    void resumeFill();

    /** Try to issue new DMA requests during normal execution*/
    void resumeFillTiming();

    /** Try to bypass DMA requests in non-caching mode */
    void resumeFillBypass();

  private: // Internal state
    Fifo<uint8_t> buffer;

    Addr nextAddr = 0;
    Addr endAddr = 0;

    std::deque<DmaDoneEventUPtr> pendingRequests;
    std::deque<DmaDoneEventUPtr> freeRequests;
};

} // namespace gem5

#endif // __DEV_DMA_DEVICE_HH__
