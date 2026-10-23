/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * (University of Wisconsin-Madison)
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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

#ifndef __SALAM_DMA_WRITE_FIFO_HH__
#define __SALAM_DMA_WRITE_FIFO_HH__

#include "dev/dma_device.hh"
#include "salam/LLVMRead/debug_flags.hh"

using namespace gem5;

class DmaWriteFifo : public Drainable, public Serializable
{
  public:
    DmaWriteFifo(DmaPort &port, size_t size, unsigned max_req_size,
                 unsigned max_pending, Request::Flags flags = 0);

    ~DmaWriteFifo();

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
    bool canFill(size_t len);

    bool tryFill(uint8_t *src, size_t len);

    template <typename T>
    bool
    tryFill(T &value)
    {
        return tryFill(static_cast<T *>(&value), sizeof(T));
    };

    /**
     * Read data from the FIFO and panic on failure.
     *
     * @see tryFill()
     *
     * @param dst Pointer to a destination buffer
     * @param len Amount of data to read.
     */
    void fill(uint8_t *src, size_t len);

    template <typename T>
    T
    fill()
    {
        T value;
        fill(static_cast<uint8_t *>(&value), sizeof(T));
        return value;
    };

    /** Get the amount of data stored in the FIFO */
    size_t
    size() const
    {
        return buffer.size();
    }
    /** Flush the FIFO */
    void
    flush()
    {
        buffer.flush();
    }

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
    void startEmpty(Addr start, size_t size);

    /**
     * Stop the DMA engine.
     *
     * Stop filling the FIFO and ignore incoming responses for pending
     * requests. The onEndOfBlock() callback will not be called after
     * this method has been invoked. However, once the last response
     * has been received, the onIdle() callback will still be called.
     */
    void stopEmpty();

    /**
     * Has the DMA engine sent out the last request for the active
     * block?
     */
    bool
    atEndOfBlock() const
    {
        return nextAddr == endAddr;
    }

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

  private:
    class DmaDoneEvent : public Event
    {
      public:
        DmaDoneEvent(DmaWriteFifo *_parent, size_t max_size);

        void kill();
        void cancel();
        bool
        canceled() const
        {
            return _canceled;
        }
        void reset(size_t size);
        void process();

        bool
        done() const
        {
            return _done;
        }
        size_t
        requestSize() const
        {
            return _requestSize;
        }
        const uint8_t *
        data() const
        {
            return _data.data();
        }
        uint8_t *
        data()
        {
            return _data.data();
        }

      private:
        DmaWriteFifo *parent;
        bool _done;
        bool _canceled;
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
    void resumeEmpty();

    /** Try to issue new DMA requests during normal execution*/
    void resumeEmptyTiming();

    /** Try to bypass DMA requests in KVM execution mode */
    void resumeEmptyFunctional();

  private: // Internal state
    Fifo<uint8_t> buffer;

    Addr nextAddr;
    Addr endAddr;

    std::deque<DmaDoneEventUPtr> pendingRequests;
    std::deque<DmaDoneEventUPtr> freeRequests;
};

#endif //__SALAM_DMA_WRITE_FIFO_HH__
