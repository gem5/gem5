/*
 * Copyright (c) 2012-2013, 2017-2018 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed here under.  You may use the software subject to the license
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
 * Declaration of the trace generator that reads a trace file
 * and plays the transactions.
 */

#ifndef __CPU_TRAFFIC_GEN_TRACE_GEN_HH__
#define __CPU_TRAFFIC_GEN_TRACE_GEN_HH__

#include "base/bitfield.hh"
#include "base/intmath.hh"
#include "base_gen.hh"
#include "mem/packet.hh"
#include "proto/protoio.hh"

namespace gem5
{

/**
 * The trace replay generator reads a trace file and plays
 * back the transactions. The trace is offset with respect to
 * the time when the state was entered.
 */
class TraceGen : public BaseGen
{
  private:
    /**
     * This struct stores a line in the trace file.
     */
    struct TraceElement
    {
        /** Specifies if the request is to be a read or a write */
        MemCmd cmd;

        /** The address for the request */
        Addr addr;

        /** The size of the access for the request */
        Addr blocksize;

        /** The time at which the request should be sent */
        Tick tick;

        /** Potential request flags to use */
        Request::FlagsType flags;

        /**
         * Check validity of this element.
         *
         * @return if this element is valid
         */
        bool
        isValid() const
        {
            return cmd != MemCmd::InvalidCmd;
        }

        /**
         * Make this element invalid.
         */
        void
        clear()
        {
            cmd = MemCmd::InvalidCmd;
        }
    };

    /**
     * The InputStream encapsulates a trace file and the
     * internal buffers and populates TraceElements based on
     * the input.
     */
    class InputStream
    {
      private:
        /// Input file stream for the protobuf trace
        ProtoInputStream trace;

      public:
        /**
         * Create a trace input stream for a given file name.
         *
         * @param filename Path to the file to read from
         */
        InputStream(const std::string &filename);

        /**
         * Reset the stream such that it can be played once
         * again.
         */
        void reset();

        /**
         * Check the trace header to make sure that it is of the right
         * format.
         */
        void init();

        /**
         * Attempt to read a trace element from the stream,
         * and also notify the caller if the end of the file
         * was reached.
         *
         * @param element Trace element to populate
         * @return True if an element could be read successfully
         */
        bool read(TraceElement &element);
    };

  public:
    /**
     * Create a trace generator.
     *
     * @param obj SimObject owning this sequence generator
     * @param requestor_id RequestorID related to the memory requests
     * @param _duration duration of this state before transitioning
     * @param trace_file File to read the transactions from
     * @param addr_offset Positive offset to add to trace address
     */
    TraceGen(SimObject &obj, RequestorID requestor_id, Tick _duration,
             const std::string &trace_file, Addr addr_offset)
        : BaseGen(obj, requestor_id, _duration),
          trace(trace_file),
          tickOffset(0),
          addrOffset(addr_offset),
          traceComplete(false)
    {}

    void enter();

    PacketPtr getNextPacket();

    void exit();

    /**
     * Returns the tick when the next request should be generated. If
     * the end of the file has been reached, it returns MaxTick to
     * indicate that there will be no more requests.
     */
    Tick nextPacketTick(bool elastic, Tick delay) const;

  private:
    /** Input stream used for reading the input trace file */
    InputStream trace;

    /** Store the current and next element in the trace */
    TraceElement currElement;
    TraceElement nextElement;

    /**
     * Stores the time when the state was entered. This is to add an
     * offset to the times stored in the trace file. This is mutable
     * to allow us to change it as part of nextPacketTick.
     */
    mutable Tick tickOffset;

    /**
     * Offset for memory requests. Used to shift the trace
     * away from the CPU address space.
     */
    Addr addrOffset;

    /**
     * Set to true when the trace replay for one instance of
     * state is complete.
     */
    bool traceComplete;
};

} // namespace gem5

#endif
