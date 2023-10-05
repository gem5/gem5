/*
 * Copyright (c) 1999-2012 Mark D. Hill and David A. Wood
 * Copyright (c) 2010 Advanced Micro Devices, Inc.
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

/*
 * Recording cache requests made to a ruby cache at certain ruby
 * time. Also dump the requests to a gziped file.
 */

#ifndef __MEM_RUBY_SYSTEM_CACHERECORDER_HH__
#define __MEM_RUBY_SYSTEM_CACHERECORDER_HH__

#include <vector>

#include "base/types.hh"
#include "config/build_gpu.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/DataBlock.hh"
#include "mem/ruby/common/TypeDefines.hh"
#include "mem/ruby/protocol/RubyRequestType.hh"

namespace gem5
{

namespace ruby
{

class Sequencer;
#if BUILD_GPU
class GPUCoalescer;
#endif

/*!
 * Class for recording cache contents. Note that the last element of the
 * class is an array of length zero. It is used for creating variable
 * length object, so that while writing the data to a file one does not
 * need to copy the meta data and the actual data separately.
 */
class TraceRecord
{
  public:
    int m_cntrl_id;
    Tick m_time;
    Addr m_data_address;
    Addr m_pc_address;
    RubyRequestType m_type;
    uint8_t m_data[0];

    void print(std::ostream& out) const;
};

class CacheRecorder
{
  public:
    CacheRecorder();
    ~CacheRecorder();

#if BUILD_GPU
    CacheRecorder(uint8_t* uncompressed_trace,
                  uint64_t uncompressed_trace_size,
                  std::vector<Sequencer*>& SequencerMap,
                  std::vector<GPUCoalescer*>& CoalescerMap,
                  uint64_t block_size_bytes);
#else
    CacheRecorder(uint8_t* uncompressed_trace,
                  uint64_t uncompressed_trace_size,
                  std::vector<Sequencer*>& SequencerMap,
                  uint64_t block_size_bytes);
#endif
    void addRecord(int cntrl, Addr data_addr, Addr pc_addr,
                   RubyRequestType type, Tick time, DataBlock& data);

    uint64_t aggregateRecords(uint8_t **data, uint64_t size);

    uint64_t getNumRecords() const;

    /*!
     * Function for flushing the memory contents of the caches to the
     * main memory. It goes through the recorded contents of the caches,
     * and issues flush requests. Except for the first one, a flush request
     * is issued only after the previous one has completed. This currently
     * requires use of MOESI Hammer protocol since only that protocol
     * supports flush requests.
     */
    void enqueueNextFlushRequest();

    /*!
     * Function for fetching warming up the memory and the caches. It goes
     * through the recorded contents of the caches, as available in the
     * checkpoint and issues fetch requests. Except for the first one, a
     * fetch request is issued only after the previous one has completed.
     * It should be possible to use this with any protocol.
     */
    void enqueueNextFetchRequest();

  private:
    // Private copy constructor and assignment operator
    CacheRecorder(const CacheRecorder& obj);
    CacheRecorder& operator=(const CacheRecorder& obj);

    std::vector<TraceRecord*> m_records;
    uint8_t* m_uncompressed_trace;
    uint64_t m_uncompressed_trace_size;
    std::vector<Sequencer*> m_seq_map;
#if BUILD_GPU
    std::vector<GPUCoalescer*> m_coalescer_map;
#endif
    uint64_t m_bytes_read;
    uint64_t m_records_read;
    uint64_t m_records_flushed;
    uint64_t m_block_size_bytes;
};

inline bool
compareTraceRecords(const TraceRecord* n1, const TraceRecord* n2)
{
    return n1->m_time > n2->m_time;
}

inline std::ostream&
operator<<(std::ostream& out, const TraceRecord& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

} // namespace ruby
} // namespace gem5

#endif //__MEM_RUBY_SYSTEM_CACHERECORDER_HH__
