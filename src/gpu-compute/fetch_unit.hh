/*
 * Copyright (c) 2014-2017 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __FETCH_UNIT_HH__
#define __FETCH_UNIT_HH__

#include <string>
#include <utility>

#include "arch/gpu_decoder.hh"
#include "base/statistics.hh"
#include "config/the_gpu_isa.hh"
#include "gpu-compute/scheduler.hh"
#include "mem/packet.hh"

class ComputeUnit;
class Wavefront;

class FetchUnit
{
  public:
    FetchUnit(const ComputeUnitParams* p, ComputeUnit &cu);
    ~FetchUnit();
    void init();
    void exec();
    void bindWaveList(std::vector<Wavefront*> *list);
    void initiateFetch(Wavefront *wavefront);
    void fetch(PacketPtr pkt, Wavefront *wavefront);
    void processFetchReturn(PacketPtr pkt);
    void flushBuf(int wfSlotId);
    static uint32_t globalFetchUnitID;

  private:
    /**
     * fetch buffer descriptor. holds buffered
     * instruction data in the fetch unit.
     */
    class FetchBufDesc
    {
      public:
        FetchBufDesc() : bufStart(nullptr), bufEnd(nullptr),
            readPtr(nullptr), fetchDepth(0), maxIbSize(0), maxFbSize(0),
            cacheLineSize(0), restartFromBranch(false), wavefront(nullptr),
            _decoder(nullptr)
        {
        }

        ~FetchBufDesc()
        {
            delete[] bufStart;
        }

        /**
         * allocate the fetch buffer space, and set the fetch depth
         * (number of lines that may be buffered), fetch size
         * (cache line size), and parent WF for this fetch buffer.
         */
        void allocateBuf(int fetch_depth, int cache_line_size, Wavefront *wf);

        int
        bufferedAndReservedLines() const
        {
            return bufferedLines() + reservedLines();
        }

        int bufferedLines() const { return bufferedPCs.size(); }
        int bufferedBytes() const { return bufferedLines() * cacheLineSize; }
        int reservedLines() const { return reservedPCs.size(); }
        bool hasFreeSpace() const { return !freeList.empty(); }
        void flushBuf();
        Addr nextFetchAddr();

        /**
         * reserve an entry in the fetch buffer for PC = vaddr,
         */
        void reserveBuf(Addr vaddr);

        /**
         * return a pointer to the raw fetch buffer data.
         * this allows the fetch pkt to use this data directly
         * to avoid unnecessary memcpy and malloc/new.
         */
        uint8_t*
        reservedBuf(Addr vaddr) const
        {
            auto reserved_pc = reservedPCs.find(vaddr);
            assert(reserved_pc != reservedPCs.end());
            assert(reserved_pc == reservedPCs.begin());

            return reserved_pc->second;
        }

        /**
         * returns true if there is an entry reserved for this address,
         * and false otherwise
         */
        bool
        isReserved(Addr vaddr) const
        {
            auto reserved_pc = reservedPCs.find(vaddr);
            bool is_reserved = (reserved_pc != reservedPCs.end());
            return is_reserved;
        }

        void fetchDone(Addr vaddr);

        /**
         * checks if the buffer contains valid data. this essentially
         * tells fetch when there is data remaining that needs to be
         * decoded into the WF's IB.
         */
        bool hasFetchDataToProcess() const;

        /**
         * each time the fetch stage is ticked, we check if there
         * are any data in the fetch buffer that may be decoded and
         * sent to the IB. because we are modeling the fetch buffer
         * as a circular buffer, it is possible that an instruction
         * can straddle the end/beginning of the fetch buffer, so
         * decodeSplitInsts() handles that case.
         */
        void decodeInsts();

        /**
         * checks if the wavefront can release any of its fetch
         * buffer entries. this will occur when the WF's PC goes
         * beyond any of the currently buffered cache lines.
         */
        void checkWaveReleaseBuf();

        void
        decoder(TheGpuISA::Decoder *dec)
        {
            _decoder = dec;
        }

        bool
        pcBuffered(Addr pc) const
        {
            bool buffered = bufferedPCs.find(pc) != bufferedPCs.end()
                            && reservedPCs.find(pc) != reservedPCs.end();

            return buffered;
        }

        /**
         * calculates the number of fetched bytes that have yet
         * to be decoded.
         */
        int fetchBytesRemaining() const;

      private:
        void decodeSplitInst();

        /**
         * check if the next instruction to be processed out of
         * the fetch buffer is split across the end/beginning of
         * the fetch buffer.
         */
        bool splitDecode() const;

        /**
         * the set of PCs (fetch addresses) that are currently
         * buffered. bufferedPCs are valid, reservedPCs are
         * waiting for their buffers to be filled with valid
         * fetch data.
         */
        std::map<Addr, uint8_t*> bufferedPCs;
        std::map<Addr, uint8_t*> reservedPCs;

        /**
         * represents the fetch buffer free list. holds buffer space
         * that is currently free. each pointer in this array must
         * have enough space to hold a cache line. in reality we
         * have one actual fetch buffer: 'bufStart', these pointers
         * point to addresses within bufStart that are aligned to the
         * cache line size.
         */
        std::deque<uint8_t*> freeList;

        /**
         * raw instruction buffer. holds cache line data associated with
         * the set of PCs (fetch addresses) that are buffered here.
         */
        uint8_t *bufStart;
        uint8_t *bufEnd;
        /**
         * pointer that points to the next chunk of inst data to be
         * decoded.
         */
        uint8_t *readPtr;
        // how many lines the fetch unit may buffer
        int fetchDepth;
        // maximum size (in number of insts) of the WF's IB
        int maxIbSize;
        // maximum size (in bytes) of this fetch buffer
        int maxFbSize;
        int cacheLineSize;
        int cacheLineBits;
        bool restartFromBranch;
        // wavefront whose IB is serviced by this fetch buffer
        Wavefront *wavefront;
        TheGpuISA::Decoder *_decoder;
    };

    bool timingSim;
    ComputeUnit &computeUnit;
    TheGpuISA::Decoder decoder;

    // Fetch scheduler; Selects one wave from
    // the fetch queue for instruction fetching.
    // The selection is made according to
    // a scheduling policy
    Scheduler fetchScheduler;

    // Stores the list of waves that are
    // ready to be fetched this cycle
    std::vector<Wavefront*> fetchQueue;

    // Stores the fetch status of all waves dispatched to this SIMD.
    // TRUE implies the wave is ready to fetch and is already
    // moved to fetchQueue
    std::vector<std::pair<Wavefront*, bool>> fetchStatusQueue;

    // Pointer to list of waves dispatched on to this SIMD unit
    std::vector<Wavefront*> *waveList;
    // holds the fetch buffers. each wave has 1 entry.
    std::vector<FetchBufDesc> fetchBuf;
    /**
     * number of cache lines we can fetch and buffer.
     * this includes the currently fetched line (i.e., the
     * line that corresponds to the WF's current PC), as
     * well as any lines that may be prefetched.
     */
    int fetchDepth;
};

#endif // __FETCH_UNIT_HH__
