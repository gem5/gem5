/*
 * Copyright (c) 2017-2018 Advanced Micro Devices, Inc.
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
 *
 * Authors: Anthony Gutierrez
 */

/**
 * @file
 * HSAQueuEntry is the simulator's internal representation of an
 * AQL queue entry (task). It encasulates all of the relevant info
 * about a task, which is gathered from various runtime data
 * structures including: the AQL MQD, the AQL packet, and the code
 * object.
 */

#ifndef __GPU_COMPUTE_HSA_QUEUE_ENTRY__
#define __GPU_COMPUTE_HSA_QUEUE_ENTRY__

#include <bitset>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>

#include "base/intmath.hh"
#include "base/types.hh"
#include "dev/hsa/hsa_packet.hh"
#include "dev/hsa/hsa_queue.hh"
#include "gpu-compute/kernel_code.hh"

class HSAQueueEntry
{
  public:
    HSAQueueEntry(std::string kernel_name, uint32_t queue_id,
                  int dispatch_id, void *disp_pkt, AMDKernelCode *akc,
                  Addr host_pkt_addr, Addr code_addr)
        : kernName(kernel_name),
          _wgSize{{(int)((_hsa_dispatch_packet_t*)disp_pkt)->workgroup_size_x,
                  (int)((_hsa_dispatch_packet_t*)disp_pkt)->workgroup_size_y,
                  (int)((_hsa_dispatch_packet_t*)disp_pkt)->workgroup_size_z}},
          _gridSize{{(int)((_hsa_dispatch_packet_t*)disp_pkt)->grid_size_x,
                    (int)((_hsa_dispatch_packet_t*)disp_pkt)->grid_size_y,
                    (int)((_hsa_dispatch_packet_t*)disp_pkt)->grid_size_z}},
          numVgprs(akc->workitem_vgpr_count),
          numSgprs(akc->wavefront_sgpr_count),
          _queueId(queue_id), _dispatchId(dispatch_id), dispPkt(disp_pkt),
          _hostDispPktAddr(host_pkt_addr),
          _completionSignal(((_hsa_dispatch_packet_t*)disp_pkt)
                            ->completion_signal),
          codeAddress(code_addr),
          kernargAddress(((_hsa_dispatch_packet_t*)disp_pkt)->kernarg_address),
          _outstandingInvs(-1), _outstandingWbs(0),
          _ldsSize((int)((_hsa_dispatch_packet_t*)disp_pkt)->
                   group_segment_size),
          _privMemPerItem((int)((_hsa_dispatch_packet_t*)disp_pkt)->
                         private_segment_size),
          _contextId(0), _wgId{{ 0, 0, 0 }},
          _numWgTotal(1), numWgArrivedAtBarrier(0), _numWgCompleted(0),
          _globalWgId(0), dispatchComplete(false)

    {
        // Precompiled BLIT kernels actually violate the spec a bit
        // and don't set many of the required akc fields.  For these kernels,
        // we need to rip register usage from the resource registers.
        //
        // We can't get an exact number of registers from the resource
        // registers because they round, but we can get an upper bound on it
        if (!numVgprs)
            numVgprs = (akc->granulated_workitem_vgpr_count + 1) * 4;

        // TODO: Granularity changes for GFX9!
        if (!numSgprs)
            numSgprs = (akc->granulated_wavefront_sgpr_count + 1) * 8;

        initialVgprState.reset();
        initialSgprState.reset();

        for (int i = 0; i < MAX_DIM; ++i) {
            _numWg[i] = divCeil(_gridSize[i], _wgSize[i]);
            _numWgTotal *= _numWg[i];
        }

        parseKernelCode(akc);
    }

    const std::string&
    kernelName() const
    {
        return kernName;
    }

    int
    wgSize(int dim) const
    {
        assert(dim < MAX_DIM);
        return _wgSize[dim];
    }

    int
    gridSize(int dim) const
    {
        assert(dim < MAX_DIM);
        return _gridSize[dim];
    }

    int
    numVectorRegs() const
    {
        return numVgprs;
    }

    int
    numScalarRegs() const
    {
        return numSgprs;
    }

    uint32_t
    queueId() const
    {
        return _queueId;
    }

    int
    dispatchId() const
    {
        return _dispatchId;
    }

    void*
    dispPktPtr()
    {
        return dispPkt;
    }

    Addr
    hostDispPktAddr() const
    {
        return _hostDispPktAddr;
    }

    Addr
    completionSignal() const
    {
        return _completionSignal;
    }

    Addr
    codeAddr() const
    {
        return codeAddress;
    }

    Addr
    kernargAddr() const
    {
        return kernargAddress;
    }

    int
    ldsSize() const
    {
        return _ldsSize;
    }

    int privMemPerItem() const { return _privMemPerItem; }

    int
    contextId() const
    {
        return _contextId;
    }

    bool
    dispComplete() const
    {
        return dispatchComplete;
    }

    int
    wgId(int dim) const
    {
        assert(dim < MAX_DIM);
        return _wgId[dim];
    }

    void
    wgId(int dim, int val)
    {
        assert(dim < MAX_DIM);
        _wgId[dim] = val;
    }

    int
    globalWgId() const
    {
        return _globalWgId;
    }

    void
    globalWgId(int val)
    {
        _globalWgId = val;
    }

    int
    numWg(int dim) const
    {
        assert(dim < MAX_DIM);
        return _numWg[dim];
    }

    void
    notifyWgCompleted()
    {
        ++_numWgCompleted;
    }

    int
    numWgCompleted() const
    {
        return _numWgCompleted;
    }

    int
    numWgTotal() const
    {
        return _numWgTotal;
    }

    void
    markWgDispatch()
    {
        ++_wgId[0];
        ++_globalWgId;

        if (wgId(0) * wgSize(0) >= gridSize(0)) {
            _wgId[0] = 0;
            ++_wgId[1];

            if (wgId(1) * wgSize(1) >= gridSize(1)) {
                _wgId[1] = 0;
                ++_wgId[2];

                if (wgId(2) * wgSize(2) >= gridSize(2)) {
                    dispatchComplete = true;
                }
            }
        }
    }

    int
    numWgAtBarrier() const
    {
        return numWgArrivedAtBarrier;
    }

    bool vgprBitEnabled(int bit) const
    {
        return initialVgprState.test(bit);
    }

    bool sgprBitEnabled(int bit) const
    {
        return initialSgprState.test(bit);
    }

    /**
     * Host-side addr of the amd_queue_t on which
     * this task was queued.
     */
    Addr hostAMDQueueAddr;

    /**
     * Keep a copy of the AMD HSA queue because we
     * need info from some of its fields to initialize
     * register state.
     */
    _amd_queue_t amdQueue;

    // the maximum number of dimensions for a grid or workgroup
    const static int MAX_DIM = 3;

    /* getter */
    int
    outstandingInvs() {
        return _outstandingInvs;
    }

    /**
     * Whether invalidate has started or finished -1 is the
     * initial value indicating inv has not started for the
     * kernel.
     */
    bool
    isInvStarted()
    {
        return (_outstandingInvs != -1);
    }

    /**
     * update the number of pending invalidate requests
     *
     * val: negative to decrement, positive to increment
     */
    void
    updateOutstandingInvs(int val)
    {
        _outstandingInvs += val;
        assert(_outstandingInvs >= 0);
    }

    /**
     * Forcefully change the state to be inv done.
     */
    void
    markInvDone()
    {
        _outstandingInvs = 0;
    }

    /**
     * Is invalidate done?
     */
    bool
    isInvDone() const
    {
        assert(_outstandingInvs >= 0);
        return (_outstandingInvs == 0);
    }

    int
    outstandingWbs() const
    {
        return _outstandingWbs;
    }

    /**
     * Update the number of pending writeback requests.
     *
     * val: negative to decrement, positive to increment
     */
    void
    updateOutstandingWbs(int val)
    {
        _outstandingWbs += val;
        assert(_outstandingWbs >= 0);
    }

  private:
    void
    parseKernelCode(AMDKernelCode *akc)
    {
        /** set the enable bits for the initial SGPR state */
        initialSgprState.set(PrivateSegBuf,
            akc->enable_sgpr_private_segment_buffer);
        initialSgprState.set(DispatchPtr,
            akc->enable_sgpr_dispatch_ptr);
        initialSgprState.set(QueuePtr,
            akc->enable_sgpr_queue_ptr);
        initialSgprState.set(KernargSegPtr,
            akc->enable_sgpr_kernarg_segment_ptr);
        initialSgprState.set(DispatchId,
            akc->enable_sgpr_dispatch_id);
        initialSgprState.set(FlatScratchInit,
            akc->enable_sgpr_flat_scratch_init);
        initialSgprState.set(PrivateSegSize,
            akc->enable_sgpr_private_segment_size);
        initialSgprState.set(GridWorkgroupCountX,
            akc->enable_sgpr_grid_workgroup_count_x);
        initialSgprState.set(GridWorkgroupCountY,
            akc->enable_sgpr_grid_workgroup_count_y);
        initialSgprState.set(GridWorkgroupCountZ,
            akc->enable_sgpr_grid_workgroup_count_z);
        initialSgprState.set(WorkgroupIdX,
            akc->enable_sgpr_workgroup_id_x);
        initialSgprState.set(WorkgroupIdY,
            akc->enable_sgpr_workgroup_id_y);
        initialSgprState.set(WorkgroupIdZ,
            akc->enable_sgpr_workgroup_id_z);
        initialSgprState.set(WorkgroupInfo,
            akc->enable_sgpr_workgroup_info);
        initialSgprState.set(PrivSegWaveByteOffset,
            akc->enable_sgpr_private_segment_wave_byte_offset);

        /**
         * set the enable bits for the initial VGPR state. the
         * workitem Id in the X dimension is always initialized.
         */
        initialVgprState.set(WorkitemIdX, true);
        initialVgprState.set(WorkitemIdY, akc->enable_vgpr_workitem_id > 0);
        initialVgprState.set(WorkitemIdZ, akc->enable_vgpr_workitem_id > 1);
    }

    // name of the kernel associated with the AQL entry
    std::string kernName;
    // workgroup Size (3 dimensions)
    std::array<int, MAX_DIM> _wgSize;
    // grid Size (3 dimensions)
    std::array<int, MAX_DIM> _gridSize;
    // total number of VGPRs per work-item
    int numVgprs;
    // total number of SGPRs per wavefront
    int numSgprs;
    // id of AQL queue in which this entry is placed
    uint32_t _queueId;
    int _dispatchId;
    // raw AQL packet pointer
    void *dispPkt;
    // host-side addr of the dispatch packet
    Addr _hostDispPktAddr;
    // pointer to bool
    Addr _completionSignal;
    // base address of the raw machine code
    Addr codeAddress;
    // base address of the kernel args
    Addr kernargAddress;
    /**
     * Number of outstanding invs for the kernel.
     * values:
     *  -1: initial value, invalidate has not started for the kernel
     *  0: 1)-1->0, about to start (a transient state, added in the same cycle)
     *     2)+1->0, all inv requests are finished, i.e., invalidate done
     *  ?: positive value, indicating the number of pending inv requests
     */
    int _outstandingInvs;
    /**
     * Number of outstanding wbs for the kernel
     * values:
     *  0: 1)initial value, flush has not started for the kernel
     *     2)+1->0: all wb requests are finished, i.e., flush done
     *  ?: positive value, indicating the number of pending wb requests
     */
    int _outstandingWbs;
    int _ldsSize;
    int _privMemPerItem;
    int _contextId;
    std::array<int, MAX_DIM> _wgId;
    std::array<int, MAX_DIM> _numWg;
    int _numWgTotal;
    int numWgArrivedAtBarrier;
    // The number of completed work groups
    int _numWgCompleted;
    int _globalWgId;
    bool dispatchComplete;

    std::bitset<NumVectorInitFields> initialVgprState;
    std::bitset<NumScalarInitFields> initialSgprState;
};

#endif // __GPU_COMPUTE_HSA_QUEUE_ENTRY__
