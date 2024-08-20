/*
 * Copyright (c) 2021 Advanced Micro Devices, Inc.
 * All rights reserved.
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
 */

#ifndef __DEV_AMDGPU_PM4_QUEUES_HH__
#define __DEV_AMDGPU_PM4_QUEUES_HH__

#include "dev/amdgpu/pm4_defines.hh"

namespace gem5
{

/**
 * Queue descriptor with relevant MQD attributes. Taken from
 * https://github.com/RadeonOpenCompute/ROCK-Kernel-Driver/blob/roc-4.3.x/
 *     drivers/gpu/drm/amd/include/v9_structs.h
 */
typedef struct GEM5_PACKED
{
    union
    {
        struct
        {
            uint32_t cp_mqd_readindex_lo;
            uint32_t cp_mqd_readindex_hi;
        };
        uint64_t mqdReadIndex;
    };
    uint32_t cp_mqd_save_start_time_lo;
    uint32_t cp_mqd_save_start_time_hi;
    uint32_t cp_mqd_save_end_time_lo;
    uint32_t cp_mqd_save_end_time_hi;
    uint32_t cp_mqd_restore_start_time_lo;
    uint32_t cp_mqd_restore_start_time_hi;
    uint32_t cp_mqd_restore_end_time_lo;
    uint32_t cp_mqd_restore_end_time_hi;
    uint32_t disable_queue;
    uint32_t reserved_107;
    uint32_t gds_cs_ctxsw_cnt0;
    uint32_t gds_cs_ctxsw_cnt1;
    uint32_t gds_cs_ctxsw_cnt2;
    uint32_t gds_cs_ctxsw_cnt3;
    uint32_t reserved_112;
    uint32_t reserved_113;
    uint32_t cp_pq_exe_status_lo;
    uint32_t cp_pq_exe_status_hi;
    uint32_t cp_packet_id_lo;
    uint32_t cp_packet_id_hi;
    uint32_t cp_packet_exe_status_lo;
    uint32_t cp_packet_exe_status_hi;
    uint32_t gds_save_base_addr_lo;
    uint32_t gds_save_base_addr_hi;
    uint32_t gds_save_mask_lo;
    uint32_t gds_save_mask_hi;
    uint32_t ctx_save_base_addr_lo;
    uint32_t ctx_save_base_addr_hi;
    uint32_t dynamic_cu_mask_addr_lo;
    uint32_t dynamic_cu_mask_addr_hi;
    union
    {
        struct
        {
            uint32_t mqd_base_addr_lo;
            uint32_t mqd_base_addr_hi;
        };
        uint64_t mqdBase;
    };
    uint32_t hqd_active;
    uint32_t hqd_vmid;
    uint32_t hqd_persistent_state;
    uint32_t hqd_pipe_priority;
    uint32_t hqd_queue_priority;
    uint32_t hqd_quantum;
    union
    {
        struct
        {
            uint32_t hqd_pq_base_lo;
            uint32_t hqd_pq_base_hi;
        };
        uint64_t base;
    };
    union
    {
        uint32_t hqd_pq_rptr;
        uint32_t rptr;
    };
    union
    {
        struct
        {
            uint32_t hqd_pq_rptr_report_addr_lo;
            uint32_t hqd_pq_rptr_report_addr_hi;
        };
        uint64_t aqlRptr;
    };
    uint32_t hqd_pq_wptr_poll_addr_lo;
    uint32_t hqd_pq_wptr_poll_addr_hi;
    union
    {
        uint32_t hqd_pq_doorbell_control;
        uint32_t doorbell;
    };
    uint32_t reserved_144;
    uint32_t hqd_pq_control;
    union
    {
        struct
        {
            uint32_t hqd_ib_base_addr_lo;
            uint32_t hqd_ib_base_addr_hi;
        };
        Addr ibBase;
    };
    union
    {
        uint32_t hqd_ib_rptr;
        uint32_t ibRptr;
    };
    uint32_t hqd_ib_control;
    uint32_t hqd_iq_timer;
    uint32_t hqd_iq_rptr;
    uint32_t cp_hqd_dequeue_request;
    uint32_t cp_hqd_dma_offload;
    uint32_t cp_hqd_sema_cmd;
    uint32_t cp_hqd_msg_type;
    uint32_t cp_hqd_atomic0_preop_lo;
    uint32_t cp_hqd_atomic0_preop_hi;
    uint32_t cp_hqd_atomic1_preop_lo;
    uint32_t cp_hqd_atomic1_preop_hi;
    uint32_t cp_hqd_hq_status0;
    uint32_t cp_hqd_hq_control0;
    uint32_t cp_mqd_control;
    uint32_t cp_hqd_hq_status1;
    uint32_t cp_hqd_hq_control1;
    uint32_t cp_hqd_eop_base_addr_lo;
    uint32_t cp_hqd_eop_base_addr_hi;
    uint32_t cp_hqd_eop_control;
    uint32_t cp_hqd_eop_rptr;
    uint32_t cp_hqd_eop_wptr;
    uint32_t cp_hqd_eop_done_events;
    uint32_t cp_hqd_ctx_save_base_addr_lo;
    uint32_t cp_hqd_ctx_save_base_addr_hi;
    uint32_t cp_hqd_ctx_save_control;
    uint32_t cp_hqd_cntl_stack_offset;
    uint32_t cp_hqd_cntl_stack_size;
    uint32_t cp_hqd_wg_state_offset;
    uint32_t cp_hqd_ctx_save_size;
    uint32_t cp_hqd_gds_resource_state;
    uint32_t cp_hqd_error;
    uint32_t cp_hqd_eop_wptr_mem;
    union
    {
        uint32_t cp_hqd_aql_control;
        uint32_t aql;
    };
    uint32_t cp_hqd_pq_wptr_lo;
    uint32_t cp_hqd_pq_wptr_hi;
} QueueDesc;

/**
 * Queue descriptor for SDMA-based user queues (RLC queues). Taken from
 * https://github.com/RadeonOpenCompute/ROCK-Kernel-Driver/blob/roc-4.3.x/
 *     drivers/gpu/drm/amd/include/v9_structs.h
 */
typedef struct GEM5_PACKED
{
    uint32_t sdmax_rlcx_rb_cntl;
    union
    {
        struct
        {
            uint32_t sdmax_rlcx_rb_base;
            uint32_t sdmax_rlcx_rb_base_hi;
        };
        uint64_t rb_base;
    };
    union
    {
        struct
        {
            uint32_t sdmax_rlcx_rb_rptr;
            uint32_t sdmax_rlcx_rb_rptr_hi;
        };
        uint64_t rptr;
    };
    union
    {
        struct
        {
            uint32_t sdmax_rlcx_rb_wptr;
            uint32_t sdmax_rlcx_rb_wptr_hi;
        };
        uint64_t wptr;
    };
    uint32_t sdmax_rlcx_rb_wptr_poll_cntl;
    uint32_t sdmax_rlcx_rb_rptr_addr_hi;
    uint32_t sdmax_rlcx_rb_rptr_addr_lo;
    uint32_t sdmax_rlcx_ib_cntl;
    uint32_t sdmax_rlcx_ib_rptr;
    uint32_t sdmax_rlcx_ib_offset;
    uint32_t sdmax_rlcx_ib_base_lo;
    uint32_t sdmax_rlcx_ib_base_hi;
    uint32_t sdmax_rlcx_ib_size;
    uint32_t sdmax_rlcx_skip_cntl;
    uint32_t sdmax_rlcx_context_status;
    uint32_t sdmax_rlcx_doorbell;
    uint32_t sdmax_rlcx_status;
    uint32_t sdmax_rlcx_doorbell_log;
    uint32_t sdmax_rlcx_watermark;
    uint32_t sdmax_rlcx_doorbell_offset;
    uint32_t sdmax_rlcx_csa_addr_lo;
    uint32_t sdmax_rlcx_csa_addr_hi;
    uint32_t sdmax_rlcx_ib_sub_remain;
    uint32_t sdmax_rlcx_preempt;
    uint32_t sdmax_rlcx_dummy_reg;
    uint32_t sdmax_rlcx_rb_wptr_poll_addr_hi;
    uint32_t sdmax_rlcx_rb_wptr_poll_addr_lo;
    uint32_t sdmax_rlcx_rb_aql_cntl;
    uint32_t sdmax_rlcx_minor_ptr_update;
    uint32_t sdmax_rlcx_midcmd_data0;
    uint32_t sdmax_rlcx_midcmd_data1;
    uint32_t sdmax_rlcx_midcmd_data2;
    uint32_t sdmax_rlcx_midcmd_data3;
    uint32_t sdmax_rlcx_midcmd_data4;
    uint32_t sdmax_rlcx_midcmd_data5;
    uint32_t sdmax_rlcx_midcmd_data6;
    uint32_t sdmax_rlcx_midcmd_data7;
    uint32_t sdmax_rlcx_midcmd_data8;
    uint32_t sdmax_rlcx_midcmd_cntl;
    uint32_t reserved_42;
    uint32_t reserved_43;
    uint32_t reserved_44;
    uint32_t reserved_45;
    uint32_t reserved_46;
    uint32_t reserved_47;
    uint32_t reserved_48;
    uint32_t reserved_49;
    uint32_t reserved_50;
    uint32_t reserved_51;
    uint32_t reserved_52;
    uint32_t reserved_53;
    uint32_t reserved_54;
    uint32_t reserved_55;
    uint32_t reserved_56;
    uint32_t reserved_57;
    uint32_t reserved_58;
    uint32_t reserved_59;
    uint32_t reserved_60;
    uint32_t reserved_61;
    uint32_t reserved_62;
    uint32_t reserved_63;
    uint32_t reserved_64;
    uint32_t reserved_65;
    uint32_t reserved_66;
    uint32_t reserved_67;
    uint32_t reserved_68;
    uint32_t reserved_69;
    uint32_t reserved_70;
    uint32_t reserved_71;
    uint32_t reserved_72;
    uint32_t reserved_73;
    uint32_t reserved_74;
    uint32_t reserved_75;
    uint32_t reserved_76;
    uint32_t reserved_77;
    uint32_t reserved_78;
    uint32_t reserved_79;
    uint32_t reserved_80;
    uint32_t reserved_81;
    uint32_t reserved_82;
    uint32_t reserved_83;
    uint32_t reserved_84;
    uint32_t reserved_85;
    uint32_t reserved_86;
    uint32_t reserved_87;
    uint32_t reserved_88;
    uint32_t reserved_89;
    uint32_t reserved_90;
    uint32_t reserved_91;
    uint32_t reserved_92;
    uint32_t reserved_93;
    uint32_t reserved_94;
    uint32_t reserved_95;
    uint32_t reserved_96;
    uint32_t reserved_97;
    uint32_t reserved_98;
    uint32_t reserved_99;
    uint32_t reserved_100;
    uint32_t reserved_101;
    uint32_t reserved_102;
    uint32_t reserved_103;
    uint32_t reserved_104;
    uint32_t reserved_105;
    uint32_t reserved_106;
    uint32_t reserved_107;
    uint32_t reserved_108;
    uint32_t reserved_109;
    uint32_t reserved_110;
    uint32_t reserved_111;
    uint32_t reserved_112;
    uint32_t reserved_113;
    uint32_t reserved_114;
    uint32_t reserved_115;
    uint32_t reserved_116;
    uint32_t reserved_117;
    uint32_t reserved_118;
    uint32_t reserved_119;
    uint32_t reserved_120;
    uint32_t reserved_121;
    uint32_t reserved_122;
    uint32_t reserved_123;
    uint32_t reserved_124;
    uint32_t reserved_125;
    /* reserved_126,127: repurposed for driver-internal use */
    uint32_t sdma_engine_id;
    uint32_t sdma_queue_id;
} SDMAQueueDesc;

/* The Primary Queue has extra attributes, which will be stored separately. */
typedef struct PrimaryQueue : QueueDesc
{
    union
    {
        struct
        {
            uint32_t queueRptrAddrLo;
            uint32_t queueRptrAddrHi;
        };
        Addr queueRptrAddr;
    };
    union
    {
        struct
        {
            uint32_t queueWptrLo;
            uint32_t queueWptrHi;
        };
        Addr queueWptr;
    };
    uint32_t doorbellOffset;
    uint32_t doorbellRangeLo;
    uint32_t doorbellRangeHi;
} PrimaryQueue;

/**
 * Class defining a PM4 queue.
 */
class PM4Queue
{
    int _id;

    /* Queue descriptor read from the system memory of the simulated system. */
    QueueDesc *q;

    /**
     * Most important fields of a PM4 queue are stored in the queue descriptor
     * (i.e., QueueDesc). However, since the write pointers are communicated
     * through the doorbell value, we will add separate atributes for them.
     */
    Addr _wptr;
    Addr _ibWptr;
    Addr _offset;
    bool _processing;
    bool _ib;
    PM4MapQueues _pkt;
  public:
    PM4Queue() : _id(0), q(nullptr), _wptr(0), _offset(0), _processing(false),
        _ib(false), _pkt() {}
    PM4Queue(int id, QueueDesc *queue, Addr offset) :
        _id(id), q(queue), _wptr(queue->rptr), _ibWptr(0), _offset(offset),
        _processing(false), _ib(false), _pkt() {}
    PM4Queue(int id, QueueDesc *queue, Addr offset, PM4MapQueues *pkt) :
        _id(id), q(queue), _wptr(queue->rptr), _ibWptr(0), _offset(offset),
        _processing(false), _ib(false), _pkt(*pkt) {}

    QueueDesc *getMQD() { return q; }
    int id() { return _id; }
    Addr mqdBase() { return q->mqdBase; }
    Addr base() { return q->base; }
    Addr ibBase() { return q->ibBase; }

    Addr
    rptr()
    {
        if (ib()) return q->ibBase + q->ibRptr;
        else return q->base + (q->rptr % size());
    }

    Addr
    wptr()
    {
        if (ib()) return q->ibBase + _ibWptr;
        else return q->base + (_wptr % size());
    }

    Addr
    getRptr()
    {
        if (ib()) return q->ibRptr;
        else return q->rptr;
    }

    Addr
    getWptr()
    {
        if (ib()) return _ibWptr;
        else return _wptr;
    }

    Addr offset() { return _offset; }
    bool processing() { return _processing; }
    bool ib() { return _ib; }

    void id(int value) { _id = value; }
    void base(Addr value) { q->base = value; }
    void ibBase(Addr value) { q->ibBase = value; }

    /**
     * It seems that PM4 nop packets with count 0x3fff, not only do not
     * consider the count value, they also fast forward the read pointer.
     * Without proper sync packets this can potentially be dangerous, since
     * more useful packets can be enqueued in the time between nop enqueu and
     * nop processing.
     */
    void
    fastforwardRptr()
    {
        if (ib()) q->ibRptr = _ibWptr;
        else q->rptr = _wptr;
    }

    void
    incRptr(Addr value)
    {
        if (ib()) q->ibRptr += value;
        else q->rptr += value;
    }

    void
    rptr(Addr value)
    {
        if (ib()) q->ibRptr = value;
        else q->rptr = value;
    }

    void
    wptr(Addr value)
    {
        if (ib()) _ibWptr = value;
        else _wptr = value;
    }

    void offset(Addr value) { _offset = value; }
    void processing(bool value) { _processing = value; }
    void ib(bool value) { _ib = value; }
    uint32_t me() { return _pkt.me + 1; }
    uint32_t pipe() { return _pkt.pipe; }
    uint32_t queue() { return _pkt.queueSlot; }
    bool privileged() { return _pkt.queueSel == 0 ? 1 : 0; }
    uint32_t queueType() { return _pkt.queueType; }
    bool isStatic() { return (_pkt.queueType != 0); }
    PM4MapQueues* getPkt() { return &_pkt; }
    void setPkt(uint32_t me, uint32_t pipe, uint32_t queue, bool privileged,
                uint32_t queueType) {
        _pkt.me = me - 1;
        _pkt.pipe = pipe;
        _pkt.queueSlot = queue;
        _pkt.queueSel = (privileged == 0) ? 1 : 0;
        _pkt.queueType = queueType;
    }

    // Same computation as processMQD. See comment there for details.
    uint64_t size() { return 4UL << ((q->hqd_pq_control & 0x3f) + 1); }
};

} // namespace gem5

#endif // __DEV_AMDGPU_PM4_QUEUES_HH__
