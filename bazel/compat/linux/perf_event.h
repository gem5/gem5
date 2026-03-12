#ifndef __BAZEL_COMPAT_LINUX_PERF_EVENT_H__
#define __BAZEL_COMPAT_LINUX_PERF_EVENT_H__

#include <stdint.h>
#include "../kvm_compat.hh"

struct perf_event_attr {
    uint32_t type;
    uint32_t size;
    uint64_t config;
    union {
        uint64_t sample_period;
        uint64_t sample_freq;
    };
    uint64_t sample_type;
    uint64_t read_format;
    uint64_t disabled       :  1,
             inherit        :  1,
             pinned         :  1,
             exclusive      :  1,
             exclude_user   :  1,
             exclude_kernel :  1,
             exclude_hv     :  1,
             exclude_idle   :  1,
             mmap           :  1,
             comm           :  1,
             freq           :  1,
             inherit_stat   :  1,
             enable_on_exec :  1,
             task           :  1,
             watermark      :  1,
             precise_ip     :  2,
             mmap_data      :  1,
             sample_id_all  :  1,
             exclude_host   :  1,
             exclude_guest  :  1,
             exclude_callchain_kernel : 1,
             exclude_callchain_user   : 1,
             mmap2          :  1,
             comm_exec      :  1,
             use_clockid    :  1,
             context_switch :  1,
             write_backward :  1,
             namespaces     :  1,
             __reserved_1   : 35;
    union {
        uint32_t wakeup_events;
        uint32_t wakeup_watermark;
    };
    uint32_t bp_type;
    union {
        uint64_t bp_addr;
        uint64_t config1;
    };
    union {
        uint64_t bp_len;
        uint64_t config2;
    };
    uint64_t branch_sample_type;
    uint64_t sample_regs_user;
    uint32_t sample_stack_user;
    int32_t  clockid;
    uint64_t sample_regs_intr;
    uint32_t aux_watermark;
    uint16_t sample_max_stack;
    uint16_t __reserved_2;
};

#endif
