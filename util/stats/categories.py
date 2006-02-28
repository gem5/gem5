# Copyright (c) 2005-2006 The Regents of The University of Michigan
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Nathan Binkert

func_categories = { \
    # Buffer management functions
    '__skb_linearize' : 'buffer',
    'skb_clone' : 'buffer',
    'skb_clone_fraglist' : 'buffer',
    'skb_seq_read' : 'buffer',
    'sock_alloc_send_skb' : 'buffer',
    'sinic_rxskb_alloc' : 'buffer',

    # Copy functions
    'sinic_copyfrom' : 'copy',
    '__copy_user' : 'copy',
    'skb_copy_bits' : 'copy',
    'skb_copy_datagram_iovec' : 'copy',
    'sinic_vcopy_iov' : 'idle',

    # Driver functions
    'do_tx_done' : 'driver',
    'ns83820_get_drvinfo' : 'driver',
    'ns83820_get_stats' : 'driver',
    'ns83820_hard_start_xmit' : 'driver',
    'ns83820_open' : 'driver',
    'ns83820_rx_kick' : 'driver',
    'ns83820_update_stats' : 'driver',
    'ns83820_irq' : 'driver',
    'phy_intr' : 'driver',
    'rx_irq' : 'driver',
    'rx_action' : 'driver',
    'sinic_intr' : 'driver',
    'sinic_xmit' : 'driver',
    'sinic_rxskb_new' : 'driver',

    # Idle functions
    'cpu_idle' : 'idle',

    # Interrupt functions
    'do_entInt' : 'interrupt',
    'entInt' : 'interrupt',
    'handle_IRQ_event' : 'interrupt',
    'irq_exit' : 'interrupt',

    # Other functions
    'ret_from_sys_call' : 'other',
    'top' : 'other',

    # Stack functions
    '__ip_conntrack_confirm' : 'stack',
    '__ip_conntrack_find' : 'stack',
    '__tcp_ack_snd_check' : 'stack',
    '__tcp_checksum_complete_user' : 'stack',
    'dev_queue_xmit' : 'stack',
    'eth_header_cache' : 'stack',
    'ether_setup' : 'stack',
    'icmp_error' : 'stack',
    'ip_call_ra_chain' : 'stack',
    'ip_conntrack_alter_reply' : 'stack',
    'ip_conntrack_tcp_update' : 'stack',
    'ip_ct_find_helper' : 'stack',
    'ip_finish_output' : 'stack',
    'ip_finish_output2' : 'stack',
    'ip_local_deliver_finish' : 'stack',
    'ip_nat_setup_info' : 'stack',
    'ip_rcv' : 'stack',
    'ip_rcv_finish' : 'stack',
    'netif_receive_skb' : 'stack',
    'nf_log_packet' : 'stack',
    'nf_queue' : 'stack',
    'tcp_connect' : 'stack',
    'tcp_data_queue' : 'stack',
    'tcp_packet' : 'stack',
    'tcp_read_sock' : 'stack',
    'tcp_rcv_established' : 'stack',
    'tcp_recvmsg' : 'stack',
    'tcp_sendpage' : 'stack',
    'tcp_transmit_skb' : 'stack',
    'tcp_v4_do_rcv' : 'stack',
    'unregister_netdevice' : 'stack',

    # Syscall functions
    'entSys' : 'syscall',

    # User functions
    'user' : 'user',
    }

def func_categorize(symbol):
    from categories import func_categories
    if symbol in func_categories:
        return func_categories[symbol]
    return None


pc_categories = {
    'CALL_PALrdunique_' : 'interrupt', #
    'Call_Pal_Callsys' : 'interrupt', #
    'Call_Pal_Rdps' : 'interrupt', #
    'Call_Pal_Rdusp' : 'interrupt', #
    'Call_Pal_Rti' : 'interrupt', #
    'Call_Pal_Swpctx' : 'interrupt', #
    'Call_Pal_Swpipl' : 'interrupt', #
    'Call_Pal_Wrusp' : 'interrupt', #
    'SHATransform': 'driver', # drivers/char/random.c,
    'TRAP_INTERRUPT_10_' : 'interrupt', #
    'Trap_Dtbmiss_Single' : 'buffer', #
    'Trap_Dtbmiss_double' : 'buffer', #
    'Trap_Interrupt' : 'interrupt', #
    'Trap_Itbmiss' : 'buffer', #
    'Trap_Unalign' : 'alignment',
    'UNALIGN_NO_DISMISS' : 'alignment',
    'UNALIGN_NO_DISMISS_10_' : 'alignment',
    '__alloc_pages' : 'buffer', # mm/page_alloc.c,
    '__anon_vma_link': 'buffer', # mm/rmap.c, include/linux/rmap.h,
    '__bio_add_page' : 'other', # fs/bio.c,
    '__bitmap_weight' : 'other', # lib/bitmap.c, include/linux/bitmap.h,
    '__blk_put_request' : 'other', # drivers/block/ll_rw_blk.c,
    '__block_commit_write' : 'other', # fs/buffer.c,
    '__block_prepare_write' : 'other', # fs/buffer.c,
    '__block_write_full_page': 'other', # fs/buffer.c,
    '__bread' : 'other', # fs/buffer.c, include/linux/buffer_head.h,
    '__brelse' : 'other', # fs/buffer.c, include/linux/buffer_head.h,
    '__bss_start' : 'user',
    '__bss_stop' : 'other', # include/asm-generic/sections.h,
    '__cond_resched' : 'other', # kernel/sched.c, include/linux/sched.h,
    '__const_udelay': 'other', # include/asm-i386/delay.h,
    '__constant_c_memset' : 'other', # include/asm-alpha/string.h,
    '__copy_from_user_ll': 'copy', # include/asm-i386/uaccess.h,
    '__copy_to_user_ll': 'copy', # include/asm-i386/uaccess.h,
    '__copy_user' : 'copy', # include/asm-alpha/uaccess.h,
    '__d_lookup' : 'other', # fs/dcache.c, include/linux/dcache.h,
    '__d_path': 'other', # fs/dcache.c,
    '__delay': 'other', # arch/alpha/lib/udelay.c, include/asm-alpha/delay.h, include/asm-i386/delay.h,
    '__dequeue_signal' : 'other', # kernel/signal.c,
    '__divl' : 'other', # arch/alpha/kernel/alpha_ksyms.c,
    '__divlu' : 'other', # arch/alpha/kernel/alpha_ksyms.c,
    '__divq' : 'other', # arch/alpha/kernel/alpha_ksyms.c,
    '__divqu' : 'other', # arch/alpha/kernel/alpha_ksyms.c,
    '__do_softirq' : 'stack', # kernel/softirq.c,
    '__down': 'interrupt', # include/asm-alpha/semaphore.h, include/asm-i386/semaphore.h,
    '__down_failed' : 'other', # arch/alpha/kernel/semaphore.c, include/asm-alpha/semaphore.h,
    '__down_trylock': 'interrupt', # include/asm-alpha/semaphore.h, include/asm-i386/semaphore.h,
    '__elv_add_request' : 'other', # drivers/block/elevator.c, include/linux/elevator.h,
    '__end_that_request_first' : 'other', # drivers/block/ll_rw_blk.c,
    '__exit_sighand': 'other', # kernel/signal.c, include/linux/sched.h,
    '__exit_signal': 'other', # kernel/signal.c, include/linux/sched.h,
    '__filemap_copy_from_user_iovec' : 'buffer', # mm/filemap.c,
    '__filemap_fdatawrite' : 'buffer', # mm/filemap.c,
    '__find_get_block' : 'other', # fs/buffer.c, include/linux/buffer_head.h,
    '__find_get_block_slow' : 'other', # fs/buffer.c,
    '__fput' : 'other', # fs/file_table.c,
    '__free_pages' : 'buffer', # mm/page_alloc.c,
    '__free_pages_ok': 'buffer', # mm/page_alloc.c,
    '__generic_file_aio_read': 'buffer', # mm/filemap.c, include/linux/fs.h,
    '__generic_unplug_device' : 'other', # drivers/block/ll_rw_blk.c, include/linux/blkdev.h,
    '__get_free_pages' : 'other', # mm/page_alloc.c, drivers/md/raid6.h,
    '__get_page_state': 'buffer', # mm/page_alloc.c,
    '__get_user_4': 'other', # include/asm-i386/uaccess.h,
    '__get_zone_counts': 'other', #
    '__getblk' : 'other', # fs/buffer.c, include/linux/buffer_head.h,
    '__getblk_slow' : 'other', # fs/buffer.c,
    '__group_complete_signal' : 'user', # kernel/signal.c,  is kinda syscall
    '__group_send_sig_info' : 'user', # kernel/signal.c,  is kinda syscall
    '__iget' : 'other', # fs/inode.c, include/linux/fs.h,
    '__insert_inode_hash': 'other', # fs/inode.c, include/linux/fs.h,
    '__insert_vm_struct': 'buffer', # mm/mmap.c,
    '__ip_conntrack_confirm' : 'stack', # net/ipv4/netfilter/ip_conntrack_core.c, include/linux/netfilter_ipv4/ip_conntrack_core.h,
    '__ip_conntrack_find' : 'stack', # net/ipv4/netfilter/ip_conntrack_core.c,
    '__ip_ct_find_proto' : 'stack', # net/ipv4/netfilter/ip_conntrack_core.c, include/linux/netfilter_ipv4/ip_conntrack_core.h,
    '__ip_route_output_key' : 'stack', # net/ipv4/route.c,
    '__kfree_skb' : 'buffer', # net/core/skbuff.c, include/linux/skbuff.h,
    '__kmalloc' : 'buffer', # mm/slab.c, include/linux/slab.h,
    '__load_new_mm_context': 'buffer',
    '__lookup': 'other', # lib/radix-tree.c,
    '__lookup_hash': 'other', # fs/namei.c,
    '__lookup_tag' : 'buffer', # lib/radix-tree.c,
    '__make_request' : 'driver', # drivers/block/ll_rw_blk.c, drivers/block/ll_rw_blk.c,
    '__mark_inode_dirty' : 'other', # fs/fs-writeback.c, include/linux/fs.h,
    '__memcpy_aligned_up' : 'copy', # arch/alpha/lib/memcpy.c,
    '__memcpy_unaligned_up' : 'copy', # arch/alpha/lib/memcpy.c,
    '__memset' : 'copy', # include/asm-alpha/string.h,
    '__mmdrop': 'other', # kernel/fork.c,
    '__mod_timer' : 'other', # kernel/timer.c, include/linux/timer.h,
    '__modify_IO_APIC_irq': 'interrupt', #
    '__net_random': 'other', #
    '__page_cache_release' : 'buffer', # mm/swap.c,
    '__pagevec_free': 'buffer', # mm/page_alloc.c, include/linux/pagevec.h,
    '__pagevec_lru_add' : 'buffer', # mm/swap.c, include/linux/pagevec.h,
    '__pagevec_lru_add_active': 'buffer', # mm/swap.c, include/linux/pagevec.h,
    '__pagevec_release' : 'buffer', # mm/swap.c, include/linux/pagevec.h,
    '__pollwait' : 'other', # fs/select.c, fs/select.c,
    '__pskb_trim_head': 'stack', # net/ipv4/tcp_output.c,
    '__put_task_struct': 'other', # kernel/fork.c, include/linux/sched.h,
    '__queue_work': 'other', # kernel/workqueue.c,
    '__rb_erase_color' : 'buffer', # lib/rbtree.c,
    '__rb_rotate_left' : 'buffer', # lib/rbtree.c,
    '__rb_rotate_right' : 'buffer', # lib/rbtree.c,
    '__rcu_process_callbacks': 'other', #
    '__read_page_state' : 'buffer', # mm/page_alloc.c, include/linux/page-flags.h,
    '__release_sock' : 'stack', # net/core/sock.c,
    '__remlu' : 'other', # arch/alpha/kernel/alpha_ksyms.c,
    '__remove_from_page_cache': 'buffer', # mm/filemap.c, include/linux/pagemap.h,
    '__remove_shared_vm_struct': 'buffer', # mm/mmap.c,
    '__remqu' : 'other', # arch/alpha/kernel/alpha_ksyms.c,
    '__rmqueue' : 'buffer', # mm/page_alloc.c,
    '__scsi_done' : 'other', # drivers/scsi/scsi.c, drivers/scsi/scsi_priv.h,
    '__scsi_get_command' : 'other', # drivers/scsi/scsi.c,
    '__set_page_buffers' : 'other', # fs/buffer.c,
    '__set_page_dirty_nobuffers' : 'buffer', # mm/page-writeback.c, include/linux/mm.h,
    '__sk_stream_mem_reclaim' : 'buffer', # net/core/stream.c,
    '__sock_create': 'stack', # net/socket.c,
    '__strncpy_from_user' : 'copy', # include/asm-alpha/uaccess.h,
    '__strnlen_user': 'user',
    '__switch_to': 'interrupt', #
    '__sync_single_inode' : 'other', # fs/fs-writeback.c,
    '__tasklet_schedule' : 'other', # kernel/softirq.c,
    '__tcp_ack_snd_check' : 'stack', # net/ipv4/tcp_input.c,
    '__tcp_data_snd_check' : 'stack', # net/ipv4/tcp_input.c,
    '__tcp_grow_window' : 'stack', # net/ipv4/tcp_input.c,
    '__tcp_put_port' : 'stack', # net/ipv4/tcp_ipv4.c,
    '__tcp_select_window' : 'stack', # net/ipv4/tcp_output.c,
    '__tcp_tw_hashdance' : 'stack', # net/ipv4/tcp_minisocks.c,
    '__tcp_v4_check_established':'stack',
    '__unhash_process': 'other', # kernel/exit.c,
    '__unmask_IO_APIC_irq': 'interrupt', #
    '__up_wakeup' : 'interrupt', # arch/alpha/kernel/semaphore.c, include/asm-alpha/semaphore.h,
    '__user_walk' : 'other', # fs/namei.c,
    '__vm_stat_account': 'other', #
    '__vma_link': 'buffer', # mm/mmap.c,
    '__vma_link_rb': 'buffer', # mm/mmap.c, include/linux/mm.h,
    '__wait_on_buffer' : 'other', # fs/buffer.c, include/linux/buffer_head.h,
    '__wake_up' : 'other', # kernel/sched.c,
    '__wake_up_common' : 'other', # kernel/sched.c,
    '__wake_up_locked': 'other', # kernel/sched.c,
    '__wake_up_parent': 'other', # kernel/signal.c,
    '__wake_up_sync': 'other', # kernel/sched.c,
    '__writeback_single_inode' : 'other', # fs/fs-writeback.c,
    'acct_process': 'other', # kernel/acct.c, include/linux/acct.h, include/linux/acct.h,
    'ack_edge_ioapic_irq': 'interrupt', #
    'ack_edge_ioapic_vector': 'interrupt', #
    'activate_page' : 'buffer', # mm/swap.c,
    'activate_task' : 'other', # kernel/sched.c,
    'add_disk_randomness' : 'other', # drivers/char/random.c, include/linux/genhd.h,
    'add_interrupt_randomness': 'driver', # drivers/char/random.c, include/linux/random.h,
    'add_timer_randomness' : 'driver', # drivers/char/random.c,
    'add_to_page_cache' : 'buffer', # mm/filemap.c, include/linux/pagemap.h,
    'add_to_page_cache_lru' : 'buffer', # mm/filemap.c, include/linux/pagemap.h,
    'add_wait_queue' : 'other', # kernel/fork.c,
    'add_wait_queue_exclusive' : 'other', # kernel/fork.c,
    'aligned' : 'other', #
    'alloc_buffer_head' : 'other', # fs/buffer.c, include/linux/buffer_head.h,
    'alloc_dcookie': 'other', # fs/dcookies.c,
    'alloc_fd_array': 'other', # fs/file.c, include/linux/file.h,
    'alloc_inode' : 'other', # fs/inode.c,
    'alloc_pidmap': 'other', # kernel/pid.c, include/linux/pid.h,
    'alloc_skb' : 'buffer', # net/core/skbuff.c, include/linux/skbuff.h,
    'alloc_slabmgmt' : 'buffer', # mm/slab.c,
    'alpha_switch_to' : 'other', # include/asm-alpha/system.h,
    'anon_vma_link': 'buffer', # mm/rmap.c, include/linux/rmap.h, include/linux/rmap.h,
    'anon_vma_prepare': 'buffer', # mm/rmap.c, include/linux/rmap.h, include/linux/rmap.h,
    'anon_vma_unlink': 'buffer', # mm/rmap.c, include/linux/rmap.h,
    'apache': 'other', #
    'apic_timer_interrupt': 'interrupt', # include/asm-i386/hw_irq.h,
    'arch_get_unmapped_area': 'buffer',
    'arch_get_unmapped_area_1': 'buffer',
    'arch_get_unmapped_area_topdown': 'other', #
    'arch_pick_mmap_layout': 'other', #
    'arch_unmap_area_topdown': 'other', #
    'arp_hash': 'stack', # net/ipv4/arp.c, net/ipv4/arp.c,
    'arp_process': 'stack', # net/ipv4/arp.c,
    'arp_rcv': 'stack', # net/ipv4/arp.c,
    'artsd': 'other', #
    'as_add_arq_hash' : 'other', # drivers/block/as-iosched.c,
    'as_add_arq_rb' : 'other', # drivers/block/as-iosched.c,
    'as_add_request' : 'other', # drivers/block/as-iosched.c,
    'as_antic_stop' : 'other', # drivers/block/as-iosched.c,
    'as_choose_req' : 'other', # drivers/block/as-iosched.c,
    'as_completed_request' : 'other', # drivers/block/as-iosched.c,
    'as_dispatch_request' : 'other', # drivers/block/as-iosched.c,
    'as_fifo_expired' : 'other', # drivers/block/as-iosched.c,
    'as_find_arq_hash' : 'other', # drivers/block/as-iosched.c,
    'as_find_arq_rb' : 'other', # drivers/block/as-iosched.c,
    'as_find_first_arq' : 'other', # drivers/block/as-iosched.c,
    'as_find_next_arq' : 'other', # drivers/block/as-iosched.c,
    'as_former_request' : 'other', # drivers/block/as-iosched.c,
    'as_get_io_context' : 'other', # drivers/block/as-iosched.c,
    'as_insert_request' : 'other', # drivers/block/as-iosched.c,
    'as_latter_request' : 'other', # drivers/block/as-iosched.c,
    'as_merge' : 'other', # drivers/block/as-iosched.c,
    'as_merged_request' : 'other', # drivers/block/as-iosched.c,
    'as_merged_requests' : 'other', # drivers/block/as-iosched.c,
    'as_move_to_dispatch' : 'other', # drivers/block/as-iosched.c,
    'as_next_request' : 'other', # drivers/block/as-iosched.c,
    'as_put_request' : 'other', # drivers/block/as-iosched.c,
    'as_queue_empty' : 'other', # drivers/block/as-iosched.c,
    'as_remove_dispatched_request' : 'other', # drivers/block/as-iosched.c,
    'as_remove_merge_hints' : 'other', # drivers/block/as-iosched.c,
    'as_remove_queued_request' : 'other', # drivers/block/as-iosched.c,
    'as_remove_request' : 'other', # drivers/block/as-iosched.c,
    'as_set_request' : 'other', # drivers/block/as-iosched.c,
    'as_update_arq' : 'other', # drivers/block/as-iosched.c,
    'as_update_iohist' : 'other', # drivers/block/as-iosched.c,
    'atomic_dec_and_lock' : 'other', # lib/dec_and_lock.c, include/linux/spinlock.h, include/linux/spinlock.h,
    'atomic_dec_and_lock_1' : 'other', # arch/alpha/lib/dec_and_lock.c,
    'attach_pid': 'other', # kernel/pid.c,
    'attempt_merge' : 'other', # drivers/block/ll_rw_blk.c,
    'auth_domain_drop' : 'other', # net/sunrpc/svcauth.c,
    'auth_domain_put' : 'other', # net/sunrpc/svcauth.c, include/linux/sunrpc/svcauth.h,
    'autoremove_wake_function' : 'other', # kernel/fork.c, include/linux/wait.h,
    'bad_range' : 'buffer', # mm/page_alloc.c,
    'balance_dirty_pages' : 'buffer', # mm/page-writeback.c,
    'balance_dirty_pages_ratelimited' : 'buffer', # mm/page-writeback.c, include/linux/writeback.h,
    'basename': 'other', #
    'bash': 'other', #
    'batch_entropy_store' : 'interrupt', # drivers/char/random.c, include/linux/random.h,
    'bh_lru_install' : 'other', # fs/buffer.c,
    'bh_waitq_head' : 'other', # fs/buffer.c, include/linux/buffer_head.h,
    'bh_wake_function' : 'other', # fs/buffer.c,
    'bio_add_page' : 'other', # fs/bio.c, include/linux/bio.h,
    'bio_alloc' : 'other', # fs/bio.c, include/linux/bio.h,
    'bio_destructor' : 'other', # fs/bio.c,
    'bio_endio' : 'other', # fs/bio.c, include/linux/bio.h,
    'bio_get_nr_vecs' : 'other', # fs/bio.c, include/linux/bio.h,
    'bio_hw_segments' : 'other', # fs/bio.c, include/linux/bio.h,
    'bio_phys_segments' : 'other', # fs/bio.c, include/linux/bio.h,
    'bio_put' : 'other', # fs/bio.c, include/linux/bio.h,
    'blk_backing_dev_unplug' : 'other', # drivers/block/ll_rw_blk.c,
    'blk_cleanup_queue': 'driver', # drivers/block/ll_rw_blk.c, include/linux/blkdev.h,
    'blk_get_queue': 'driver', # drivers/block/ll_rw_blk.c, include/linux/blkdev.h,
    'blk_hw_contig_segment' : 'other', # drivers/block/ll_rw_blk.c, include/linux/blkdev.h,
    'blk_phys_contig_segment' : 'other', # drivers/block/ll_rw_blk.c, include/linux/blkdev.h,
    'blk_plug_device' : 'other', # drivers/block/ll_rw_blk.c, include/linux/blkdev.h,
    'blk_queue_bounce' : 'buffer', # mm/highmem.c, include/linux/blkdev.h,
    'blk_recount_segments' : 'other', # drivers/block/ll_rw_blk.c, include/linux/blkdev.h,
    'blk_remove_plug' : 'other', # drivers/block/ll_rw_blk.c, include/linux/blkdev.h,
    'blk_rq_map_sg' : 'other', # drivers/block/ll_rw_blk.c, include/linux/blkdev.h,
    'blk_run_queue' : 'other', # drivers/block/ll_rw_blk.c, include/linux/blkdev.h,
    'blkdev_ioctl': 'driver', # drivers/block/ioctl.c, include/linux/fs.h,
    'block_ioctl': 'other', # fs/block_dev.c,
    'block_prepare_write' : 'other', # fs/buffer.c, include/linux/buffer_head.h,
    'block_read_full_page': 'other', # fs/buffer.c, include/linux/buffer_head.h,
    'block_write_full_page': 'other', # fs/buffer.c, include/linux/buffer_head.h,
    'bmap': 'other', # fs/jfs/jfs_dmap.h, fs/inode.c, include/linux/fs.h,
    'buffer_insert_list' : 'other', # fs/buffer.c, include/linux/buffer_head.h,
    'buffered_rmqueue' : 'buffer', # mm/page_alloc.c,
    'cache_alloc_refill' : 'buffer', # mm/slab.c,
    'cache_check' : 'other', # net/sunrpc/cache.c, include/linux/sunrpc/cache.h,
    'cache_flusharray' : 'buffer', # mm/slab.c,
    'cache_grow' : 'buffer', # mm/slab.c,
    'cache_init_objs' : 'buffer', # mm/slab.c,
    'cache_reap': 'buffer', # mm/slab.c,
    'cached_lookup': 'other', # fs/namei.c,
    'call_rcu' : 'other', # kernel/rcupdate.c,
    'can_share_swap_page': 'buffer', # mm/swapfile.c, include/linux/swap.h, include/linux/swap.h,
    'can_vma_merge_after': 'buffer', # mm/mmap.c,
    'can_vma_merge_before': 'buffer', # mm/mmap.c,
    'capable': 'other',
    'cascade' : 'interrupt', # kernel/timer.c,
    'cat': 'other', #
    'cdev_get': 'other', # fs/char_dev.c, include/linux/cdev.h,
    'cdrom': 'other', #
    'check_kill_permission' : 'other', # kernel/signal.c,
    'chrdev_open': 'other', # fs/char_dev.c, include/linux/fs.h,
    'cleanup_rbuf' : 'stack', # net/ipv4/tcp.c,
    'clear_inode' : 'other', # fs/inode.c, include/linux/fs.h,
    'clear_page' : 'buffer', # include/asm-alpha/page.h,
    'clear_page_dirty_for_io' : 'buffer', # mm/page-writeback.c, include/linux/mm.h,
    'clear_page_tables': 'buffer', # mm/memory.c, include/linux/mm.h,
    'clear_queue_congested' : 'other', # drivers/block/ll_rw_blk.c,
    'clear_user': 'other', # include/asm-alpha/uaccess.h, include/asm-i386/uaccess.h,
    'clock_panelapplet.so': 'other', #
    'close_private_file' : 'other', # fs/file_table.c, include/linux/fs.h,
    'copy_skb_header' : 'copy',
    'common_interrupt': 'interrupt', #
    'complete': 'other', # kernel/sched.c,
    'compute_creds': 'other', # fs/exec.c, include/linux/binfmts.h,
    'con_chars_in_buffer': 'driver', # drivers/char/vt.c,
    'con_write_room': 'driver', # drivers/char/vt.c,
    'convert_fxsr_from_user': 'interrupt', #
    'convert_fxsr_to_user': 'interrupt', #
    'copy_files': 'other', # kernel/fork.c,
    'copy_from_user': 'copy', # include/asm-alpha/uaccess.h, include/asm-i386/uaccess.h,
    'copy_mm': 'other', # kernel/fork.c,
    'copy_namespace': 'other', # fs/namespace.c, include/linux/namespace.h,
    'copy_page': 'copy',
    'copy_page_range': 'buffer', # mm/memory.c, include/linux/mm.h,
    'copy_process': 'other', # kernel/fork.c, include/linux/sched.h,
    'copy_semundo': 'other', # ipc/sem.c, include/linux/sem.h,
    'copy_strings': 'other', # fs/exec.c, include/linux/binfmts.h,
    'copy_strings_kernel': 'other', # fs/exec.c, include/linux/binfmts.h,
    'copy_thread': 'syscall', # arch/alpha/kernel/process.c, include/linux/sched.h,
    'copy_to_user': 'copy', # include/asm-alpha/uaccess.h, include/asm-i386/uaccess.h,
    'copy_vma': 'buffer', # mm/mmap.c, include/linux/mm.h,
    'count': 'driver', # fs/exec.c, init/initramfs.c, drivers/char/serial_tx3912.c, drivers/char/rocket.c, drivers/isdn/hardware/eicon/diva_didd.c, drivers/isdn/hardware/eicon/divasmain.c, drivers/isdn/hardware/eicon/divasmain.c, drivers/isdn/hardware/eicon/capimain.c, drivers/isdn/hardware/eicon/divasi.c, drivers/isdn/hardware/eicon/divasi.c, drivers/isdn/hardware/eicon/divasi.c, drivers/isdn/hardware/eicon/divasi.c, drivers/isdn/hardware/eicon/divasi.c, drivers/isdn/hardware/eicon/divamnt.c, drivers/isdn/hardware/eicon/divamnt.c, drivers/isdn/hardware/eicon/divamnt.c, drivers/isdn/hardware/eicon/divamnt.c, drivers/isdn/hardware/eicon/divamnt.c, drivers/media/video/w9966.c, drivers/media/video/w9966.c,
    'count_open_files': 'other', # kernel/fork.c,
    'cp_new_stat' : 'other', # fs/stat.c,
    'cp_new_stat64': 'other', # fs/stat.c,
    'cpu_idle' : 'idle', # arch/alpha/kernel/process.c, init/main.c,
    'cpu_quiet' : 'other', # kernel/rcupdate.c,
    'create_buffers' : 'other', # fs/buffer.c,
    'create_elf_tables': 'other', # fs/binfmt_elf.c,
    'create_empty_buffers' : 'other', # fs/buffer.c, include/linux/buffer_head.h,
    'cron': 'other', #
    'csum_partial' : 'stack', # arch/alpha/lib/checksum.c, include/asm-alpha/checksum.h,
    'csum_partial_copy_nocheck' : 'copy', # arch/alpha/lib/csum_partial_copy.c, include/asm-alpha/checksum.h,
    'csum_partial_copy_to_xdr' : 'copy', # net/sunrpc/xprt.c, net/sunrpc/svcsock.c,
    'csum_tcpudp_magic' : 'stack', # arch/alpha/lib/checksum.c, include/asm-alpha/checksum.h,
    'csum_tcpudp_nofold' : 'stack', # arch/alpha/lib/checksum.c, include/asm-alpha/checksum.h,
    'current_kernel_time' : 'other', # kernel/time.c, include/linux/time.h,
    'cut': 'other', #
    'd_alloc' : 'other', # fs/dcache.c, include/linux/dcache.h,
    'd_alloc_anon' : 'other', # fs/dcache.c, include/linux/dcache.h,
    'd_callback' : 'other', # fs/dcache.c,
    'd_find_alias' : 'other', # fs/dcache.c, include/linux/dcache.h,
    'd_free' : 'other', # fs/dcache.c,
    'd_instantiate' : 'other', # fs/dcache.c, include/linux/dcache.h,
    'd_lookup': 'other', # fs/dcache.c, include/linux/dcache.h,
    'd_path': 'other', # fs/dcache.c, include/linux/dcache.h,
    'd_rehash' : 'other', # fs/dcache.c, include/linux/dcache.h,
    'deactivate_task' : 'other', # kernel/sched.c,
    'default_idle' : 'idle', # arch/alpha/kernel/process.c, include/linux/platform.h,
    'default_llseek': 'other', # fs/read_write.c, include/linux/fs.h,
    'default_wake_function' : 'other', # kernel/sched.c, include/linux/wait.h,
    'del_singleshot_timer_sync' : 'other', # kernel/timer.c, include/linux/timer.h, include/linux/timer.h,
    'del_timer' : 'other', # kernel/timer.c, include/linux/timer.h,
    'delay_pmtmr': 'interrupt', #
    'delayed_work_timer_fn': 'other', # kernel/workqueue.c,
    'dentry_open': 'other', # fs/open.c, include/linux/fs.h,
    'deny_write_access': 'other', # fs/namei.c, include/linux/fs.h,
    'dequeue_signal' : 'other', # kernel/signal.c, include/linux/sched.h,
    'dequeue_task' : 'other', # kernel/sched.c,
    'destroy_context': 'interrupt', # include/asm-alpha/mmu_context.h, include/asm-i386/mmu_context.h,
    'destroy_inode' : 'other', # fs/inode.c, include/linux/fs.h,
    'detach_pid': 'other', # kernel/pid.c,
    'detach_vmas_to_be_unmapped': 'buffer', # mm/mmap.c,
    'dev_queue_xmit' : 'stack', # net/core/dev.c, include/linux/netdevice.h,
    'dev_shutdown' : 'stack', # net/sched/sch_generic.c,
    'dev_watchdog': 'stack', # net/sched/sch_generic.c,
    'device_not_available': 'interrupt', #
    'disable_irq_nosync': 'interrupt', # arch/alpha/kernel/irq.c, include/asm-alpha/irq.h, include/asm-i386/irq.h,
    'disk_round_stats' : 'other', # drivers/block/ll_rw_blk.c, include/linux/genhd.h,
    'dnotify_flush' : 'other', # fs/dnotify.c, include/linux/dnotify.h,
    'dnotify_parent' : 'other', # fs/dnotify.c, include/linux/dnotify.h,
    'do_IRQ': 'driver', # drivers/s390/cio/cio.c,
    'do_anonymous_page' : 'buffer', # mm/memory.c,
    'do_bindings' : 'stack', # net/ipv4/netfilter/ip_nat_core.c, include/linux/netfilter_ipv4/ip_nat_core.h,
    'do_brk': 'buffer', # mm/mmap.c, mm/nommu.c, include/linux/mm.h,
    'do_csum_partial_copy_from_user' : 'copy', # arch/alpha/lib/csum_partial_copy.c,
    'do_entInt' : 'interrupt', # arch/alpha/kernel/irq_alpha.c,
    'do_entUna': 'alignment',
    'do_execve': 'other', # fs/exec.c, include/linux/sched.h,
    'do_exit': 'other', # kernel/exit.c,
    'do_fcntl' : 'user', # fs/fcntl.c, used to be syscall`
    'do_fork': 'other', # kernel/fork.c, include/linux/sched.h,
    'do_futex': 'other', # kernel/futex.c, include/linux/futex.h,
    'do_generic_mapping_read': 'buffer', # mm/filemap.c, include/linux/fs.h,
    'do_gettimeofday' : 'user', # arch/alpha/kernel/time.c, include/linux/time.h,  used to by syscall
    'do_group_exit': 'other', # kernel/exit.c, include/linux/sched.h,
    'do_invalidatepage': 'buffer', # mm/truncate.c,
    'do_lookup' : 'user', # fs/namei.c,  used to by syscall
    'do_mmap_pgoff': 'buffer', # mm/mmap.c, mm/nommu.c, include/linux/mm.h,
    'do_mpage_readpage': 'other', # fs/mpage.c,
    'do_mremap': 'buffer', # mm/mremap.c,
    'do_munmap': 'buffer', # mm/mmap.c, mm/nommu.c, include/linux/mm.h,
    'do_no_page' : 'user', # mm/memory.c,  used to by syscall
    'do_nosym': 'other', #
    'do_notify_parent': 'other', # kernel/signal.c, include/linux/sched.h,
    'do_notify_resume': 'interrupt', # arch/alpha/kernel/signal.c,
    'do_osf_sigprocmask' : 'user', # arch/alpha/kernel/signal.c,  used to by syscall
    'do_page_cache_readahead': 'buffer', # mm/readahead.c, include/linux/mm.h,
    'do_page_fault' : 'user', # arch/alpha/mm/fault.c,  used to by syscall
    'do_pipe': 'syscall', # fs/pipe.c, arch/alpha/kernel/osf_sys.c, include/linux/fs.h,
    'do_poll' : 'user', # fs/select.c, drivers/macintosh/apm_emu.c,  used to by syscall
    'do_pollfd' : 'user', # fs/select.c,  used to by syscall
    'do_posix_clock_monotonic_gettime': 'other', # kernel/posix-timers.c, kernel/posix-timers.c, include/linux/time.h,
    'do_posix_clock_monotonic_gettime_parts': 'other', # kernel/posix-timers.c, kernel/posix-timers.c,
    'do_posix_gettime': 'other', # kernel/posix-timers.c, kernel/posix-timers.c,
    'do_readv_writev' : 'user', # fs/read_write.c,  used to by syscall
    'do_select': 'other', # fs/select.c, include/linux/poll.h,
    'do_setitimer': 'other', # kernel/itimer.c, include/linux/time.h,
    'do_sigaction': 'other', # kernel/signal.c, include/linux/sched.h,
    'do_signal' : 'user', # arch/alpha/kernel/signal.c, arch/alpha/kernel/signal.c,  used to by syscall
    'do_sigreturn' : 'user', # arch/alpha/kernel/signal.c,  used to by syscall
    'do_sigsuspend' : 'user', # arch/alpha/kernel/signal.c,  used to by syscall
    'do_softirq' : 'interrupt', # kernel/softirq.c, include/linux/interrupt.h,
    'do_switch_stack' : 'other', #
    'do_sync_read' : 'other', # fs/read_write.c, include/linux/fs.h,
    'do_sync_write': 'other', # fs/read_write.c, include/linux/fs.h,
    'do_timer' : 'other', # kernel/timer.c, include/linux/sched.h,
    'do_truncate': 'other', # fs/open.c, include/linux/fs.h,
    'do_tx_done' : 'driver', # drivers/net/ns83820.c,
    'do_wait': 'other', #
    'do_wp_page': 'buffer', # mm/memory.c,
    'do_writepages' : 'buffer', # mm/page-writeback.c, include/linux/writeback.h,
    'done' : 'other', # drivers/usb/gadget/net2280.c, drivers/usb/gadget/goku_udc.c, drivers/usb/gadget/pxa2xx_udc.c, drivers/scsi/aha152x.c, drivers/scsi/aha152x.c, include/linux/wavefront.h,
    'dp264_disable_irq' : 'interrupt', # arch/alpha/kernel/sys_dp264.c,
    'dp264_enable_irq' : 'interrupt', # arch/alpha/kernel/sys_dp264.c,
    'dp264_end_irq' : 'interrupt', # arch/alpha/kernel/sys_dp264.c,
    'dp264_srm_device_interrupt' : 'interrupt', # arch/alpha/kernel/sys_dp264.c,
    'dput' : 'other', # fs/dcache.c, include/linux/dcache.h,
    'drain_array_locked': 'buffer', # mm/slab.c, mm/slab.c,
    'drive_stat_acct' : 'other', # drivers/block/ll_rw_blk.c, include/linux/blkdev.h,
    'drop_buffers': 'other', # fs/buffer.c,
    'drop_key_refs': 'other', # kernel/futex.c,
    'dst_alloc': 'stack', # net/core/dst.c,
    'dst_output' : 'stack', #
    'dummy_bprm_alloc_security': 'other', # security/dummy.c,
    'dummy_bprm_apply_creds': 'other', # security/dummy.c,
    'dummy_bprm_check_security': 'other', # security/dummy.c,
    'dummy_bprm_secureexec': 'other', # security/dummy.c,
    'dummy_bprm_set_security': 'other', # security/dummy.c,
    'dummy_capable': 'other', # security/dummy.c,
    'dummy_d_instantiate': 'other', # security/dummy.c,
    'dummy_file_alloc_security': 'other', # security/dummy.c,
    'dummy_file_free_security': 'other', # security/dummy.c,
    'dummy_file_ioctl': 'other', # security/dummy.c,
    'dummy_file_mmap': 'other', # security/dummy.c,
    'dummy_file_permission': 'other', # security/dummy.c,
    'dummy_inode_alloc_security': 'other', # security/dummy.c,
    'dummy_inode_create': 'other', # security/dummy.c,
    'dummy_inode_free_security': 'other', # security/dummy.c,
    'dummy_inode_getattr': 'other', # security/dummy.c,
    'dummy_inode_mkdir': 'other', # security/dummy.c,
    'dummy_inode_permission': 'other', # security/dummy.c,
    'dummy_inode_post_create': 'other', # security/dummy.c,
    'dummy_inode_post_mkdir': 'other', # security/dummy.c,
    'dummy_task_create': 'other', # security/dummy.c,
    'dummy_task_free_security': 'other', # security/dummy.c,
    'dummy_task_kill': 'other', # security/dummy.c,
    'dummy_task_wait': 'other', # security/dummy.c,
    'dummy_vm_enough_memory': 'other', # security/dummy.c,
    'dup_task_struct': 'other', # kernel/fork.c,
    'e100': 'driver', #
    'e1000': 'driver',
    'effective_prio' : 'other', # kernel/sched.c,
    'ehci_hcd': 'driver', # drivers/usb/host/ehci.h,
    'elf_map': 'other', # fs/binfmt_elf.c, fs/binfmt_elf.c,
    'eligible_child': 'other', # kernel/exit.c,
    'elv_completed_request' : 'other', # drivers/block/elevator.c, include/linux/elevator.h,
    'elv_former_request' : 'other', # drivers/block/elevator.c, include/linux/elevator.h,
    'elv_latter_request' : 'other', # drivers/block/elevator.c, include/linux/elevator.h,
    'elv_merge' : 'other', # drivers/block/elevator.c, include/linux/elevator.h,
    'elv_merge_requests' : 'other', # drivers/block/elevator.c, include/linux/elevator.h,
    'elv_merged_request' : 'other', # drivers/block/elevator.c, include/linux/elevator.h,
    'elv_next_request' : 'other', # drivers/block/elevator.c, include/linux/elevator.h,
    'elv_put_request' : 'other', # drivers/block/elevator.c, include/linux/elevator.h,
    'elv_queue_empty' : 'other', # drivers/block/elevator.c, include/linux/elevator.h,
    'elv_remove_request' : 'other', # drivers/block/elevator.c, include/linux/elevator.h,
    'elv_rq_merge_ok' : 'other', # drivers/block/elevator.c, include/linux/elevator.h,
    'elv_set_request' : 'other', # drivers/block/elevator.c, include/linux/elevator.h,
    'elv_try_last_merge' : 'other', # drivers/block/elevator.c, include/linux/elevator.h,
    'enable_irq': 'driver', # arch/alpha/kernel/irq.c, drivers/net/wan/sdla_ppp.c, drivers/net/wan/sdla_x25.c, drivers/net/wan/wanpipe_multppp.c, drivers/net/wan/sdla_chdlc.c, drivers/net/wan/sdlamain.c, drivers/net/wan/sdla_fr.c, include/asm-alpha/irq.h, include/asm-i386/irq.h,
    'encode_post_op_attr' : 'other', # fs/nfsd/nfs3xdr.c,
    'encode_wcc_data' : 'other', # fs/nfsd/nfs3xdr.c,
    'end' : 'other', # arch/alpha/boot/misc.c, drivers/media/video/w9966.c, drivers/media/video/w9966.c,
    'end_bio_bh_io_sync' : 'other', # fs/buffer.c,
    'end_buffer_async_write': 'other', # fs/buffer.c, include/linux/buffer_head.h,
    'end_buffer_write_sync' : 'other', # fs/buffer.c, include/linux/buffer_head.h,
    'end_edge_ioapic_vector': 'other', # include/asm-i386/io_apic.h,
    'end_level_ioapic_irq': 'interrupt', #
    'end_level_ioapic_vector': 'interrupt', #
    'end_page_writeback' : 'buffer', # mm/filemap.c, include/linux/pagemap.h,
    'end_that_request_chunk' : 'other', # drivers/block/ll_rw_blk.c, include/linux/blkdev.h,
    'end_that_request_first': 'driver', # drivers/block/ll_rw_blk.c, include/linux/blkdev.h,
    'end_that_request_last' : 'other', # drivers/block/ll_rw_blk.c, include/linux/blkdev.h,
    'enqueue_task' : 'other', # kernel/sched.c,
    'entInt' : 'interrupt', # arch/alpha/kernel/proto.h,
    'entMM' : 'interrupt', # arch/alpha/kernel/proto.h,
    'entSys' : 'interrupt', # arch/alpha/kernel/proto.h,
    'entUna' : 'alignment',
    'entUnaUser':'alignment',
    'error_code': 'other', #
    'eth_header' : 'stack', # net/ethernet/eth.c, include/linux/etherdevice.h,
    'eth_type_trans' : 'stack', # net/ethernet/eth.c, include/linux/etherdevice.h,
    'ev5_flush_tlb_current_page': 'buffer',
    'ev5_switch_mm' : 'other', # include/asm-alpha/mmu_context.h,
    'eventpoll_init_file' : 'other', # fs/eventpoll.c, include/linux/eventpoll.h,
    'exec_mmap': 'other', # fs/exec.c,
    'exim4': 'other', #
    'exit_aio': 'other', # fs/aio.c,
    'exit_itimers': 'other', # kernel/posix-timers.c, include/linux/sched.h,
    'exit_mmap': 'buffer', # mm/mmap.c, mm/nommu.c, include/linux/mm.h,
    'exit_notify': 'other', # kernel/exit.c,
    'exit_sem': 'other', # ipc/sem.c, include/linux/sem.h, include/linux/sem.h,
    'exp_find_key' : 'other', # fs/nfsd/export.c, include/linux/nfsd/export.h,
    'exp_readlock' : 'other', # fs/nfsd/export.c, include/linux/nfsd/export.h,
    'exp_readunlock' : 'other', # fs/nfsd/export.c, include/linux/nfsd/export.h,
    'expand_fd_array': 'other', # fs/file.c, include/linux/file.h,
    'expand_files': 'other', # fs/fcntl.c,
    'expand_stack': 'buffer', # mm/mmap.c, include/linux/mm.h,
    'expkey_put' : 'other', # fs/nfsd/export.c, include/linux/nfsd/export.h,
    'export_decode_fh' : 'other', # fs/exportfs/expfs.c,
    'export_iget' : 'other', # fs/exportfs/expfs.c,
    'expr': 'other', #
    'ext2_alloc_block' : 'other', # fs/ext2/inode.c,
    'ext2_alloc_branch' : 'other', # fs/ext2/inode.c,
    'ext2_block_to_path' : 'other', # fs/ext2/inode.c,
    'ext2_discard_prealloc' : 'other', # fs/ext2/inode.c, fs/ext2/ext2.h,
    'ext2_find_near' : 'other', # fs/ext2/inode.c,
    'ext2_free_blocks' : 'other', # fs/ext2/balloc.c, fs/ext2/ext2.h,
    'ext2_get_block' : 'other', # fs/ext2/inode.c,
    'ext2_get_branch' : 'other', # fs/ext2/inode.c,
    'ext2_get_group_desc' : 'other', # fs/ext2/balloc.c, fs/ext2/ext2.h,
    'ext2_get_inode' : 'other', # fs/ext2/inode.c,
    'ext2_new_block' : 'other', # fs/ext2/balloc.c, fs/ext2/ext2.h,
    'ext2_prepare_write' : 'other', # fs/ext2/inode.c,
    'ext2_put_inode' : 'other', # fs/ext2/inode.c, fs/ext2/ext2.h,
    'ext2_release_file' : 'other', # fs/ext2/file.c,
    'ext2_setattr' : 'other', # fs/ext2/inode.c, fs/ext2/ext2.h,
    'ext2_sync_file' : 'other', # fs/ext2/fsync.c, fs/ext2/ext2.h,
    'ext2_sync_inode' : 'other', # fs/ext2/inode.c, fs/ext2/ext2.h,
    'ext2_update_inode' : 'other', # fs/ext2/inode.c, fs/ext2/inode.c,
    'ext2_write_inode' : 'other', # fs/ext2/inode.c, fs/ext2/ext2.h,
    'ext2_writepages' : 'other', # fs/ext2/inode.c,
    'ext3': 'other', #
    'fasync_helper': 'other', # fs/fcntl.c, include/linux/fs.h,
    'fd_install' : 'other', # fs/open.c,
    'fget' : 'other', # fs/file_table.c,
    'fget_light' : 'other', # fs/file_table.c,
    'fh_put' : 'other', # fs/nfsd/nfsfh.c, include/linux/nfsd/nfsfh.h,
    'fh_verify' : 'other', # fs/nfsd/nfsfh.c, include/linux/nfsd/nfsfh.h,
    'fib_lookup': 'stack', # net/ipv4/fib_rules.c,
    'fib_rule_put': 'stack', # net/ipv4/fib_rules.c,
    'fib_semantic_match': 'stack', # net/ipv4/fib_semantics.c,
    'file_ioctl': 'other', # fs/ioctl.c,
    'file_kill' : 'other', # fs/file_table.c, include/linux/fs.h,
    'file_move': 'other', # fs/file_table.c, include/linux/fs.h,
    'file_ra_state_init': 'buffer', # mm/readahead.c, include/linux/fs.h,
    'file_read_actor': 'buffer', # mm/filemap.c, include/linux/fs.h,
    'filemap_fdatawait' : 'buffer', # mm/filemap.c, include/linux/fs.h,
    'filemap_fdatawrite' : 'buffer', # mm/filemap.c, include/linux/fs.h,
    'filemap_nopage': 'buffer', # mm/filemap.c, include/linux/mm.h,
    'filesystems_read_proc': 'other', # fs/proc/proc_misc.c,
    'filp_close' : 'other', # fs/open.c, include/linux/fs.h,
    'filp_open' : 'other', # fs/open.c, include/linux/fs.h,
    'find_best_ips_proto_fast' : 'stack', # net/ipv4/netfilter/ip_nat_core.c,
    'find_busiest_group' : 'other', # kernel/sched.c,
    'find_dcookie': 'other', # fs/dcookies.c,
    'find_exported_dentry' : 'other', # fs/exportfs/expfs.c, fs/nfsd/export.c,
    'find_extend_vma': 'buffer', # mm/mmap.c, mm/nommu.c, include/linux/mm.h,
    'find_get_page' : 'buffer', # mm/filemap.c, include/linux/pagemap.h,
    'find_get_pages': 'buffer', # mm/filemap.c, include/linux/pagemap.h,
    'find_get_pages_tag' : 'buffer', # mm/filemap.c, include/linux/pagemap.h,
    'find_inode_fast' : 'other', # fs/inode.c,
    'find_inode_number' : 'other', # fs/dcache.c, include/linux/fs.h,
    'find_lock_page' : 'buffer', # mm/filemap.c, include/linux/pagemap.h,
    'find_mergeable_anon_vma': 'buffer', # mm/mmap.c, include/linux/mm.h,
    'find_nat_proto' : 'stack', # net/ipv4/netfilter/ip_nat_core.c, include/linux/netfilter_ipv4/ip_nat_protocol.h,
    'find_next_zero_bit': 'other', # include/asm-alpha/bitops.h, include/asm-i386/bitops.h,
    'find_or_create_page' : 'buffer', # mm/filemap.c, include/linux/pagemap.h,
    'find_pid' : 'user', # kernel/pid.c, used to be syscall
    'find_snap_client': 'stack', # net/802/psnap.c,
    'find_task_by_pid' : 'user', # kernel/pid.c, include/linux/sched.h, used to be syscall
    'find_task_by_pid_type': 'other', #
    'find_vma' : 'buffer', # mm/mmap.c, mm/nommu.c, include/linux/mm.h, used to be syscall
    'find_vma_prepare': 'buffer', # mm/mmap.c,
    'find_vma_prev': 'buffer', # mm/mmap.c, include/linux/mm.h,
    'finish_task_switch' : 'other', # kernel/sched.c, used to be syscall
    'finish_wait' : 'other', # kernel/fork.c, used to be syscall
    'flush_old_exec': 'other', # fs/exec.c, include/linux/binfmts.h,
    'flush_signal_handlers': 'other', # kernel/signal.c, include/linux/sched.h,
    'flush_sigqueue': 'other', # kernel/signal.c,
    'flush_thread': 'syscall', # arch/alpha/kernel/process.c, include/linux/sched.h,
    'fn_hash_lookup': 'stack', # net/ipv4/fib_hash.c,
    'follow_mount' : 'user', # fs/namei.c, used to be syscall
    'found' : 'other', # sound/oss/forte.c, scripts/kconfig/gconf.c, drivers/net/fec.c, drivers/scsi/ibmmca.c, drivers/scsi/fd_mcs.c,
    'fput' : 'user', # fs/file_table.c, used to be syscall
    'free_block' : 'buffer', # mm/slab.c, drivers/char/drm/radeon_mem.c, mm/slab.c,
    'free_buffer_head': 'other', # fs/buffer.c, include/linux/buffer_head.h,
    'free_fd_array': 'other', # fs/file.c, include/linux/file.h,
    'free_hot_cold_page' : 'buffer', # mm/page_alloc.c,
    'free_hot_page' : 'buffer', # mm/page_alloc.c,
    'free_page_and_swap_cache': 'buffer', # mm/swap_state.c, include/linux/swap.h, include/linux/swap.h,
    'free_pages' : 'buffer', # mm/page_alloc.c, drivers/char/drm/drm_memory_debug.h, drivers/md/raid6.h, drivers/char/drm/drmP.h,
    'free_pages_bulk': 'buffer', # mm/page_alloc.c,
    'free_pgtables': 'buffer', # mm/mmap.c,
    'free_pidmap': 'other', # kernel/pid.c,
    'free_task': 'other', # kernel/fork.c,
    'free_uid' : 'other', # kernel/user.c, include/linux/sched.h,
    'freed_request' : 'other', # drivers/block/ll_rw_blk.c,
    'fs_may_remount_ro' : 'other', # fs/file_table.c, include/linux/fs.h,
    'fsync_buffers_list' : 'other', # fs/buffer.c, include/linux/buffer_head.h,
    'futex_wait': 'other', # kernel/futex.c,
    'futex_wake': 'other', # kernel/futex.c,
    'gconfd-2': 'other', #
    'generic_commit_write' : 'user', # fs/buffer.c, include/linux/buffer_head.h, used to be syscall
    'generic_delete_inode': 'other', # fs/inode.c, include/linux/fs.h,
    'generic_drop_inode' : 'user', # fs/inode.c, used to be syscall
    'generic_file_aio_read': 'buffer', # mm/filemap.c, include/linux/fs.h,
    'generic_file_aio_write': 'buffer', # mm/filemap.c, include/linux/fs.h,
    'generic_file_aio_write_nolock' : 'user', # mm/filemap.c, include/linux/fs.h, used to be syscall
    'generic_file_buffered_write': 'other', #
    'generic_file_llseek': 'other', # fs/read_write.c, include/linux/fs.h,
    'generic_file_mmap': 'buffer', # mm/filemap.c, include/linux/fs.h,
    'generic_file_open' : 'user', # fs/open.c, include/linux/fs.h, used to be syscall
    'generic_file_write' : 'user', # mm/filemap.c, include/linux/fs.h, used to be syscall
    'generic_file_write_nolock' : 'user', # mm/filemap.c, include/linux/fs.h, used to be syscall
    'generic_file_writev' : 'user', # mm/filemap.c, include/linux/fs.h, used to be syscall
    'generic_fillattr' : 'user', # fs/stat.c, include/linux/fs.h, used to be syscall
    'generic_forget_inode' : 'user', # fs/inode.c, used to be syscall
    'generic_make_request' : 'user', # drivers/block/ll_rw_blk.c, include/linux/blkdev.h, used to be syscall
    'generic_unplug_device' : 'driver', # drivers/block/ll_rw_blk.c, include/linux/blkdev.h,
    'get_conntrack_index' : 'stack', # net/ipv4/netfilter/ip_conntrack_proto_tcp.c,
    'get_device' : 'driver', # drivers/base/core.c, include/linux/device.h,
    'get_dirty_limits' : 'buffer', # mm/page-writeback.c,
    'get_empty_filp' : 'other', # fs/file_table.c, include/linux/fs.h,
    'get_free_idx': 'interrupt', #
    'get_futex_key': 'other', # kernel/futex.c,
    'get_io_context' : 'other', # drivers/block/ll_rw_blk.c, include/linux/blkdev.h,
    'get_jiffies_64': 'other', # kernel/time.c, include/linux/jiffies.h, include/linux/jiffies.h,
    'get_new_inode_fast': 'other', # fs/inode.c,
    'get_object' : 'other', # fs/exportfs/expfs.c,
    'get_offset_pmtmr': 'interrupt', #
    'get_one_pte_map_nested': 'buffer', # mm/mremap.c,
    'get_page_state': 'buffer', # mm/page_alloc.c, include/linux/page-flags.h,
    'get_pipe_inode': 'other', # fs/pipe.c,
    'get_request' : 'other', # drivers/block/ll_rw_blk.c,
    'get_sample_stats' : 'stack', # net/core/dev.c,
    'get_signal_to_deliver' : 'other', # kernel/signal.c, include/linux/signal.h,
    'get_task_mm': 'other', # include/linux/sched.h,
    'get_tuple' : 'driver', # net/ipv4/netfilter/ip_conntrack_core.c, drivers/isdn/hisax/elsa_cs.c, drivers/isdn/hisax/teles_cs.c, drivers/isdn/hisax/avma1_cs.c, drivers/isdn/hardware/avm/avm_cs.c, drivers/bluetooth/bt3c_cs.c, drivers/bluetooth/btuart_cs.c, drivers/bluetooth/dtl1_cs.c, include/linux/netfilter_ipv4/ip_conntrack_core.h,
    'get_unique_tuple' : 'stack', # net/ipv4/netfilter/ip_nat_core.c,
    'get_unmapped_area': 'buffer', # mm/mmap.c, mm/nommu.c, include/linux/mm.h,
    'get_unused_fd' : 'other', # fs/open.c, include/linux/file.h,  used to be syscall
    'get_vmalloc_info': 'other', # fs/proc/proc_misc.c,
    'get_write_access' : 'other', # fs/namei.c, include/linux/fs.h,  used to be syscall
    'get_writeback_state' : 'other', # mm/page-writeback.c,  used to be syscall
    'get_zone_counts': 'buffer', # mm/page_alloc.c, include/linux/mmzone.h,
    'getname' : 'other', # fs/namei.c, include/linux/fs.h,  used to be syscall
    'getnstimeofday': 'other', #
    'getrusage': 'other', # kernel/sys.c, kernel/exit.c,
    'grab_block' : 'other', # fs/ext2/balloc.c,
    'grep': 'other', #
    'group_release_blocks' : 'other', # fs/ext2/balloc.c,
    'group_reserve_blocks' : 'other', # fs/ext2/balloc.c,
    'group_send_sig_info' : 'other', # kernel/signal.c, include/linux/signal.h,
    'groups_alloc' : 'other', # kernel/sys.c, include/linux/sched.h,  used to be syscall
    'groups_free' : 'other', # kernel/sys.c, include/linux/sched.h,  used to be syscall
    'groups_search' : 'other', # kernel/sys.c,  used to be syscall
    'groups_sort' : 'other', # kernel/sys.c,  used to be syscall
    'groups_to_user': 'other', # kernel/sys.c,
    'grow_dev_page' : 'other', # fs/buffer.c,
    'halfMD4Transform' : 'other', # fs/ext3/hash.c, drivers/char/random.c,
    'handle_IRQ_event' : 'interrupt', # arch/alpha/kernel/irq.c, include/asm-alpha/irq.h,
    'handle_irq' : 'interrupt', # arch/alpha/kernel/irq.c, arch/alpha/kernel/irq_impl.h,
    'handle_mm_fault' : 'interrupt', # mm/memory.c, include/linux/mm.h,
    'handle_signal': 'interrupt', # arch/alpha/kernel/signal.c,
    'handle_stop_signal' : 'interrupt', # kernel/signal.c,
    'hash_conntrack' : 'stack', # net/ipv4/netfilter/ip_conntrack_core.c,
    'hash_futex': 'other', # kernel/futex.c,
    'hash_refile' : 'other', # fs/nfsd/nfscache.c,
    'i8042_interrupt': 'interrupt', # drivers/input/serio/i8042.c, drivers/input/serio/i8042.c,
    'i8042_timer_func': 'driver', # drivers/input/serio/i8042.c,
    'i_waitq_head' : 'other', # fs/inode.c,
    'ide_cd': 'other', #
    'ide_core': 'other', #
    'ide_disk': 'other', # include/linux/ide.h,
    'idle_cpu' : 'idle', # kernel/sched.c, include/linux/sched.h,
    'iget_locked' : 'other', # fs/inode.c, include/linux/fs.h,
    'in_group_p' : 'other', # kernel/sys.c, include/linux/sched.h,
    'inet_accept' : 'stack', # net/ipv4/af_inet.c,
    'inet_create': 'stack', # net/ipv4/af_inet.c,
    'inet_getname' : 'stack', # net/ipv4/af_inet.c,
    'inet_release' : 'stack', # net/ipv4/af_inet.c,
    'inet_sendmsg' : 'stack', # net/ipv4/af_inet.c,
    'inet_sendpage' : 'stack', # net/ipv4/af_inet.c,
    'inet_shutdown' : 'stack', # net/ipv4/af_inet.c,
    'inet_sock_destruct' : 'stack', # net/ipv4/af_inet.c,
    'init': 'driver', # net/core/pktgen.c, net/ipv4/netfilter/ip_conntrack_ftp.c, net/ipv4/netfilter/ip_conntrack_irc.c, net/ipv4/netfilter/ip_tables.c, net/ipv4/netfilter/ipt_ECN.c, net/ipv4/netfilter/ipt_LOG.c, net/ipv4/netfilter/ipt_helper.c, net/ipv4/netfilter/ipt_TOS.c, net/ipv4/netfilter/ipt_addrtype.c, net/ipv4/netfilter/ipt_limit.c, net/ipv4/netfilter/ipt_tcpmss.c, net/ipv4/netfilter/ipt_ecn.c, net/ipv4/netfilter/ipt_esp.c, net/ipv4/netfilter/ipt_mac.c, net/ipv4/netfilter/ipt_tos.c, net/ipv4/netfilter/ipt_ttl.c, net/ipv4/netfilter/ip_nat_ftp.c, net/ipv4/netfilter/ip_nat_irc.c, net/ipv4/netfilter/ipt_multiport.c, net/ipv4/netfilter/ipt_dscp.c, net/ipv4/netfilter/arp_tables.c, net/ipv4/netfilter/ip_conntrack_tftp.c, net/ipv4/netfilter/ipt_physdev.c, net/ipv4/netfilter/ipt_state.c, net/ipv4/netfilter/ipt_ah.c, net/ipv4/netfilter/ipt_mark.c, net/ipv4/netfilter/ip_queue.c, net/ipv4/netfilter/ipt_conntrack.c, net/ipv4/netfilter/ip_fw_compat.c, net/ipv4/netfilter/ipt_NETMAP.c, net/ipv4/netfilter/ipt_pkttype.c, net/ipv4/netfilter/ipt_MASQUERADE.c, net/ipv4/netfilter/ip_conntrack_standalone.c, net/ipv4/netfilter/ip_nat_snmp_basic.c, net/ipv4/netfilter/ipt_length.c, net/ipv4/netfilter/arpt_mangle.c, net/ipv4/netfilter/ipt_CLASSIFY.c, net/ipv4/netfilter/ip_nat_standalone.c, net/ipv4/netfilter/ipt_NOTRACK.c, net/ipv4/netfilter/ip_nat_amanda.c, net/ipv4/netfilter/ipt_REDIRECT.c, net/ipv4/netfilter/ipt_TCPMSS.c, net/ipv4/netfilter/ipt_REJECT.c, net/ipv4/netfilter/ip_conntrack_amanda.c, net/ipv4/netfilter/ipt_owner.c, net/ipv4/netfilter/ipt_DSCP.c, net/ipv4/netfilter/ip_nat_tftp.c, net/ipv4/netfilter/arptable_filter.c, net/ipv4/netfilter/ipt_iprange.c, net/ipv4/netfilter/ipt_MARK.c, net/ipv4/netfilter/iptable_filter.c, net/ipv4/netfilter/iptable_mangle.c, net/ipv4/netfilter/ipt_SAME.c, net/ipv4/netfilter/ipt_realm.c, net/ipv4/netfilter/ipt_ULOG.c, net/ipv4/netfilter/iptable_raw.c, net/ipv6/netfilter/ip6t_length.c, net/ipv6/netfilter/ip6t_eui64.c, net/ipv6/netfilter/ip6t_frag.c, net/ipv6/netfilter/ip6t_multiport.c, net/ipv6/netfilter/ip6t_ah.c, net/ipv6/netfilter/ip6t_hl.c, net/ipv6/netfilter/ip6t_rt.c, net/ipv6/netfilter/ip6t_mark.c, net/ipv6/netfilter/ip6_queue.c, net/ipv6/netfilter/ip6table_filter.c, net/ipv6/netfilter/ip6table_mangle.c, net/ipv6/netfilter/ip6t_owner.c, net/ipv6/netfilter/ip6t_LOG.c, net/ipv6/netfilter/ip6t_dst.c, net/ipv6/netfilter/ip6t_esp.c, net/ipv6/netfilter/ip6t_hbh.c, net/ipv6/netfilter/ip6t_mac.c, net/ipv6/netfilter/ip6_tables.c, net/ipv6/netfilter/ip6t_MARK.c, net/ipv6/netfilter/ip6table_raw.c, net/ipv6/netfilter/ip6t_limit.c, net/bridge/netfilter/ebt_among.c, net/bridge/netfilter/ebt_dnat.c, net/bridge/netfilter/ebt_802_3.c, net/bridge/netfilter/ebt_mark.c, net/bridge/netfilter/ebt_redirect.c, net/bridge/netfilter/ebt_pkttype.c, net/bridge/netfilter/ebt_snat.c, net/bridge/netfilter/ebt_vlan.c, net/bridge/netfilter/ebt_arp.c, net/bridge/netfilter/ebt_log.c, net/bridge/netfilter/ebt_stp.c, net/bridge/netfilter/ebtables.c, net/bridge/netfilter/ebt_limit.c, net/bridge/netfilter/ebtable_broute.c, net/bridge/netfilter/ebt_arpreply.c, net/bridge/netfilter/ebt_ip.c, net/bridge/netfilter/ebtable_filter.c, net/bridge/netfilter/ebt_mark_m.c, net/bridge/netfilter/ebtable_nat.c, net/decnet/netfilter/dn_rtmsg.c, init/main.c, scripts/kconfig/qconf.cc, , as member of class ConfigItemdrivers/usb/host/ehci-hcd.c, drivers/usb/gadget/ether.c, drivers/usb/gadget/net2280.c, drivers/usb/gadget/goku_udc.c, drivers/usb/gadget/zero.c, drivers/usb/gadget/dummy_hcd.c, drivers/usb/gadget/inode.c, drivers/media/dvb/frontends/grundig_29504-401.c, crypto/tcrypt.c, crypto/khazad.c, crypto/digest.c, crypto/des.c, crypto/md4.c, crypto/md5.c, crypto/tea.c, crypto/serpent.c, crypto/blowfish.c, crypto/sha1.c, crypto/crypto_null.c, crypto/crc32c.c, crypto/deflate.c, crypto/cast5.c, crypto/cast6.c, crypto/sha256.c, crypto/sha512.c, crypto/twofish.c, kernel/futex.c, init/main.c, net/ipv4/netfilter/ipt_recent.c, drivers/i2c/chips/w83781d.c, drivers/i2c/chips/w83627hf.c, drivers/media/video/saa7114.c,
    'init_bictcp' : 'stack', # net/ipv4/tcp_input.c,
    'init_buffer_head' : 'other', # fs/buffer.c,
    'init_conntrack' : 'stack', # net/ipv4/netfilter/ip_conntrack_core.c,
    'init_fpu': 'interrupt', # include/asm-i386/i387.h,
    'init_new_context': 'interrupt', # include/asm-alpha/mmu_context.h, include/asm-i386/mmu_context.h,
    'init_page_buffers' : 'other', # fs/buffer.c,
    'init_westwood' : 'stack', # net/ipv4/tcp_input.c,
    'inode_add_bytes' : 'other', # fs/stat.c, include/linux/fs.h,
    'inode_change_ok': 'other', # fs/attr.c, include/linux/fs.h,
    'inode_has_buffers' : 'other', # fs/buffer.c, include/linux/buffer_head.h,
    'inode_setattr': 'other', # fs/attr.c, include/linux/fs.h,
    'inode_sub_bytes' : 'other', # fs/stat.c, include/linux/fs.h,
    'inode_times_differ' : 'other', # fs/inode.c,
    'inode_update_time' : 'other', # fs/inode.c, include/linux/fs.h,
    'insert_vm_struct': 'buffer', # mm/mmap.c, include/linux/mm.h,
    'install_arg_page': 'other', # fs/exec.c, include/linux/mm.h,
    'internal_add_timer' : 'other', # kernel/timer.c,
    'invalid_dpte_no_dismiss_10_' : 'interrupt', #
    'invalidate_inode_buffers' : 'other', # fs/buffer.c, include/linux/buffer_head.h,
    'invert_tuple' : 'stack', # net/ipv4/netfilter/ip_conntrack_core.c,
    'invert_tuplepr' : 'stack', # net/ipv4/netfilter/ip_conntrack_core.c, include/linux/netfilter_ipv4/ip_conntrack.h,
    'io_schedule' : 'other', # kernel/sched.c, include/linux/sched.h,
    'ip_append_data' : 'stack', # net/ipv4/ip_output.c,
    'ip_append_page' : 'stack', # net/ipv4/ip_output.c,
    'ip_build_and_send_pkt' : 'stack', # net/ipv4/ip_output.c,
    'ip_cmsg_recv': 'stack', # net/ipv4/ip_sockglue.c,
    'ip_cmsg_send' : 'stack', # net/ipv4/ip_sockglue.c,
    'ip_confirm' : 'stack', # net/ipv4/netfilter/ip_conntrack_standalone.c,
    'ip_conntrack': 'other', # include/linux/netfilter_ipv4/ip_conntrack.h,
    'ip_conntrack_alter_reply' : 'stack', # net/ipv4/netfilter/ip_conntrack_core.c, include/linux/netfilter_ipv4/ip_conntrack.h,
    'ip_conntrack_defrag' : 'stack', # net/ipv4/netfilter/ip_conntrack_standalone.c,
    'ip_conntrack_find_get' : 'stack', # net/ipv4/netfilter/ip_conntrack_core.c, include/linux/netfilter_ipv4/ip_conntrack_core.h,
    'ip_conntrack_get' : 'stack', # net/ipv4/netfilter/ip_conntrack_core.c, include/linux/netfilter_ipv4/ip_conntrack.h,
    'ip_conntrack_in' : 'stack', # net/ipv4/netfilter/ip_conntrack_core.c, include/linux/netfilter_ipv4/ip_conntrack_core.h,
    'ip_conntrack_local' : 'stack', # net/ipv4/netfilter/ip_conntrack_standalone.c,
    'ip_conntrack_tuple_taken' : 'stack', # net/ipv4/netfilter/ip_conntrack_core.c, include/linux/netfilter_ipv4/ip_conntrack.h,
    'ip_conntrack_unexpect_related' : 'stack', # net/ipv4/netfilter/ip_conntrack_core.c, include/linux/netfilter_ipv4/ip_conntrack_helper.h,
    'ip_ct_find_helper' : 'stack', # net/ipv4/netfilter/ip_conntrack_core.c, include/linux/netfilter_ipv4/ip_conntrack_helper.h,
    'ip_ct_find_proto' : 'stack', # net/ipv4/netfilter/ip_conntrack_core.c, include/linux/netfilter_ipv4/ip_conntrack_core.h,
    'ip_ct_gather_frags' : 'stack', # net/ipv4/netfilter/ip_conntrack_core.c, include/linux/netfilter_ipv4/ip_conntrack.h,
    'ip_ct_refresh' : 'stack', # net/ipv4/netfilter/ip_conntrack_core.c, include/linux/netfilter_ipv4/ip_conntrack.h,
    'ip_defrag' : 'stack', # net/ipv4/ip_fragment.c,
    'ip_evictor' : 'stack', # net/ipv4/ip_fragment.c,
    'ip_fast_csum' : 'stack', # arch/alpha/lib/checksum.c, include/asm-alpha/checksum.h,
    'ip_finish_output' : 'stack', # net/ipv4/ip_output.c,
    'ip_finish_output2' : 'stack', # net/ipv4/ip_output.c,
    'ip_frag_create' : 'stack', # net/ipv4/ip_fragment.c,
    'ip_frag_destroy' : 'stack', # net/ipv4/ip_fragment.c,
    'ip_frag_intern' : 'stack', # net/ipv4/ip_fragment.c,
    'ip_frag_queue' : 'stack', # net/ipv4/ip_fragment.c,
    'ip_frag_reasm' : 'stack', # net/ipv4/ip_fragment.c,
    'ip_local_deliver' : 'stack', # net/ipv4/ip_input.c,
    'ip_local_deliver_finish' : 'stack', # net/ipv4/ip_input.c,
    'ip_map_lookup' : 'stack', # net/sunrpc/svcauth_unix.c,
    'ip_map_put' : 'stack', # net/sunrpc/svcauth_unix.c,
    'ip_mc_drop_socket' : 'stack', # net/ipv4/igmp.c, net/ipv4/af_inet.c, include/linux/igmp.h,
    'ip_nat_fn' : 'stack', # net/ipv4/netfilter/ip_nat_standalone.c,
    'ip_nat_out' : 'stack', # net/ipv4/netfilter/ip_nat_standalone.c,
    'ip_nat_rule_find' : 'stack', # net/ipv4/netfilter/ip_nat_rule.c, include/linux/netfilter_ipv4/ip_nat_rule.h,
    'ip_nat_setup_info' : 'stack', # net/ipv4/netfilter/ip_nat_core.c, include/linux/netfilter_ipv4/ip_nat.h,
    'ip_nat_used_tuple' : 'stack', # net/ipv4/netfilter/ip_nat_core.c, include/linux/netfilter_ipv4/ip_nat.h,
    'ip_output' : 'stack', # net/ipv4/ip_output.c,
    'ip_push_pending_frames' : 'stack', # net/ipv4/ip_output.c,
    'ip_queue_xmit' : 'stack', # net/ipv4/ip_output.c,
    'ip_rcv' : 'stack', # net/ipv4/ip_input.c,
    'ip_rcv_finish' : 'stack', # net/ipv4/ip_input.c,
    'ip_refrag' : 'stack', # net/ipv4/netfilter/ip_conntrack_standalone.c,
    'ip_route_input' : 'stack', # net/ipv4/route.c,
    'ip_route_input_slow': 'stack', # net/ipv4/route.c,
    'ip_route_output_flow' : 'stack', # net/ipv4/route.c,
    'ip_send_check' : 'stack', # net/ipv4/ip_output.c,
    'ip_tables': 'other', #
    'ipq_kill' : 'stack', # net/ipv4/ip_fragment.c,
    'ipqhashfn' : 'stack', # net/ipv4/ip_fragment.c,
    'ipt_do_table' : 'stack', # net/ipv4/netfilter/ip_tables.c, include/linux/netfilter_ipv4/ip_tables.h,
    'ipt_find_target_lock' : 'stack', # net/ipv4/netfilter/ip_tables.c, include/linux/netfilter_ipv4/ip_tables.h, include/linux/netfilter.h,
    'ipt_hook' : 'stack', # net/ipv4/netfilter/iptable_filter.c, net/ipv4/netfilter/iptable_raw.c,
    'ipt_local_hook' : 'stack', # net/ipv4/netfilter/iptable_mangle.c,
    'ipt_local_out_hook' : 'stack', # net/ipv4/netfilter/iptable_filter.c,
    'ipt_route_hook' : 'stack', # net/ipv4/netfilter/iptable_mangle.c,
    'iptable_filter': 'other', #
    'iptable_mangle': 'other', #
    'iptable_nat': 'other', #
    'iput' : 'other', # fs/inode.c, include/linux/fs.h,
    'ipv4_sabotage_in' : 'stack', # net/bridge/br_netfilter.c,
    'ipv4_sabotage_out' : 'stack', # net/bridge/br_netfilter.c,
    'irq_entries_start': 'interrupt', #
    'is_bad_inode' : 'other', # fs/bad_inode.c, include/linux/fs.h,
    'it_real_fn': 'other', # kernel/itimer.c, include/linux/timer.h,
    'jbd': 'other', #
    'juk': 'other', #
    'kded_kmilod.so': 'other', #
    'kdeinit': 'other', #
    'kernel_read': 'other', # fs/exec.c, include/linux/fs.h,
    'kfree' : 'buffer', # mm/slab.c, include/linux/slab.h,
    'kfree_skbmem' : 'buffer', # net/core/skbuff.c, include/linux/skbuff.h,
    'kill_fasync': 'other', # fs/fcntl.c, include/linux/fs.h,
    'kill_proc_info' : 'other', # kernel/signal.c, include/linux/sched.h,
    'kill_something_info' : 'other', # kernel/signal.c,
    'kmap': 'buffer', # include/asm-i386/highmem.h,
    'kmap_atomic': 'buffer', # include/linux/highmem.h, include/asm-i386/highmem.h,
    'kmap_high': 'buffer', # mm/highmem.c,
    'kmem_cache_alloc' : 'buffer', # mm/slab.c, include/linux/slab.h,
    'kmem_cache_free' : 'buffer', # mm/slab.c, include/linux/slab.h,
    'kmem_flagcheck' : 'buffer', # mm/slab.c,
    'kmem_freepages' : 'buffer', # mm/slab.c,
    'kmem_getpages' : 'buffer', # mm/slab.c,
    'kobject_get' : 'other', # lib/kobject.c, include/linux/kobject.h,
    'kobject_put' : 'other', # lib/kobject.c, include/linux/kobject.h,
    'kref_get': 'other', # lib/kref.c, include/linux/kref.h,
    'kscd': 'other', #
    'ksoftirqd' : 'interrupt', # kernel/softirq.c,
    'ksysguardd': 'other', #
    'kthread_should_stop' : 'other', # kernel/kthread.c, include/linux/kthread.h,
    'kunmap': 'buffer', # include/linux/highmem.h, include/asm-i386/highmem.h,
    'kunmap_atomic': 'buffer', # include/linux/highmem.h, include/asm-i386/highmem.h,
    'kunmap_high': 'buffer', # mm/highmem.c,
    'kwrapper': 'other', #
    'ld-2.3.2.so': 'other', #
    'lease_get_mtime' : 'other', # fs/locks.c, include/linux/fs.h,
    'libORBit-2.so.0.0.0': 'other', #
    'libX11.so.6.2': 'other', #
    'libXext.so.6.4': 'other', #
    'libXft.so.2.1.1': 'other', #
    'libXrender.so.1.2.2': 'other', #
    'libacl.so.1.1.0': 'other', #
    'libarts.so': 'other', #
    'libartsdsp.so.0.0.0': 'other', #
    'libartsflow.so.1.0.0': 'other', #
    'libartsmidi.so.0.0.0': 'other', #
    'libattr.so.1.1.0': 'other', #
    'libc-2.3.2.so' : 'user',
    'libcdaudio.so': 'other', #
    'libcrypt-2.3.2.so': 'other', #
    'libcrypto.so.0.9.7': 'other', #
    'libdb3.so.3.0.2': 'other', #
    'libdl-2.3.2.so': 'other', #
    'libgcc_s.so.1': 'other', #
    'libgconf-2.so.4.1.0': 'other', #
    'libgcrypt.so.11.1.1': 'other', #
    'libgdk-1.2.so.0.9.1': 'other', #
    'libgdk-x11-2.0.so.0.400.13': 'other', #
    'libgfx_gtk.so': 'other', #
    'libgkgfx.so': 'other', #
    'libgklayout.so': 'other', #
    'libglib-1.2.so.0.0.10': 'other', #
    'libglib-2.0.so.0.400.8': 'other', #
    'libgnutls.so.11.1.16': 'other', #
    'libgobject-2.0.so.0.400.8': 'other', #
    'libgthread-2.0.so.0.400.8': 'other', #
    'libgtk-x11-2.0.so.0.400.13': 'other', #
    'libhtmlpars.so': 'other', #
    'libimglib2.so': 'other', #
    'libkdecore.so.4.2.0': 'other', #
    'libkdefx.so.4.2.0': 'other', #
    'libkdeinit_kded.so': 'other', #
    'libkdeinit_kdesktop.so': 'other', #
    'libkdeinit_kicker.so': 'other', #
    'libkdeinit_klauncher.so': 'other', #
    'libkdeinit_klipper.so': 'other', #
    'libkdeui.so.4.2.0': 'other', #
    'libksgrd.so.1.2.0': 'other', #
    'libm-2.3.2.so': 'other', #
    'libmcop.so.1.0.0': 'other', #
    'libmcop_mt.so.1.0.0': 'other', #
    'libmikmod.so': 'other', #
    'libmpg123.so': 'other', #
    'libncurses.so.5.4': 'other', #
    'libnecko.so': 'other', #
    'libnsl-2.3.2.so': 'other', #
    'libnspr4.so': 'other', #
    'libnss_compat-2.3.2.so': 'other', #
    'libnss_files-2.3.2.so': 'other', #
    'libnss_nis-2.3.2.so': 'other', #
    'libpcre.so.3.10.0': 'other', #
    'libplc4.so': 'other', #
    'libplds4.so': 'other', #
    'libpref.so': 'other', #
    'libpthread-0.10.so': 'user',
    'libpthread-0.60.so': 'other', #
    'libpthread-0.60.so': 'other', #
    'libpthread-0.60.so': 'other', #
    'libpthread-0.60.so': 'other', #
    'libpthread-0.60.so': 'other', #
    'libpthread-0.60.so': 'other', #
    'libpthread-0.60.so': 'other', #
    'libpthread-0.60.so': 'other', #
    'libpthread-0.60.so': 'other', #
    'libpthread-0.60.so': 'other', #
    'libpthread-0.60.so': 'other', #
    'libpthread-0.60.so': 'other', #
    'libpthread-0.60.so': 'other', #
    'libpthread-0.60.so': 'other', #
    'libpthread-0.60.so': 'other', #
    'libpthread-0.60.so': 'other', #
    'libpthread-0.60.so': 'other', #
    'libpthread-0.60.so': 'other', #
    'libpthread-0.60.so': 'other', #
    'libpthread-0.60.so': 'other', #
    'libpthread-0.60.so': 'other', #
    'libpthread-0.60.so': 'other', #
    'libpthread-0.60.so': 'other', #
    'libpthread-0.60.so': 'other', #
    'libqt-mt.so.3.3.3': 'other', #
    'libqtmcop.so.1.0.0': 'other', #
    'librdf.so': 'other', #
    'libresolv-2.3.2.so': 'other', #
    'librt-2.3.2.so': 'other', #
    'libstdc++.so.5.0.7': 'other', #
    'libtasn1.so.2.0.10': 'other', #
    'libuconv.so': 'other', #
    'libwidget_gtk2.so': 'other', #
    'libwrap.so.0.7.6': 'other', #
    'libxmms.so.1.3.1': 'other', #
    'libxpcom.so': 'other', #
    'link_path_walk' : 'other', # fs/namei.c,
    'll_back_merge_fn' : 'other', # drivers/block/ll_rw_blk.c,
    'll_front_merge_fn' : 'other', # drivers/block/ll_rw_blk.c,
    'll_merge_requests_fn' : 'other', # drivers/block/ll_rw_blk.c,
    'll_rw_block' : 'other', # fs/buffer.c, include/linux/buffer_head.h,
    'llc_rcv': 'stack', # net/llc/llc_input.c,
    'llc_sap_find': 'stack', # net/llc/llc_core.c,
    'load_balance' : 'other', # kernel/sched.c,
    'load_balance_newidle' : 'other', # kernel/sched.c,
    'load_elf_binary': 'other', # fs/binfmt_elf.c, fs/binfmt_elf.c,
    'load_elf_interp': 'other', # fs/binfmt_elf.c,
    'load_script': 'other', # fs/binfmt_script.c,
    'local_bh_enable' : 'interrupt', # kernel/softirq.c, include/linux/interrupt.h,
    'lock_sock' : 'stack', # net/core/sock.c,
    'lockfile-create': 'other', #
    'lockfile-remove': 'other', #
    'locks_remove_flock' : 'other', # fs/locks.c, include/linux/fs.h,
    'locks_remove_posix' : 'other', # fs/locks.c, include/linux/fs.h,
    'lookup_create': 'other', # fs/namei.c, include/linux/dcache.h,
    'lookup_hash': 'other', # fs/namei.c, include/linux/namei.h,
    'lookup_mnt' : 'other', # fs/namespace.c, include/linux/dcache.h,
    'loop' : 'interrupt', #
    'loopback_xmit': 'driver',
    'lru_add_drain' : 'buffer', # mm/swap.c, include/linux/swap.h,
    'lru_cache_add' : 'buffer', # mm/swap.c,
    'lru_cache_add_active': 'buffer', # mm/swap.c,
    'lru_put_front' : 'other', # fs/nfsd/nfscache.c,
    'ls': 'driver', # drivers/fc4/fc.c,
    'mail': 'other', #
    'mapping_tagged' : 'buffer', # mm/page-writeback.c, include/linux/fs.h,
    'mark_buffer_dirty' : 'other', # fs/buffer.c,
    'mark_buffer_dirty_inode' : 'other', # fs/buffer.c, include/linux/buffer_head.h,
    'mark_offset_pmtmr': 'interrupt', #
    'mark_page_accessed' : 'buffer', # mm/swap.c,
    'mask_and_ack_level_ioapic_vector': 'interrupt', # include/asm-i386/io_apic.h,
    'math_state_restore': 'interrupt', #
    'mawk': 'other', #
    'max_sane_readahead': 'buffer', # mm/readahead.c, include/linux/mm.h,
    'max_select_fd': 'other', # fs/select.c,
    'may_open': 'other', # fs/namei.c, include/linux/fs.h,
    'memcmp' : 'copy', # lib/string.c,
    'memcpy' : 'copy', # lib/string.c, arch/alpha/lib/memcpy.c, arch/alpha/kernel/alpha_ksyms.c, include/asm-alpha/string.h, include/asm-alpha/string.h,
    'memcpy_fromiovec': 'copy', # net/core/iovec.c, include/linux/socket.h,
    'memcpy_fromiovecend': 'copy', # net/core/iovec.c, include/linux/socket.h,
    'memcpy_toiovec' : 'copy', # net/core/iovec.c, include/linux/socket.h,
    'meminfo_read_proc': 'other', # fs/proc/proc_misc.c,
    'memmove' : 'copy', # lib/string.c, include/asm-alpha/string.h,
    'mempool_alloc' : 'buffer', # mm/mempool.c, include/linux/mempool.h,
    'mempool_alloc_slab' : 'buffer', # mm/mempool.c, include/linux/mempool.h,
    'mempool_free' : 'buffer', # mm/mempool.c, include/linux/mempool.h,
    'mempool_free_slab' : 'buffer', # mm/mempool.c, include/linux/mempool.h,
    'memscan' : 'copy', # lib/string.c,
    'mkdir': 'other', #
    'mm_alloc': 'buffer', # kernel/fork.c, include/linux/sched.h,
    'mm_init': 'driver', # drivers/block/umem.c, kernel/fork.c,
    'mm_release': 'other', # kernel/fork.c, include/linux/sched.h,
    'mmput': 'other', # kernel/fork.c, include/linux/sched.h,
    'mod_timer' : 'other', # kernel/timer.c, include/linux/timer.h,
    'move_addr_to_user' : 'copy', # net/socket.c, include/linux/socket.h,
    'move_one_page': 'buffer', # mm/mremap.c,
    'move_vma': 'buffer', # mm/mremap.c,
    'mpage_alloc' : 'other', # fs/mpage.c,
    'mpage_bio_submit' : 'other', # fs/mpage.c,
    'mpage_end_io_write' : 'other', # fs/mpage.c,
    'mpage_readpage': 'other', # fs/mpage.c, include/linux/mpage.h,
    'mpage_readpages': 'other', # fs/mpage.c, include/linux/mpage.h,
    'mpage_writepage' : 'other', # fs/mpage.c,
    'mpage_writepages' : 'other', # fs/mpage.c, include/linux/mpage.h,
    'mv': 'other', #
    'n_tty_chars_in_buffer': 'driver', # drivers/char/n_tty.c,
    'n_tty_receive_buf': 'driver', # drivers/char/n_tty.c,
    'n_tty_receive_room': 'driver', # drivers/char/n_tty.c,
    'need_resched': 'driver', # include/linux/sched.h, drivers/char/tipar.c,
    'neigh_lookup': 'stack', # net/core/neighbour.c,
    'neigh_periodic_timer': 'stack', # net/core/neighbour.c,
    'neigh_resolve_output' : 'stack', # net/core/neighbour.c,
    'neigh_timer_handler': 'stack', # net/core/neighbour.c, net/core/neighbour.c,
    'neigh_update': 'stack', # net/core/neighbour.c,
    'net_rx_action' : 'driver', # net/core/dev.c,
    'net_tx_action' : 'driver', # net/core/dev.c,
    'netif_receive_skb' : 'driver', # net/core/dev.c, include/linux/netdevice.h,
    'netif_rx' : 'driver', # net/core/dev.c, include/linux/netdevice.h,
    'netperf' : 'user',
    'netserver': 'user',
    'new_inode' : 'other', # fs/inode.c, include/linux/fs.h,
    'next_signal' : 'other', # kernel/signal.c,
    'next_thread': 'other', # kernel/exit.c,
    'nf_hook_slow' : 'stack', # net/core/netfilter.c, include/linux/netfilter.h,
    'nf_iterate' : 'stack', # net/core/netfilter.c,
    'nfs3svc_decode_commitargs' : 'other', # fs/nfsd/nfs3xdr.c, include/linux/nfsd/xdr3.h,
    'nfs3svc_decode_writeargs' : 'other', # fs/nfsd/nfs3xdr.c, include/linux/nfsd/xdr3.h,
    'nfs3svc_encode_commitres' : 'other', # fs/nfsd/nfs3xdr.c, include/linux/nfsd/xdr3.h,
    'nfs3svc_encode_writeres' : 'other', # fs/nfsd/nfs3xdr.c, include/linux/nfsd/xdr3.h,
    'nfs3svc_release_fhandle' : 'other', # fs/nfsd/nfs3xdr.c, include/linux/nfsd/xdr3.h,
    'nfsd' : 'other', # fs/nfsd/nfssvc.c, fs/nfsd/nfssvc.c,
    'nfsd3_proc_commit' : 'other', # fs/nfsd/nfs3proc.c,
    'nfsd3_proc_write' : 'other', # fs/nfsd/nfs3proc.c,
    'nfsd_acceptable' : 'other', # fs/nfsd/nfsfh.c,
    'nfsd_cache_append' : 'other', # fs/nfsd/nfscache.c, fs/nfsd/nfscache.c,
    'nfsd_cache_lookup' : 'other', # fs/nfsd/nfscache.c, include/linux/nfsd/cache.h,
    'nfsd_cache_update' : 'other', # fs/nfsd/nfscache.c, include/linux/nfsd/cache.h,
    'nfsd_close' : 'other', # fs/nfsd/vfs.c, include/linux/nfsd/nfsd.h,
    'nfsd_commit' : 'other', # fs/nfsd/vfs.c, include/linux/nfsd/nfsd.h,
    'nfsd_dispatch' : 'other', # fs/nfsd/nfssvc.c, include/linux/nfsd/nfsd.h,
    'nfsd_open' : 'other', # fs/nfsd/vfs.c, include/linux/nfsd/nfsd.h,
    'nfsd_permission' : 'other', # fs/nfsd/vfs.c, include/linux/nfsd/nfsd.h,
    'nfsd_setuser' : 'other', # fs/nfsd/auth.c, include/linux/nfsd/auth.h,
    'nfsd_sync' : 'other', # fs/nfsd/vfs.c,
    'nfsd_write' : 'other', # fs/nfsd/vfs.c, include/linux/nfsd/nfsd.h,
    'no_pm_change_10_' : 'interrupt', #
    'no_quad' : 'interrupt', #
    'nonseekable_open' : 'other', # fs/open.c, include/linux/fs.h,
    'normal_int' : 'other', #
    'normal_poll': 'driver', # drivers/char/n_tty.c,
    'note_interrupt': 'interrupt', #
    'notifier_call_chain': 'other', # kernel/sys.c, include/linux/notifier.h,
    'notify_change': 'other', # fs/attr.c, include/linux/fs.h,
    'nr_blockdev_pages': 'other', # fs/block_dev.c, include/linux/blkdev.h,
    'nr_free_pages': 'buffer', # mm/page_alloc.c, include/linux/swap.h,
    'nr_running': 'other', # kernel/sched.c, include/linux/sched.h,
    'ns83820': 'driver',
    'ns83820_do_isr' : 'driver',
    'ns83820_hard_start_xmit' : 'driver',
    'ns83820_irq' : 'driver',
    'ns83820_rx_kick' : 'driver',
    'ns83820_tx_watch' : 'driver',
    'ns83821_do_isr' : 'driver', #
    'ns83821_hard_start_xmit' : 'driver', #
    'ns83821_irq' : 'driver', #
    'ns83821_rx_kick' : 'driver', #
    'number' : 'interrupt', # lib/vsprintf.c, arch/alpha/kernel/srm_env.c,
    'nvidia': 'other', #
    'old_mmap': 'interrupt', #
    'open_exec': 'other', # fs/exec.c, include/linux/fs.h,
    'open_namei' : 'user', # fs/namei.c, include/linux/fs.h,  used to by syscall
    'open_private_file' : 'user', # fs/file_table.c, include/linux/fs.h,  used to by syscall
    'oprofile': 'other', #
    'oprofiled': 'other', #
    'osf_brk': 'user',
    'osf_mmap': 'user',
    'osf_sigprocmask' : 'other', #
    'osync_buffers_list' : 'other', # fs/buffer.c,
    'padzero': 'other', # fs/binfmt_elf.c,
    'page_add_anon_rmap' : 'buffer', # mm/rmap.c, include/linux/rmap.h,
    'page_add_file_rmap': 'buffer', # mm/rmap.c, include/linux/rmap.h,
    'page_address': 'buffer', # mm/highmem.c, include/linux/mm.h, include/linux/mm.h, include/linux/mm.h,
    'page_cache_readahead': 'buffer', # mm/readahead.c, include/linux/mm.h,
    'page_fault': 'interrupt', #
    'page_remove_rmap': 'buffer', # mm/rmap.c, include/linux/rmap.h,
    'page_slot': 'buffer', # mm/highmem.c,
    'page_symlink' : 'other', # fs/namei.c, include/linux/fs.h,
    'page_waitqueue' : 'buffer', # mm/filemap.c,
    'pagevec_lookup': 'buffer', # mm/swap.c, include/linux/pagevec.h,
    'pagevec_lookup_tag' : 'buffer', # mm/swap.c, include/linux/pagevec.h,
    'pal_dtb_ldq' : 'interrupt', #
    'pal_itb_ldq' : 'interrupt', #
    'pal_post_interrupt' : 'interrupt', #
    'path_lookup' : 'user', # fs/namei.c,  used to by syscall
    'path_release' : 'user', # fs/namei.c, include/linux/namei.h,  used to by syscall
    'pci_bus_read_config_word': 'driver', # include/linux/pci.h,
    'pci_conf1_read': 'driver', #
    'pci_dac_dma_supported' : 'driver', # arch/alpha/kernel/pci_iommu.c, include/asm-alpha/pci.h,
    'pci_map_page' : 'driver', # arch/alpha/kernel/pci_iommu.c, include/asm-generic/pci-dma-compat.h, include/asm-alpha/pci.h,
    'pci_map_single' : 'driver', # arch/alpha/kernel/pci_iommu.c, arch/alpha/kernel/pci-noop.c, include/asm-generic/pci-dma-compat.h, drivers/net/wan/wanxl.c, drivers/net/wan/wanxl.c, drivers/scsi/aic7xxx/aic79xx_osm.h, drivers/scsi/aic7xxx/aic7xxx_osm.h, include/asm-alpha/pci.h,
    'pci_map_single_1' : 'driver', # arch/alpha/kernel/pci_iommu.c,
    'pci_read': 'driver', #
    'pci_unmap_page' : 'driver', # arch/alpha/kernel/pci_iommu.c, include/asm-generic/pci-dma-compat.h, include/asm-alpha/pci.h,
    'pci_unmap_single' : 'driver', # arch/alpha/kernel/pci_iommu.c, arch/alpha/kernel/pci-noop.c, include/asm-generic/pci-dma-compat.h, drivers/scsi/aic7xxx/aic79xx_osm.h, drivers/scsi/aic7xxx/aic7xxx_osm.h, include/asm-alpha/pci.h,
    'percpu_counter_mod' : 'buffer', # mm/swap.c, include/linux/percpu_counter.h,
    'perl': 'other', #
    'permission' : 'user', # fs/namei.c, include/linux/fs.h, used to be syscall
    'pfifo_fast_dequeue' : 'stack', # net/sched/sch_generic.c,
    'pfifo_fast_enqueue' : 'stack', # net/sched/sch_generic.c,
    'pgd_alloc': 'buffer', # arch/alpha/mm/init.c, include/asm-alpha/pgalloc.h, include/asm-i386/pgalloc.h,
    'pgd_ctor': 'buffer', # include/asm-i386/pgtable.h,
    'pgd_free': 'buffer', # include/asm-alpha/pgalloc.h, include/asm-i386/pgalloc.h,
    'pipe_ioctl': 'other', # fs/pipe.c,
    'pipe_new': 'other', # fs/pipe.c, include/linux/pipe_fs_i.h,
    'pipe_poll': 'other', # fs/pipe.c,
    'pipe_read': 'other', # fs/pipe.c,
    'pipe_read_release': 'other', # fs/pipe.c,
    'pipe_readv': 'other', # fs/pipe.c,
    'pipe_release': 'other', # fs/pipe.c,
    'pipe_wait': 'other', # fs/pipe.c, include/linux/pipe_fs_i.h,
    'pipe_write': 'other', # fs/pipe.c,
    'pipe_write_fasync': 'other', # fs/pipe.c,
    'pipe_write_release': 'other', # fs/pipe.c,
    'pipe_writev': 'other', # fs/pipe.c,
    'pipefs_delete_dentry': 'other', # fs/pipe.c,
    'place_in_hashes' : 'stack', # net/ipv4/netfilter/ip_nat_core.c, include/linux/netfilter_ipv4/ip_nat_core.h,
    'poll_freewait' : 'other', # fs/select.c, include/linux/poll.h,
    'poll_idle': 'idle', #
    'poll_initwait' : 'other', # fs/select.c, include/linux/poll.h,
    'portmap': 'other', #
    'preempt_schedule': 'other', # kernel/sched.c, include/linux/preempt.h,
    'prep_new_page' : 'buffer', # mm/page_alloc.c,
    'prepare_binprm': 'other', # fs/exec.c, include/linux/binfmts.h,
    'prepare_to_copy': 'interrupt', # include/asm-alpha/processor.h, include/asm-i386/processor.h,
    'prepare_to_wait' : 'other', # kernel/fork.c,
    'prio_tree_expand': 'buffer', # mm/prio_tree.c,
    'prio_tree_insert': 'buffer', # mm/prio_tree.c,
    'prio_tree_remove': 'buffer', # mm/prio_tree.c,
    'prio_tree_replace': 'buffer', # mm/prio_tree.c,
    'proc_alloc_inode': 'other', # fs/proc/inode.c,
    'proc_calc_metrics': 'other', # fs/proc/proc_misc.c,
    'proc_delete_inode': 'other', # fs/proc/inode.c,
    'proc_destroy_inode': 'other', # fs/proc/inode.c,
    'proc_file_read': 'other', # fs/proc/generic.c, fs/proc/generic.c,
    'proc_get_inode': 'other', # fs/proc/inode.c, include/linux/proc_fs.h,
    'proc_lookup': 'other', # fs/proc/generic.c, include/linux/proc_fs.h,
    'proc_pid_unhash': 'other', # fs/proc/base.c, include/linux/proc_fs.h,
    'proc_pident_lookup': 'other', # fs/proc/base.c,
    'proc_root_lookup': 'other', # fs/proc/root.c,
    'process_backlog' : 'stack', # net/core/dev.c,
    'process_timeout': 'other', # kernel/timer.c,
    'profile_hit': 'other', #
    'profile_hook': 'other', # kernel/profile.c, include/linux/profile.h, include/linux/profile.h,
    'profile_munmap': 'other', #
    'profile_task_exit': 'other', #
    'profile_tick': 'other', #
    'pskb_expand_head': 'stack', # net/core/skbuff.c, include/linux/skbuff.h,
    'pte_alloc_map': 'buffer', # mm/memory.c,
    'pte_alloc_one': 'buffer', # include/asm-alpha/pgalloc.h, include/asm-i386/pgalloc.h,
    'ptrace_cancel_bpt' : 'user', # arch/alpha/kernel/ptrace.c, arch/alpha/kernel/proto.h, used to be syscall
    'pty_chars_in_buffer': 'driver', # drivers/char/pty.c,
    'pty_open': 'driver', # drivers/char/pty.c,
    'pty_write_room': 'driver', # drivers/char/pty.c,
    'put_device' : 'driver', # drivers/base/core.c, include/linux/device.h,
    'put_files_struct': 'other', # kernel/exit.c,
    'put_filp': 'other', # fs/file_table.c, include/linux/file.h,
    'put_io_context' : 'driver', # drivers/block/ll_rw_blk.c, include/linux/blkdev.h,
    'put_unused_fd' : 'other', # fs/open.c,
    'qdisc_restart' : 'stack', # net/sched/sch_generic.c,
    'queue_delayed_work': 'other', # kernel/workqueue.c,
    'queue_me': 'other', # kernel/futex.c,
    'quiesce' : 'idle', #
    'radix_tree_delete': 'other', # lib/radix-tree.c, include/linux/radix-tree.h,
    'radix_tree_extend': 'other', # lib/radix-tree.c,
    'radix_tree_gang_lookup': 'other', # lib/radix-tree.c, include/linux/radix-tree.h,
    'radix_tree_gang_lookup_tag' : 'other', # lib/radix-tree.c, include/linux/radix-tree.h,
    'radix_tree_insert' : 'other', # lib/radix-tree.c, include/linux/radix-tree.h,
    'radix_tree_lookup' : 'other', # lib/radix-tree.c, include/linux/radix-tree.h,
    'radix_tree_node_alloc' : 'other', # lib/radix-tree.c,
    'radix_tree_preload' : 'other', # lib/radix-tree.c, lib/radix-tree.c, include/linux/radix-tree.h,
    'radix_tree_tag_clear' : 'other', # lib/radix-tree.c, include/linux/radix-tree.h,
    'radix_tree_tag_set' : 'other', # lib/radix-tree.c, include/linux/radix-tree.h,
    'radix_tree_tagged' : 'other', # lib/radix-tree.c, include/linux/radix-tree.h,
    'raise_softirq' : 'interrupt', # kernel/softirq.c,
    'raise_softirq_irqoff' : 'interrupt', # kernel/softirq.c,
    'rb_erase' : 'buffer', # lib/rbtree.c, include/linux/rbtree.h,
    'rb_insert_color' : 'buffer', # lib/rbtree.c, include/linux/rbtree.h,
    'rb_next' : 'buffer', # lib/rbtree.c, fs/jffs2/nodelist.h, include/linux/rbtree.h,
    'rb_prev' : 'buffer', # lib/rbtree.c, fs/jffs2/nodelist.h, include/linux/rbtree.h,
    'rcu_check_callbacks' : 'other', # kernel/rcupdate.c, include/linux/rcupdate.h,
    'rcu_check_quiescent_state' : 'other', # kernel/rcupdate.c,
    'rcu_do_batch' : 'other', # kernel/rcupdate.c,
    'rcu_process_callbacks' : 'other', # kernel/rcupdate.c,
    'rcu_start_batch' : 'other', # kernel/rcupdate.c,
    'read_block_bitmap' : 'other', # fs/udf/balloc.c, fs/ext2/balloc.c, fs/ext3/balloc.c,
    'real_lookup': 'other', # fs/namei.c,
    'rebalance_tick' : 'other', # kernel/sched.c,
    'recalc_bh_state' : 'other', # fs/buffer.c,
    'recalc_sigpending' : 'interrupt', # kernel/signal.c, include/linux/sched.h,
    'recalc_sigpending_tsk' : 'interrupt', # kernel/signal.c,
    'recalc_task_prio' : 'other', # kernel/sched.c,
    'release_blocks' : 'other', # fs/ext2/balloc.c,
    'release_pages' : 'buffer', # mm/swap.c, include/linux/pagemap.h,
    'release_sock' : 'stack', # net/core/sock.c,
    'release_task': 'other', # kernel/exit.c, include/linux/sched.h,
    'release_thread': 'interrupt', # arch/alpha/kernel/process.c, include/asm-um/processor-generic.h, include/asm-alpha/processor.h, include/asm-i386/processor.h,
    'release_x86_irqs': 'interrupt', # include/asm-i386/irq.h,
    'remove_arg_zero': 'other', # fs/exec.c, include/linux/binfmts.h,
    'remove_from_page_cache': 'buffer', # mm/filemap.c, include/linux/pagemap.h,
    'remove_suid' : 'buffer', # mm/filemap.c, include/linux/fs.h,
    'remove_vm_struct': 'buffer', # mm/mmap.c,
    'remove_wait_queue' : 'other', # kernel/fork.c,
    'resched_task' : 'other', # kernel/sched.c,
    'reserve_blocks' : 'other', # fs/ext2/balloc.c,
    'restore_all' : 'other', #
    'restore_fpu': 'interrupt', # include/asm-i386/i387.h,
    'restore_i387': 'interrupt', # include/asm-i386/i387.h,
    'restore_i387_fxsave': 'interrupt', #
    'restore_sigcontext' : 'interrupt', # arch/alpha/kernel/signal.c,
    'resume_kernel': 'interrupt', #
    'resume_userspace': 'other', #
    'ret_from_exception': 'other', #
    'ret_from_intr': 'interrupt', #
    'ret_from_reschedule' : 'other', #
    'ret_from_sys_call' : 'user', # arch/alpha/kernel/signal.c, used to be syscall
    'rm': 'other', #
    'rm_from_queue': 'other', # kernel/signal.c,
    'rmqueue_bulk' : 'buffer', # mm/page_alloc.c,
    'rt_check_expire': 'stack', # net/ipv4/route.c,
    'rt_hash_code' : 'stack', # net/ipv4/route.c,
    'rt_intern_hash': 'stack', # net/ipv4/route.c, net/ipv4/route.c,
    'rt_may_expire': 'stack', # net/ipv4/route.c,
    'rtc_enable_disable' : 'interrupt', # arch/alpha/kernel/irq_alpha.c,
    'rti_to_kern' : 'interrupt', #
    'rti_to_user' : 'user', # used to be syscall
    'run-parts': 'other', #
    'run_local_timers' : 'other', # kernel/timer.c, include/linux/timer.h,
    'run_timer_softirq' : 'other', # kernel/timer.c,
    'rx_action' : 'driver', # drivers/net/ns83820.c,
    'rx_irq' : 'driver', # drivers/net/ns83820.c,
    'rx_refill_atomic' : 'driver', # drivers/net/ns83820.c,
    'save_i387': 'interrupt', # include/asm-i386/i387.h,
    'save_i387_fxsave': 'interrupt', #
    'sched_clock' : 'user', # arch/alpha/kernel/time.c, include/linux/sched.h, used to be syscall
    'sched_exit': 'other', # kernel/sched.c,
    'sched_fork': 'other', # kernel/sched.c,
    'schedule' : 'other', # kernel/sched.c, include/linux/sched.h,
    'schedule_delayed_work': 'other', # kernel/workqueue.c,
    'schedule_tail': 'other', # kernel/sched.c,
    'schedule_timeout' : 'other', # kernel/timer.c, sound/oss/cs4281/cs4281m.c,
    'scheduler_tick' : 'other', # kernel/sched.c, include/linux/sched.h,
    'scsi_add_timer' : 'other', # drivers/scsi/scsi_error.c,
    'scsi_alloc_sgtable' : 'other', # drivers/scsi/scsi_lib.c,
    'scsi_cmd_ioctl': 'driver', # drivers/block/scsi_ioctl.c, include/linux/blkdev.h,
    'scsi_decide_disposition' : 'other', # drivers/scsi/scsi_error.c, drivers/scsi/scsi_priv.h,
    'scsi_delete_timer' : 'other', # drivers/scsi/scsi_error.c,
    'scsi_device_unbusy' : 'other', # drivers/scsi/scsi_lib.c, drivers/scsi/scsi_priv.h,
    'scsi_dispatch_cmd' : 'other', # drivers/scsi/scsi.c, drivers/scsi/scsi_priv.h,
    'scsi_done' : 'other', # drivers/scsi/scsi.c, drivers/scsi/scsi_priv.h,
    'scsi_end_request' : 'other', # drivers/scsi/scsi_lib.c,
    'scsi_finish_command' : 'other', # drivers/scsi/scsi.c,
    'scsi_free_sgtable' : 'other', # drivers/scsi/scsi_lib.c,
    'scsi_get_command' : 'other', # drivers/scsi/scsi.c,
    'scsi_init_cmd_errh' : 'other', # drivers/scsi/scsi_lib.c,
    'scsi_init_io' : 'other', # drivers/scsi/scsi_lib.c,
    'scsi_io_completion' : 'other', # drivers/scsi/scsi_lib.c,
    'scsi_mod': 'driver',
    'scsi_next_command' : 'other', # drivers/scsi/scsi_lib.c, drivers/scsi/scsi_priv.h,
    'scsi_prep_fn' : 'other', # drivers/scsi/scsi_lib.c,
    'scsi_put_command' : 'other', # drivers/scsi/scsi.c,
    'scsi_request_fn' : 'other', # drivers/scsi/scsi_lib.c,
    'scsi_run_queue' : 'other', # drivers/scsi/scsi_lib.c,
    'scsi_softirq' : 'other', # drivers/scsi/scsi.c,
    'sd_init_command' : 'other', # drivers/scsi/sd.c, drivers/scsi/sd.c,
    'sd_rw_intr' : 'other', # drivers/scsi/sd.c, drivers/scsi/sd.c,
    'search_binary_handler': 'other', # fs/exec.c, include/linux/binfmts.h,
    'second_overflow': 'interrupt', # kernel/timer.c,
    'secure_tcp_sequence_number' : 'stack', # drivers/char/random.c, include/linux/random.h,
    'sed': 'other', #
    'select_bits_alloc': 'other', # fs/compat.c, fs/select.c,
    'select_bits_free': 'other', # fs/compat.c, fs/select.c,
    'send_group_sig_info': 'other', # kernel/signal.c, include/linux/sched.h,
    'send_signal' : 'user', # kernel/signal.c, used to be syscall
    'seq_read': 'other', # fs/seq_file.c, include/linux/seq_file.h,
    'set_bh_page' : 'other', # fs/buffer.c, include/linux/buffer_head.h,
    'set_binfmt': 'other', # fs/exec.c, include/linux/binfmts.h,
    'set_brk': 'user', # fs/binfmt_aout.c, fs/binfmt_elf.c,
    'set_current_groups' : 'other', # kernel/sys.c, include/linux/sched.h,
    'set_page_address': 'buffer', # mm/highmem.c, include/linux/mm.h, include/linux/mm.h, include/linux/mm.h,
    'set_page_dirty': 'buffer', # mm/page-writeback.c,
    'set_slab_attr' : 'buffer', # mm/slab.c,
    'set_task_comm': 'other', #
    'setfl' : 'user', # fs/fcntl.c, used to be syscall
    'setup_arg_pages': 'other', # fs/exec.c, include/linux/binfmts.h,
    'setup_frame' : 'interrupt', # arch/alpha/kernel/signal.c,
    'setup_sigcontext' : 'interrupt', # arch/alpha/kernel/signal.c,
    'show_stat': 'other', # fs/proc/proc_misc.c,
    'si_swapinfo': 'buffer', # mm/swapfile.c, include/linux/swap.h, include/linux/swap.h,
    'sig_ignored' : 'other', # kernel/signal.c,
    'signal_wake_up' : 'other', # kernel/signal.c, include/linux/sched.h,
    'sigprocmask' : 'other', # kernel/signal.c, include/linux/signal.h,
    'single_open': 'other', # fs/seq_file.c, include/linux/seq_file.h,
    'sk_alloc' : 'buffer', # net/core/sock.c,
    'sk_free' : 'buffer', # net/core/sock.c,
    'sk_reset_timer' : 'buffer', # net/core/sock.c,
    'sk_stop_timer' : 'buffer', # net/core/sock.c,
    'sk_stream_kill_queues' : 'buffer', # net/core/stream.c,
    'sk_stream_mem_schedule' : 'buffer', # net/core/stream.c,
    'sk_stream_rfree' : 'buffer', # net/core/stream.c,
    'sk_stream_wait_close' : 'buffer', # net/core/stream.c,
    'sk_stream_wait_memory' : 'buffer', # net/core/stream.c,
    'sk_stream_write_space' : 'buffer', # net/core/stream.c,
    'sk_wait_data' : 'buffer', # net/core/sock.c,
    'skb_checksum': 'stack', # net/core/skbuff.c, include/linux/skbuff.h,
    'skb_checksum_help': 'stack', # net/core/dev.c, include/linux/netdevice.h,
    'skb_clone' : 'buffer', # net/core/skbuff.c, include/linux/skbuff.h,
    'skb_copy_and_csum_bits' : 'copy', # net/core/skbuff.c, include/linux/skbuff.h,
    'skb_copy_and_csum_datagram':'copy',
    'skb_copy_bits' : 'copy', # net/core/skbuff.c, include/linux/skbuff.h,
    'skb_copy_datagram_iovec' : 'copy', # net/core/datagram.c, include/linux/skbuff.h,
    'skb_dequeue' : 'buffer', # net/core/skbuff.c, include/linux/skbuff.h,
    'skb_drop_fraglist' : 'buffer', # net/core/skbuff.c,
    'skb_free_datagram' : 'buffer', # net/core/datagram.c, include/linux/skbuff.h,
    'skb_queue_head': 'stack', # net/core/skbuff.c, include/linux/skbuff.h,
    'skb_queue_tail' : 'buffer', # net/core/skbuff.c, include/linux/skbuff.h,
    'skb_read_and_csum_bits' : 'buffer', # net/sunrpc/xprt.c,
    'skb_recv_datagram' : 'buffer', # net/core/datagram.c, include/linux/skbuff.h,
    'skb_release_data' : 'buffer', # net/core/skbuff.c, net/core/dev.c,
    'skip_atoi': 'other', # lib/vsprintf.c,
    'slab_destroy' : 'buffer', # mm/slab.c,
    'smp_apic_timer_interrupt': 'interrupt', #
    'smp_percpu_timer_interrupt' : 'interrupt', # arch/alpha/kernel/smp.c, arch/alpha/kernel/proto.h,
    'snap_rcv': 'stack', # net/802/psnap.c,
    'sock_aio_read' : 'stack', # net/socket.c, net/socket.c,
    'sock_aio_write': 'stack', # net/socket.c, net/socket.c,
    'sock_alloc' : 'user', # net/socket.c, include/linux/net.h, used to be syscall
    'sock_alloc_inode' : 'user', # net/socket.c, used to be syscall
    'sock_alloc_send_pskb' : 'user', # net/core/sock.c, used to be syscall
    'sock_alloc_send_skb' : 'user', # net/core/sock.c, used to be syscall
    'sock_close' : 'user', # net/socket.c, net/socket.c, used to be syscall
    'sock_common_recvmsg' : 'user', # net/core/sock.c, used to be syscall
    'sock_def_readable' : 'user', # net/core/sock.c, used to be syscall
    'sock_def_wakeup' : 'user', # net/core/sock.c, used to be syscall
    'sock_destroy_inode' : 'user', # net/socket.c, used to be syscall
    'sock_disable_timestamp' : 'user', # net/core/sock.c, used to be syscall
    'sock_fasync' : 'user', # net/socket.c, net/socket.c, used to be syscall
    'sock_init_data': 'stack', # net/core/sock.c,
    'sock_ioctl': 'stack', # net/socket.c, net/socket.c,
    'sock_map_fd' : 'user', # net/socket.c, include/linux/net.h, used to be syscall
    'sock_poll' : 'user', # net/socket.c, net/socket.c, used to be syscall
    'sock_readv': 'stack', # net/socket.c, net/socket.c,
    'sock_readv_writev' : 'user', # net/socket.c, include/linux/net.h, used to be syscall
    'sock_recvmsg' : 'user', # net/socket.c, include/linux/net.h, used to be syscall
    'sock_release' : 'user', # net/socket.c, include/linux/net.h, used to be syscall
    'sock_rfree' : 'user', # net/core/sock.c, used to be syscall
    'sock_sendmsg' : 'user', # net/socket.c, include/linux/net.h, used to be syscall
    'sock_wfree' : 'user', # net/core/sock.c, used to be syscall
    'sock_wmalloc' : 'user', # net/core/sock.c, used to be syscall
    'sock_writev' : 'user', # net/socket.c, net/socket.c, used to be syscall
    'sockfd_lookup' : 'user', # net/socket.c, net/sched/sch_atm.c, include/linux/net.h, used to be syscall
    'sockfs_delete_dentry' : 'user', # net/socket.c, used to be syscall
    'sort': 'driver', # drivers/scsi/eata.c, drivers/scsi/u14-34f.c,
    'split_vma': 'buffer', # mm/mmap.c, include/linux/mm.h,
    'sprintf' : 'other', # lib/vsprintf.c, drivers/isdn/hardware/eicon/platform.h,
    'sshd': 'other', #
    'steal_locks': 'other', # fs/locks.c, include/linux/fs.h,
    'strcmp' : 'copy', # lib/string.c,
    'strlcpy': 'other', # lib/string.c,
    'strlen' : 'copy', # lib/string.c, include/asm-alpha/string.h,
    'strncpy' : 'copy', # lib/string.c, include/asm-alpha/string.h,
    'strncpy_from_user': 'copy', # include/asm-alpha/uaccess.h, include/asm-i386/uaccess.h,
    'strnlen_user': 'other', # include/asm-alpha/uaccess.h, include/asm-i386/uaccess.h,
    'submit_bh' : 'buffer', # fs/buffer.c, include/linux/buffer_head.h,
    'submit_bio' : 'other', # drivers/block/ll_rw_blk.c, include/linux/fs.h,
    'sunrpc': 'other', #
    'svc_authenticate' : 'other', # net/sunrpc/svcauth.c, include/linux/sunrpc/svcauth.h,
    'svc_authorise' : 'other', # net/sunrpc/svcauth.c, include/linux/sunrpc/svcauth.h,
    'svc_deferred_dequeue' : 'other', # net/sunrpc/svcsock.c, net/sunrpc/svcsock.c,
    'svc_drop' : 'other', # net/sunrpc/svcsock.c, include/linux/sunrpc/svcsock.h,
    'svc_expkey_lookup' : 'other', # fs/nfsd/export.c,
    'svc_export_put' : 'other', # fs/nfsd/export.c, include/linux/nfsd/export.h,
    'svc_process' : 'other', # net/sunrpc/svc.c, include/linux/sunrpc/svc.h,
    'svc_recv' : 'other', # net/sunrpc/svcsock.c, include/linux/sunrpc/svcsock.h,
    'svc_reserve' : 'other', # net/sunrpc/svcsock.c, include/linux/sunrpc/svc.h,
    'svc_send' : 'other', # net/sunrpc/svcsock.c, include/linux/sunrpc/svcsock.h,
    'svc_sendto' : 'other', # net/sunrpc/svcsock.c,
    'svc_sock_enqueue' : 'other', # net/sunrpc/svcsock.c,
    'svc_sock_release' : 'other', # net/sunrpc/svcsock.c,
    'svc_udp_data_ready' : 'other', # net/sunrpc/svcsock.c, net/sunrpc/svcsock.c,
    'svc_udp_recvfrom' : 'other', # net/sunrpc/svcsock.c, net/sunrpc/svcsock.c,
    'svc_udp_sendto' : 'other', # net/sunrpc/svcsock.c, net/sunrpc/svcsock.c,
    'svc_write_space' : 'other', # net/sunrpc/svcsock.c,
    'svcauth_unix_accept' : 'other', # net/sunrpc/svcauth_unix.c,
    'svcauth_unix_release' : 'other', # net/sunrpc/svcauth_unix.c,
    'switch_names': 'other', # fs/dcache.c,
    'swpctx_cont' : 'other', #
    'sync_buffer' : 'other', # fs/buffer.c, drivers/oprofile/buffer_sync.c,
    'sync_dirty_buffer' : 'other', # fs/buffer.c, include/linux/buffer_head.h,
    'sync_inode' : 'other', # fs/fs-writeback.c, include/linux/fs.h,
    'sync_mapping_buffers' : 'other', # fs/buffer.c, include/linux/buffer_head.h,
    'sync_sb_inodes': 'other', # fs/fs-writeback.c,
    'sync_supers': 'other', # fs/super.c, include/linux/fs.h,
    'sys_accept' : 'user', # net/socket.c, include/linux/syscalls.h, used to be syscall
    'sys_access': 'other', # fs/open.c, include/linux/syscalls.h,
    'sys_brk': 'user', # mm/mmap.c, mm/nommu.c, include/linux/syscalls.h,
    'sys_clock_gettime': 'user', # kernel/posix-timers.c, include/linux/syscalls.h,
    'sys_clone': 'user', # include/asm-i386/unistd.h,
    'sys_close' : 'user', # fs/open.c, include/linux/syscalls.h, used to be syscall
    'sys_dup2': 'user', # fs/fcntl.c, include/linux/syscalls.h,
    'sys_execve': 'user', # include/asm-alpha/unistd.h, include/asm-i386/unistd.h,
    'sys_exit_group': 'user', # kernel/exit.c, include/linux/syscalls.h,
    'sys_fcntl' : 'user', # fs/fcntl.c, include/linux/syscalls.h, used to be syscall
    'sys_fcntl64': 'user', # fs/fcntl.c, include/linux/syscalls.h,
    'sys_fstat64': 'user', # fs/stat.c, include/linux/syscalls.h,
    'sys_ftruncate': 'user', # fs/open.c, include/linux/syscalls.h,
    'sys_futex': 'user', # kernel/futex.c, include/linux/syscalls.h,
    'sys_getdents64': 'user',
    'sys_geteuid': 'other', # kernel/timer.c, include/linux/syscalls.h,
    'sys_getgroups': 'user', # kernel/sys.c, include/linux/syscalls.h,
    'sys_getpid': 'user', # kernel/timer.c, include/linux/syscalls.h,
    'sys_getppid': 'other', # kernel/timer.c, include/linux/syscalls.h,
    'sys_getrlimit': 'other', # kernel/sys.c, include/linux/syscalls.h,
    'sys_getsockname' : 'user', # net/socket.c, include/linux/syscalls.h, used to be syscall
    'sys_gettimeofday' : 'user', # kernel/time.c, include/linux/syscalls.h, used to be syscall
    'sys_getuid': 'user', # kernel/timer.c, include/linux/syscalls.h,
    'sys_getxpid' : 'user', # used to be syscall
    'sys_int_21' : 'interrupt', #
    'sys_int_22' : 'interrupt', #
    'sys_interrupt' : 'interrupt', #
    'sys_ioctl': 'user', # fs/ioctl.c, drivers/block/cciss.c, include/linux/syscalls.h,
    'sys_kill' : 'user', # kernel/signal.c, include/linux/syscalls.h, used to be syscall
    'sys_llseek': 'other', # fs/read_write.c, include/linux/syscalls.h,
    'sys_lseek': 'user', # fs/read_write.c, include/linux/syscalls.h,
    'sys_mkdir': 'user', # fs/namei.c, include/linux/syscalls.h,
    'sys_mmap2': 'user', # include/asm-i386/unistd.h,
    'sys_mremap': 'user', # mm/mremap.c, include/linux/syscalls.h,
    'sys_munmap': 'user', # mm/mmap.c, mm/nommu.c, include/linux/syscalls.h,
    'sys_nanosleep': 'user', # kernel/timer.c, include/linux/syscalls.h,
    'sys_newlstat' : 'user', # fs/stat.c, include/linux/syscalls.h, used to be syscall
    'sys_newstat' : 'user', # fs/stat.c, include/linux/syscalls.h, used to be syscall
    'sys_newuname': 'user', # kernel/sys.c, include/linux/syscalls.h,
    'sys_open' : 'user', # fs/open.c, include/linux/syscalls.h, used to be syscall
    'sys_pipe': 'interrupt', # include/asm-i386/unistd.h,
    'sys_poll' : 'user', # fs/select.c, include/linux/syscalls.h, used to be syscall
    'sys_read' : 'user', # fs/read_write.c, include/linux/syscalls.h, used to be syscall
    'sys_recv' : 'user', # net/socket.c, include/linux/syscalls.h, used to be syscall
    'sys_recvfrom' : 'user', # net/socket.c, include/linux/syscalls.h, used to be syscall
    'sys_rename': 'other', # fs/namei.c, include/linux/syscalls.h,
    'sys_rmdir': 'user',
    'sys_rt_sigaction': 'user', # arch/alpha/kernel/signal.c, kernel/signal.c, include/asm-alpha/unistd.h, include/asm-i386/unistd.h,
    'sys_rt_sigprocmask': 'user', # kernel/signal.c, include/linux/syscalls.h,
    'sys_select': 'user', # fs/select.c, include/linux/syscalls.h,
    'sys_send' : 'user', # net/socket.c, include/linux/syscalls.h, used to be syscall
    'sys_sendto' : 'user', # net/socket.c, include/linux/syscalls.h, used to be syscall
    'sys_set_thread_area': 'user', #
    'sys_setitimer': 'user', # kernel/itimer.c, include/linux/syscalls.h,
    'sys_shutdown' : 'user', # net/socket.c, include/linux/syscalls.h, used to be syscall
    'sys_sigreturn' : 'user', # used to be syscall
    'sys_sigsuspend' : 'user', # used to be syscall
    'sys_socketcall': 'user', # net/socket.c, include/linux/syscalls.h,
    'sys_stat64': 'user', # fs/stat.c, include/linux/syscalls.h,
    'sys_time': 'other', # kernel/time.c, include/linux/syscalls.h,
    'sys_times': 'other', # kernel/sys.c, include/linux/syscalls.h,
    'sys_umask': 'other', # kernel/sys.c, include/linux/syscalls.h,
    'sys_unlink': 'other', # fs/namei.c, include/linux/syscalls.h,
    'sys_wait4': 'user', # kernel/exit.c, include/linux/syscalls.h,
    'sys_waitpid': 'user', # kernel/exit.c, include/linux/syscalls.h,
    'sys_write' : 'user', # fs/read_write.c, include/linux/syscalls.h, used to be syscall
    'sys_writev' : 'user', # fs/read_write.c, include/linux/syscalls.h, used to be syscall
    'syscall_call': 'other', #
    'syscall_exit': 'other', #
    'sysguard_panelapplet.so': 'other', #
    'syslogd': 'other', #
    'system_call': 'interrupt', #
    'tail': 'other', #
    'task_curr' : 'other', # kernel/sched.c, include/linux/sched.h,
    'task_rq_lock' : 'other', # kernel/sched.c,
    'task_timeslice' : 'other', # kernel/sched.c,
    'tasklet_action' : 'other', # kernel/softirq.c,
    'tcp_accept' : 'stack', # net/ipv4/tcp.c,
    'tcp_ack' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_ack_no_tstamp' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_ack_update_window' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_bucket_destroy' : 'stack', # net/ipv4/tcp_ipv4.c,
    'tcp_check_req' : 'stack', # net/ipv4/tcp_minisocks.c,
    'tcp_child_process' : 'stack', # net/ipv4/tcp_minisocks.c,
    'tcp_clean_rtx_queue' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_clear_xmit_timers' : 'stack', # net/ipv4/tcp_timer.c,
    'tcp_close' : 'stack', # net/ipv4/tcp.c,
    'tcp_close_state' : 'stack', # net/ipv4/tcp.c,
    'tcp_copy_to_iovec' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_create_openreq_child' : 'stack', # net/ipv4/tcp_minisocks.c,
    'tcp_current_mss': 'other', #
    'tcp_cwnd_application_limited': 'stack', # net/ipv4/tcp_input.c,
    'tcp_data_queue' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_delete_keepalive_timer' : 'stack', # net/ipv4/tcp_timer.c,
    'tcp_destroy_sock' : 'stack', # net/ipv4/tcp.c,
    'tcp_enter_quickack_mode' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_event_data_recv' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_fin' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_fixup_rcvbuf' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_fixup_sndbuf' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_fragment': 'stack', # net/ipv4/tcp_output.c,
    'tcp_incr_quickack' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_init_buffer_space' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_init_metrics' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_init_xmit_timers' : 'stack', # net/ipv4/tcp_timer.c,
    'tcp_invert_tuple' : 'stack', # net/ipv4/netfilter/ip_conntrack_proto_tcp.c,
    'tcp_make_synack' : 'stack', # net/ipv4/tcp_output.c,
    'tcp_new' : 'stack', # net/ipv4/netfilter/ip_conntrack_proto_tcp.c,
    'tcp_new_space' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_ofo_queue' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_packet' : 'stack', # net/ipv4/netfilter/ip_conntrack_proto_tcp.c,
    'tcp_parse_options' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_pkt_to_tuple' : 'stack', # net/ipv4/netfilter/ip_conntrack_proto_tcp.c,
    'tcp_poll' : 'stack', # net/ipv4/tcp.c,
    'tcp_prequeue_process' : 'stack', # net/ipv4/tcp.c,
    'tcp_push_one' : 'stack', # net/ipv4/tcp_output.c,
    'tcp_put_port' : 'stack', # net/ipv4/tcp_ipv4.c,
    'tcp_queue_skb' : 'stack', # net/ipv4/tcp_output.c,
    'tcp_rcv_established' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_rcv_rtt_update' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_rcv_space_adjust' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_rcv_state_process' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_recvmsg' : 'stack', # net/ipv4/tcp.c,
    'tcp_reset_keepalive_timer' : 'stack', # net/ipv4/tcp_timer.c,
    'tcp_rtt_estimator' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_send_ack' : 'stack', # net/ipv4/tcp_output.c,
    'tcp_send_delayed_ack' : 'stack', # net/ipv4/tcp_output.c,
    'tcp_send_fin' : 'stack', # net/ipv4/tcp_output.c,
    'tcp_sendmsg' : 'stack', # net/ipv4/tcp.c,
    'tcp_set_skb_tso_segs': 'other', #
    'tcp_shutdown' : 'stack', # net/ipv4/tcp.c,
    'tcp_sync_mss' : 'stack', # net/ipv4/tcp_output.c,
    'tcp_time_wait' : 'stack', # net/ipv4/tcp_minisocks.c,
    'tcp_transmit_skb' : 'stack', # net/ipv4/tcp_output.c,
    'tcp_trim_head': 'stack', # net/ipv4/tcp_output.c,
    'tcp_tso_acked': 'stack', #
    'tcp_tw_schedule' : 'stack', # net/ipv4/tcp_minisocks.c,
    'tcp_unhash' : 'stack', # net/ipv4/tcp_ipv4.c,
    'tcp_update_metrics' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_urg' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_v4_checksum_init':'stack',
    'tcp_v4_conn_request' : 'stack', # net/ipv4/tcp_ipv4.c,
    'tcp_v4_connect': 'stack', # net/ipv4/tcp_ipv4.c,
    'tcp_v4_destroy_sock' : 'stack', # net/ipv4/tcp_ipv4.c,
    'tcp_v4_do_rcv' : 'stack', # net/ipv4/tcp_ipv4.c,
    'tcp_v4_hnd_req' : 'stack', # net/ipv4/tcp_ipv4.c,
    'tcp_v4_init_sock': 'stack', # net/ipv4/tcp_ipv4.c,
    'tcp_v4_rcv' : 'stack', # net/ipv4/tcp_ipv4.c,
    'tcp_v4_rebuild_header' : 'stack', # net/ipv4/tcp_ipv4.c,
    'tcp_v4_route_req' : 'stack', # net/ipv4/tcp_ipv4.c,
    'tcp_v4_search_req' : 'stack', # net/ipv4/tcp_ipv4.c,
    'tcp_v4_send_check' : 'stack', # net/ipv4/tcp_ipv4.c, net/ipv4/tcp_ipv4.c,
    'tcp_v4_send_synack' : 'stack', # net/ipv4/tcp_ipv4.c,
    'tcp_v4_syn_recv_sock' : 'stack', # net/ipv4/tcp_ipv4.c,
    'tcp_v4_synq_add' : 'stack', # net/ipv4/tcp_ipv4.c,
    'tcp_vegas_init' : 'stack', # net/ipv4/tcp_input.c,
    'tcp_write_xmit' : 'stack', # net/ipv4/tcp_output.c,
    'test_clear_page_dirty': 'buffer', # mm/page-writeback.c, include/linux/page-flags.h,
    'test_clear_page_writeback' : 'buffer', # mm/page-writeback.c, include/linux/page-flags.h,
    'test_set_page_writeback' : 'buffer', # mm/page-writeback.c, include/linux/page-flags.h,
    'timer_interrupt' : 'interrupt', # arch/alpha/kernel/time.c, arch/alpha/kernel/proto.h,
    'tr': 'other', #
    'truncate_complete_page': 'buffer', # mm/truncate.c,
    'truncate_inode_pages': 'buffer', # mm/truncate.c, include/linux/mm.h,
    'try_to_wake_up' : 'other', # kernel/sched.c,
    'tsunami_readb': 'driver',
    'tsunami_readl' : 'interrupt', # include/asm-alpha/core_tsunami.h,
    'tsunami_update_irq_hw' : 'interrupt', # arch/alpha/kernel/sys_dp264.c,
    'tsunami_writeb': 'driver',
    'tsunami_writel' : 'interrupt', # include/asm-alpha/core_tsunami.h,
    'tty_hung_up_p': 'driver', # drivers/char/tty_io.c, include/linux/tty.h,
    'tty_ldisc_deref': 'other', #
    'tty_ldisc_ref_wait': 'other', #
    'tty_ldisc_try': 'other', #
    'tty_open': 'driver', # drivers/char/tty_io.c, drivers/char/tty_io.c, drivers/net/wan/sdla_chdlc.c,
    'tty_poll': 'driver', # drivers/char/tty_io.c, drivers/char/tty_io.c,
    'tty_write': 'driver', # drivers/char/tty_io.c, drivers/char/tty_io.c,
    'udp_checksum_init' : 'stack', # net/ipv4/udp.c,
    'udp_ioctl': 'stack', # net/ipv4/udp.c,
    'udp_packet' : 'stack', # net/ipv4/netfilter/ip_conntrack_proto_udp.c,
    'udp_pkt_to_tuple' : 'stack', # net/ipv4/netfilter/ip_conntrack_proto_udp.c,
    'udp_push_pending_frames' : 'stack', # net/ipv4/udp.c,
    'udp_queue_rcv_skb' : 'stack', # net/ipv4/udp.c,
    'udp_rcv' : 'stack', # net/ipv4/udp.c,
    'udp_recvmsg': 'stack', # net/ipv4/udp.c,
    'udp_sendmsg' : 'stack', # net/ipv4/udp.c,
    'udp_sendpage' : 'stack', # net/ipv4/udp.c,
    'udp_v4_get_port': 'stack', # net/ipv4/udp.c,
    'udp_v4_lookup_longway' : 'stack', # net/ipv4/udp.c,
    'unalign_trap_cont' : 'alignment',
    'unalign_trap_count' : 'alignment',
    'undo_switch_stack' : 'other', #
    'unix': 'other', #
    'unlock_buffer' : 'other', # fs/buffer.c,
    'unlock_new_inode': 'other', # fs/inode.c, include/linux/fs.h,
    'unlock_page' : 'buffer', # mm/filemap.c,
    'unmap_mapping_range': 'buffer', # mm/memory.c, include/linux/mm.h,
    'unmap_page_range': 'buffer', # mm/memory.c,
    'unmap_region': 'buffer', # mm/mmap.c,
    'unmap_underlying_metadata' : 'other', # fs/buffer.c, include/linux/buffer_head.h,
    'unmap_vma': 'buffer', # mm/mmap.c,
    'unmap_vma_list': 'buffer', # mm/mmap.c,
    'unmap_vmas': 'buffer', # mm/memory.c, include/linux/mm.h,
    'unmask_IO_APIC_irq': 'interrupt', #
    'unmask_IO_APIC_vector': 'interrupt', #
    'unqueue_me': 'other', # kernel/futex.c,
    'unshare_files': 'other', # kernel/fork.c, include/linux/fs.h,
    'up' : 'driver', # arch/alpha/kernel/semaphore.c, include/asm-alpha/semaphore.h, net/ipv4/netfilter/ip_tables.c, net/ipv6/netfilter/ip6_tables.c, drivers/video/atafb.c, include/asm-alpha/semaphore.h,
    'update_atime': 'other', # fs/inode.c, include/linux/fs.h,
    'update_one_process' : 'other', # kernel/timer.c,
    'update_process_times' : 'other', # kernel/timer.c, include/linux/sched.h,
    'update_wall_time' : 'other', # kernel/timer.c,
    'update_wall_time_one_tick' : 'other', # kernel/timer.c,
    'usbcore': 'other', #
    'vfs_create': 'other', # fs/namei.c, include/linux/fs.h,
    'vfs_fstat': 'other', # fs/stat.c, include/linux/fs.h,
    'vfs_getattr' : 'user', # fs/stat.c, include/linux/fs.h, used to be syscall
    'vfs_llseek': 'other', # fs/read_write.c, include/linux/fs.h,
    'vfs_lstat' : 'user', # fs/stat.c, include/linux/fs.h, used to be syscall
    'vfs_mkdir': 'other', # fs/namei.c, include/linux/fs.h,
    'vfs_permission' : 'user', # fs/namei.c, include/linux/fs.h, used to be syscall
    'vfs_read' : 'user', # fs/read_write.c, include/linux/fs.h, used to be syscall
    'vfs_rename': 'other', # fs/namei.c, include/linux/fs.h,
    'vfs_rename_other': 'other', # fs/namei.c,
    'vfs_stat' : 'user', # fs/stat.c, include/linux/fs.h, used to be syscall
    'vfs_unlink': 'other', # fs/namei.c, include/linux/fs.h,
    'vfs_write' : 'user', # fs/read_write.c, include/linux/fs.h, used to be syscall
    'vfs_writev' : 'user', # fs/read_write.c, include/linux/fs.h, used to be syscall
    'vma_adjust': 'buffer', # mm/mmap.c, include/linux/mm.h,
    'vma_link': 'buffer', # mm/mmap.c,
    'vma_merge': 'buffer', # mm/mmap.c, include/linux/mm.h,
    'vma_prio_tree_add': 'buffer', # mm/prio_tree.c, include/linux/mm.h,
    'vma_prio_tree_insert': 'buffer', # mm/prio_tree.c, include/linux/mm.h,
    'vma_prio_tree_remove': 'buffer', # mm/prio_tree.c, include/linux/mm.h,
    'vmstat_open': 'other', # fs/proc/proc_misc.c,
    'vmstat_show': 'buffer', # mm/page_alloc.c,
    'vmtruncate': 'buffer', # mm/nommu.c, mm/memory.c, include/linux/mm.h,
    'vsnprintf' : 'other', # lib/vsprintf.c, include/linux/kernel.h,
    'vsprintf' : 'driver', # lib/vsprintf.c, arch/alpha/boot/main.c, drivers/scsi/aic7xxx_old/aic7xxx_proc.c, include/linux/kernel.h,
    'wait_for_completion': 'driver', # drivers/acorn/block/mfmhd.c, kernel/sched.c,
    'wait_on_page_writeback_range' : 'buffer', # mm/filemap.c,
    'wait_task_zombie': 'other', # kernel/exit.c,
    'wake_futex': 'other', # kernel/futex.c,
    'wake_up_buffer' : 'other', # fs/buffer.c, include/linux/buffer_head.h,
    'wake_up_inode' : 'other', # fs/inode.c, include/linux/writeback.h,
    'wake_up_new_task': 'other', #
    'wake_up_page' : 'buffer', # mm/filemap.c,
    'wake_up_process' : 'other', # kernel/sched.c,
    'wake_up_state' : 'other', # kernel/sched.c,
    'wb_timer_fn': 'buffer', # mm/page-writeback.c, mm/page-writeback.c,
    'wc': 'other', #
    'work_notifysig': 'other', #
    'work_pending' : 'other', #
    'work_resched': 'other', #
    'worker_thread': 'other', # kernel/workqueue.c,
    'write_boundary_block' : 'other', # fs/buffer.c, include/linux/buffer_head.h,
    'write_chan': 'driver', # drivers/char/n_tty.c,
    'write_inode' : 'other', # fs/fs-writeback.c,
    'write_null': 'driver', # drivers/char/mem.c,
    'writeback_acquire': 'other', # fs/fs-writeback.c, include/linux/backing-dev.h,
    'writeback_in_progress' : 'other', # fs/fs-writeback.c, include/linux/backing-dev.h,
    'writeback_inodes': 'other', # fs/fs-writeback.c, include/linux/writeback.h,
    'xdr_partial_copy_from_skb' : 'copy', # net/sunrpc/xdr.c, include/linux/sunrpc/xdr.h,
    'xfrm_lookup' : 'stack', # net/xfrm/xfrm_policy.c,
    'xmms': 'other', #
    'zap_pmd_range': 'buffer', # mm/memory.c,
    'zap_pte_range': 'buffer', # mm/memory.c,
    'zone_statistics' : 'buffer', # mm/page_alloc.c,
    'libaprutil-0.so.0' : 'user',
    'libapr-0.so.0' : 'user',
    'httpd' : 'user',
    'do_tcp_sendpages': 'copy',
    'tcp_setsockopt' : 'stack',
    'sys_setsockopt' : 'stack',
    'do_sendfile' : 'copy',
    'ip_route_output_slow': 'stack',
    'tcp_sendpage': 'copy',
    'file_send_actor': 'copy',
    'flush_tlb_page': 'buffer',
    'sock_common_setsockopt': 'stack',
    'sock_sendpage': 'copy',

#
#   New functions
#

    '__alloc_percpu': 'buffer', # mm/slab.c, include/linux/percpu.h,
    '__pskb_pull_tail': 'stack', # net/core/skbuff.c, include/linux/skbuff.h,
    '__reml': 'other', # arch/alpha/kernel/alpha_ksyms.c,
    '__tasklet_hi_schedule': 'interrupt', # kernel/softirq.c,
    '__tcp_checksum_complete_user': 'stack', # net/ipv4/tcp_input.c,
    '__tcp_v4_lookup_listener': 'stack', # net/ipv4/tcp_ipv4.c,
    '__tcp_v4_rehash': 'stack', # net/ipv4/tcp_ipv4.c,
    '__tcp_westwood_fast_bw': 'stack', # net/ipv4/tcp_input.c,
    '__tcp_westwood_slow_bw': 'stack', # net/ipv4/tcp_input.c,
    '__xfrm_policy_check': 'stack', # net/xfrm/xfrm_policy.c,
    'alcor_disable_irq': 'interrupt', # arch/alpha/kernel/sys_alcor.c,
    'alpha_read_fp_reg': 'other', # arch/alpha/lib/fpreg.c, arch/alpha/kernel/proto.h, arch/alpha/math-emu/math.c, include/asm-alpha/fpu.h,
    'atkbd_probe': 'other', # drivers/input/keyboard/atkbd.c,
    'background_writeout': 'buffer', # mm/page-writeback.c, mm/page-writeback.c,
    'bad_page': 'buffer', # mm/page_alloc.c,
    'batch_entropy_process': 'other', # drivers/char/random.c, drivers/char/random.c,
    'block_hotplug_filter': 'driver', # drivers/block/genhd.c,
    'brioctl_set': 'stack', # net/socket.c, include/linux/if_bridge.h,
    'cdev_put': 'fs', # fs/char_dev.c, include/linux/cdev.h,
    'change_protection': 'buffer', # mm/mprotect.c,
    'check_timer_failed': 'interrupt', # kernel/timer.c,
    'clipper_disable_irq': 'interrupt', # arch/alpha/kernel/sys_dp264.c,
    'clipper_enable_irq': 'interrupt', # arch/alpha/kernel/sys_dp264.c,
    'count_active_tasks': 'interrupt', # kernel/timer.c,
    'csum_ipv6_magic': 'stack', # include/asm-i386/checksum.h, include/asm-alpha/checksum.h,
    'del_timer_sync': 'interrupt', # kernel/timer.c, include/linux/timer.h, include/linux/timer.h,
    'dev_ifname': 'stack', # net/core/dev.c,
    'dev_queue_xmit_nit': 'stack', # net/core/dev.c, include/linux/netdevice.h,
    'dev_valid_name': 'stack', # net/core/dev.c,
    'do_entDbg': 'interrupt', # arch/alpha/kernel/traps.c,
    'do_proc_dointvec_jiffies_conv': 'interrupt', # kernel/sysctl.c,
    'down_interruptible': 'interrupt', # arch/alpha/kernel/semaphore.c, include/asm-alpha/semaphore.h, include/asm-i386/semaphore.h, include/asm-alpha/semaphore.h, net/ipv4/netfilter/ip_tables.c, net/ipv6/netfilter/ip6_tables.c,
    'drain_array': 'buffer', #
    'drain_cpu_caches': 'buffer', # mm/slab.c,
    'dummy_file_fcntl': 'other', # security/dummy.c,
    'dummy_sem_semop': 'other', # security/dummy.c,
    'emit_log_char': 'other', # kernel/printk.c,
    'entDbg': 'interrupt', # arch/alpha/kernel/proto.h,
    'entIF': 'interrupt', # arch/alpha/kernel/proto.h,
    'eth_header_cache_update': 'stack', # net/ethernet/eth.c, include/linux/etherdevice.h,
    'eth_header_parse': 'stack', # net/ethernet/eth.c, include/linux/etherdevice.h,
    'ethtool_get_settings': 'stack', # net/core/ethtool.c,
    'fifo_open': 'fs', # fs/fifo.c,
    'find_trylock_page': 'buffer', # mm/filemap.c, include/linux/pagemap.h,
    'find_undo': 'buffer', # ipc/sem.c,
    'find_user': 'buffer', # kernel/user.c, include/linux/sched.h,
    'flow_cache_cpu_prepare': 'stack', # net/core/flow.c,
    'flow_cache_flush_per_cpu': 'stack', # net/core/flow.c,
    'flow_cache_flush_tasklet': 'stack', # net/core/flow.c,
    'flow_key_compare': 'stack', # net/core/flow.c,
    'flush_icache_user_range': 'interrupt', # arch/alpha/kernel/smp.c, include/asm-alpha/cacheflush.h, include/asm-alpha/cacheflush.h, include/asm-i386/cacheflush.h,
    'flush_tlb_mm': 'interrupt', # arch/alpha/kernel/smp.c, include/asm-alpha/tlbflush.h, include/asm-i386/tlbflush.h, include/asm-alpha/tlbflush.h, include/asm-i386/tlbflush.h,
    'force_page_cache_readahead': 'buffer', # mm/readahead.c, include/linux/mm.h,
    'free_percpu': 'buffer', # mm/slab.c, include/linux/percpu.h, include/linux/percpu.h,
    'generic_file_sendfile': 'buffer', # mm/filemap.c, include/linux/fs.h,
    'get_one_pte_map': 'buffer', # mm/mremap.c,
    'gunzip': 'other', # lib/inflate.c,
    'handle_ipi': 'interrupt', # arch/alpha/kernel/smp.c, arch/alpha/kernel/proto.h,
    'input_devices_read': 'driver', # drivers/input/input.c,
    'input_link_handle': 'driver', # drivers/input/input.c,
    'input_register_device': 'driver', # drivers/input/input.c, include/linux/input.h,
    'insb': 'driver', # arch/alpha/kernel/io.c, include/asm-alpha/io.h,
    'insl': 'driver', # arch/alpha/kernel/io.c, include/asm-alpha/io.h, drivers/net/smc9194.c, drivers/net/smc9194.c,
    'invalidate_bh_lru': 'fs', # fs/buffer.c,
    'iommu_arena_alloc': 'interrupt', # arch/alpha/kernel/pci_iommu.c,
    'iommu_arena_find_pages': 'interrupt', # arch/alpha/kernel/pci_iommu.c,
    'iommu_arena_free': 'interrupt', # arch/alpha/kernel/pci_iommu.c,
    'ip_compute_csum': 'stack', # arch/alpha/lib/checksum.c, include/asm-i386/checksum.h, include/asm-alpha/checksum.h,
    'ip_getsockopt': 'stack', # net/ipv4/ip_sockglue.c,
    'ip_mc_output': 'stack', # net/ipv4/ip_output.c,
    'ip_options_compile': 'stack', # net/ipv4/ip_options.c,
    'ip_rt_dump': 'stack', # net/ipv4/route.c,
    'ipc_checkid': 'stack', # ipc/util.c, ipc/util.h,
    'ipc_lock': 'stack', # ipc/util.c, ipc/util.h,
    'ipc_unlock': 'stack', # ipc/util.c, ipc/util.h,
    'ipcperms': 'stack', # ipc/util.c, ipc/util.h,
    'ipi_flush_tlb_page': 'interrupt', # arch/alpha/kernel/smp.c,
    'isp1020_intr_handler': 'other', # drivers/scsi/qlogicisp.c, drivers/scsi/qlogicisp.c,
    'isp1020_queuecommand': 'other', # drivers/scsi/qlogicisp.c, drivers/scsi/qlogicisp.h,
    'kernel_thread': 'interrupt', # include/asm-um/processor-generic.h, include/asm-alpha/processor.h, include/asm-i386/processor.h,
    'kmem_find_general_cachep': 'buffer', # mm/slab.c,
    'kmem_ptr_validate': 'buffer', # mm/slab.c,
    'llc_mac_hdr_init': 'stack', # net/llc/llc_output.c, net/llc/llc_output.h,
    'lock_rename': 'fs', # fs/namei.c, include/linux/namei.h,
    'lookup_undo': 'stack', # ipc/sem.c,
    'memcpy_tokerneliovec': 'stack', # net/core/iovec.c, include/linux/socket.h,
    'migrate_task': 'other', # kernel/sched.c,
    'net_ratelimit': 'stack', # net/core/utils.c, include/linux/net.h,
    'netlink_release': 'stack', # net/netlink/netlink_dev.c, net/netlink/af_netlink.c,
    'nf_log_packet': 'stack', # net/core/netfilter.c, include/linux/netfilter_logging.h, include/linux/netfilter.h,
    'nf_queue': 'stack', # net/core/netfilter.c,
    'nr_free_zone_pages': 'buffer', # mm/page_alloc.c,
    'osf_writev': 'driver', # arch/alpha/kernel/osf_sys.c,
    'pci_map_sg': 'driver', # arch/alpha/kernel/pci-noop.c, arch/alpha/kernel/pci_iommu.c, include/asm-generic/pci-dma-compat.h, include/asm-alpha/pci.h,
    'pci_unmap_sg': 'driver', # arch/alpha/kernel/pci-noop.c, arch/alpha/kernel/pci_iommu.c, include/asm-generic/pci-dma-compat.h, include/asm-alpha/pci.h,
    'pcibios_align_resource': 'driver', # arch/alpha/kernel/pci.c, include/linux/pci.h,
    'pfifo_fast_requeue': 'stack', # net/sched/sch_generic.c,
    'pointer_lock': 'interrupt', # arch/alpha/kernel/smp.c,
    'posix_unblock_lock': 'fs', # fs/locks.c, include/linux/fs.h,
    'prepare_timeout': 'interrupt', # ipc/mqueue.c,
    'printk': 'other', # kernel/printk.c, drivers/md/raid6.h,
    'process_mcheck_info': 'interrupt', # arch/alpha/kernel/irq_alpha.c, arch/alpha/kernel/proto.h,
    'read_cache_pages': 'buffer', # mm/readahead.c, include/linux/pagemap.h,
    'register_gifconf': 'stack', # net/core/dev.c, include/linux/netdevice.h,
    'rwsem_down_read_failed': 'interrupt', # lib/rwsem.c, include/asm-alpha/rwsem.h,
    'search_exception_tables': 'interrupt', # kernel/extable.c, include/linux/module.h,
    'security_fixup_ops': 'other', # security/dummy.c, security/security.c,
    'send_ipi_message': 'interrupt', # arch/alpha/kernel/smp.c,
    'send_sig_info': 'interrupt', # kernel/signal.c, include/linux/sched.h,
    'set_fs_altroot': 'fs', # fs/namei.c, include/linux/fs_struct.h,
    'sg_classify': 'interrupt', # arch/alpha/kernel/pci_iommu.c,
    'sg_fill': 'interrupt', # arch/alpha/kernel/pci_iommu.c,
    'sk_common_release': 'stack', # net/core/sock.c,
    'sk_stream_wait_connect': 'stack', # net/core/stream.c,
    'skb_over_panic': 'stack', # net/core/skbuff.c, include/linux/skbuff.h,
    'skb_under_panic': 'stack', # net/core/skbuff.c, include/linux/skbuff.h,
    'smp_call_function_on_cpu': 'interrupt', # arch/alpha/kernel/smp.c, include/asm-alpha/smp.h, include/asm-alpha/smp.h,
    'sock_def_write_space': 'stack', # net/core/sock.c,
    'sock_getsockopt': 'stack', # net/core/sock.c,
    'sock_wait_for_wmem': 'stack', # net/core/sock.c,
    'srm_dispatch': 'other', #
    'srm_fixup': 'other', # include/asm-alpha/console.h,
    'stxcpy_aligned': 'buffer', #
    'sys_capset': 'other', # kernel/capability.c, include/linux/syscalls.h,
    'sys_fadvise64': 'buffer', # mm/fadvise.c, include/linux/syscalls.h,
    'sys_fadvise64_64': 'buffer', # mm/fadvise.c, include/linux/syscalls.h,
    'sys_newfstat': 'fs', # fs/stat.c, include/linux/syscalls.h,
    'sys_semop': 'stack', # ipc/sem.c, include/linux/syscalls.h,
    'sys_semtimedop': 'stack', # ipc/sem.c, include/linux/syscalls.h,
    'sys_sendfile64': 'fs', # fs/read_write.c, include/linux/syscalls.h,
    'sys_socketpair': 'stack', # net/socket.c, include/linux/syscalls.h,
    'sys_vhangup': 'fs', # fs/open.c, include/linux/syscalls.h,
    'tasklet_hi_action': 'interrupt', # kernel/softirq.c,
    'tcp_ack_probe': 'stack', # net/ipv4/tcp_input.c,
    'tcp_advertise_mss': 'stack', # net/ipv4/tcp_output.c,
    'tcp_enter_loss': 'stack', # net/ipv4/tcp_input.c,
    'tcp_fastretrans_alert': 'stack', # net/ipv4/tcp_input.c,
    'tcp_ioctl': 'stack', # net/ipv4/tcp.c,
    'tcp_process_frto': 'stack', # net/ipv4/tcp_input.c,
    'tcp_rcv_synsent_state_process': 'stack', # net/ipv4/tcp_input.c,
    'tcp_recv_urg': 'stack', # net/ipv4/tcp.c,
    'tcp_reset': 'stack', # net/ipv4/tcp_input.c,
    'tcp_retransmit_skb': 'stack', # net/ipv4/tcp_output.c,
    'tcp_sacktag_write_queue': 'stack', # net/ipv4/tcp_input.c,
    'tcp_time_to_recover': 'stack', # net/ipv4/tcp_input.c,
    'tcp_v4_err': 'stack', # net/ipv4/tcp_ipv4.c,
    'tcp_v4_get_port': 'stack', # net/ipv4/tcp_ipv4.c,
    'tcp_v4_lookup': 'stack', # net/ipv4/tcp_ipv4.c, net/ipv4/tcp_diag.c,
    'tcp_v4_reselect_saddr': 'stack', # net/ipv4/tcp_ipv4.c,
    'this_rq_lock': 'interrupt', #
    'tr_source_route': 'stack', # net/802/tr.c, include/linux/trdevice.h,
    'try_atomic_semop': 'stack', # ipc/sem.c,
    'tsunami_outw': 'driver', #
    'twothirdsMD4Transform': 'other', # drivers/char/random.c,
    'unregister_netdevice': 'stack', # net/core/dev.c, include/linux/netdevice.h,
    'update_queue': 'stack', # ipc/sem.c,
    'vegas_cong_avoid': 'stack', # net/ipv4/tcp_input.c,
    'vm_acct_memory': 'buffer', # mm/swap.c, include/linux/mman.h,
    'vsscanf': 'other', # lib/vsprintf.c, include/linux/kernel.h,
    'wait_for_packet': 'stack', # net/core/datagram.c,
    'westwood_update_window': 'stack', # net/ipv4/tcp_input.c,
    'within_one_quad': 'other', #
}

pc_categories_re = [
#    ( re.compile('.*'), 'other' )
]

def pc_categorize(symbol):
    from categories import pc_categories, pc_categories_re
    if symbol in pc_categories:
        return pc_categories[symbol]
    for regexp, category in pc_categories_re:
        if regexp.match(symbol):
            return category

    return None

