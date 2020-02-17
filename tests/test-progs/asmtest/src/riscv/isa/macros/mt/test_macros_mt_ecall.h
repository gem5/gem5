/*
 * Copyright (c) 2018, Cornell University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 *
 * Neither the name of Cornell University nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

//------------------------------------------------------------------------
// This test_macros includes necessary functions and macros to create
// and exit threads. They're used in multi-threaded assembly tests.
// This assumes the target system can concurrently support 4 different
// threads (i.e., 1 master thread and 3 child threads).
//
// Threads are synchronized through futex system call (i.e., wait and
// wakeup operations).
//------------------------------------------------------------------------

#ifndef __TEST_MACROS_MT_FUTEX_H
#define __TEST_MACROS_MT_FUTEX_H

#define SYSCALL_FUTEX         98
#define SYSCALL_GETTID        178
#define SYSCALL_MUNMAP        215
#define SYSCALL_CLONE         220
#define SYSCALL_MMAP          222

#define MEM_SIZE              (4096 * 1024)

#define PROT_READ             0x1
#define PROT_WRITE            0x2
#define MMAP_PROT_FLAGS       (PROT_READ | PROT_WRITE)

#define MAP_PRIVATE           0x02
#define MAP_ANONYMOUS         0x20
#define MAP_STACK             0x20000
#define MMAP_MAP_FLAGS        (MAP_PRIVATE | MAP_ANONYMOUS | MAP_STACK)

#define CLONE_VM              0x00000100
#define CLONE_FS              0x00000200
#define CLONE_FILES           0x00000400
#define CLONE_SIGHAND         0x00000800
#define CLONE_PARENT          0x00008000
#define CLONE_THREAD          0x00010000
#define CLONE_IO              0x80000000
#define CLONE_PARENT_SETTID   0x00100000	/* set the TID in the parent */
#define CLONE_CHILD_CLEARTID  0x00200000	/* clear the TID in the child */
#define CLONE_SETTLS          0x00080000
#define CLONE_FLAGS           (CLONE_VM | CLONE_FS | CLONE_FILES \
                              | CLONE_SIGHAND | CLONE_PARENT \
                              | CLONE_THREAD | CLONE_IO \
                              | CLONE_PARENT_SETTID \
                              | CLONE_CHILD_CLEARTID \
                              | CLONE_SETTLS)

#define FUTEX_WAIT            0
#define FUTEX_WAKE            1
#define FUTEX_CMP_REQUEUE     4
#define FUTEX_WAKE_OP         5
#define FUTEX_WAIT_BITSET     9
#define FUTEX_WAKE_BITSET     10
#define FUTEX_PRIVATE_FLAG    128
#define FUTEX_CLOCK_REALTIME  256
#define FUTEX_CMD_MASK        ~(FUTEX_PRIVATE_FLAG | FUTEX_CLOCK_REALTIME)

#define FUTEX_OP_SET          0  /* uaddr2 = oparg; */
#define FUTEX_OP_ADD          1  /* uaddr2 += oparg; */
#define FUTEX_OP_OR           2  /* uaddr2 |= oparg; */
#define FUTEX_OP_ANDN         3  /* uaddr2 &= ~oparg; */
#define FUTEX_OP_XOR          4  /* uaddr2 ^= oparg; */
#define FUTEX_OP_ARG_SHIFT    8  /* Use (1 << oparg) as operand */

#define FUTEX_OP_CMP_EQ       0  /* if (oldval == cmparg) wake */
#define FUTEX_OP_CMP_NE       1  /* if (oldval != cmparg) wake */
#define FUTEX_OP_CMP_LT       2  /* if (oldval < cmparg) wake */
#define FUTEX_OP_CMP_LE       3  /* if (oldval <= cmparg) wake */
#define FUTEX_OP_CMP_GT       4  /* if (oldval > cmparg) wake */
#define FUTEX_OP_CMP_GE       5  /* if (oldval >= cmparg) wake */

#define FUTEX_OP(op, oparg, cmp, cmparg)                    \
                (((op & 0xf) << 28) |                       \
                 ((cmp & 0xf) << 24) |                      \
                 ((oparg & 0xfff) << 12) |                  \
                 (cmparg & 0xfff))

#define FUTEX_WAIT_PRIVATE        (FUTEX_WAIT | FUTEX_PRIVATE_FLAG)
#define FUTEX_WAKE_PRIVATE        (FUTEX_WAKE | FUTEX_PRIVATE_FLAG)
#define FUTEX_WAIT_BITSET_PRIVATE (FUTEX_WAIT_BITSET | FUTEX_PRIVATE_FLAG)
#define FUTEX_WAKE_BITSET_PRIVATE (FUTEX_WAKE_BITSET | FUTEX_PRIVATE_FLAG)

#define FAILURE               1
#define SUCCESS               0

//------------------------------------------------------------------------
// _create_threads: create a given number of threads
//
//    The calling thread (a.k.a, master thread) saves information about its
//    child threads in its stack in the following structure:
//
//    | child_stack_ptr_0       |  << fp: frame pointer
//    | child_tls_ptr_0         |
//    | child_thread_id_0       |
//    | saved_child_thread_id_0 |
//    | child_stack_ptr_1       |
//    | child_tls_ptr_1         |
//    | child_thread_id_1       |
//    | saved_child_thread_id_1 |
//    | ...                     |  << sp: stack pointer
//
//    For each child thread, we need to save the following information
//    in the parent thread's stack frame:
//
//    - child_stack_ptr stores the lower address of the child thread's
//      stack space
//
//    - child_tls_ptr stores the lower address of the child thread's
//      thread local storage (TLS)
//
//    - child_thread_id stores the thread ID of the child thread. This
//      variable will be cleared by the child thread when it exits.
//
//    - saved_child_thread_id also stores the thread ID of the child
//      thread, but this variable is used only by the parent thread.
//
//    This function takes the number of threads to create in a0. It
//    updates n_child_threads variable to the number of successfully
//    created threads.
//------------------------------------------------------------------------

_create_threads:
  mv      t0, a0                // get the number of threads
  mv      s0, ra                // save return register
  la      t3, n_worker_threads
1:
  // allocate a new stack space and save its pointer in the caller's stack
  jal     ra, _alloc_mem
  addi    sp, sp, -8
  sd      a0, (sp)
  mv      t1, a0

  // allocate a new thread local storage (TLS) and save its pointer in the
  // caller's stack
  jal     ra, _alloc_mem
  addi    sp, sp, -8
  sd      a0, (sp)
  mv      t2, a0

  // allocate space in the caller's stack to store new thread ID
  addi    sp, sp, -8

  // clone a new thread
  li      a0, CLONE_FLAGS
  li      s2, MEM_SIZE
  add     a1, t1, s2        // pointer to the high address of the new stack
  mv      a2, sp            // ptid
  mv      a3, t2            // pointer to the low address of the new TLS,
                            // assuming TLS grows upward
  mv      a4, sp            // ctid
  li      a7, SYSCALL_CLONE // clone syscall number
  ecall                     // call clone syscall
  bltz    a0, 2f            // syscall error
  beqz    a0, _mt_test      // only the new thread jumps to _mt_test

  // save child thread ID in the caller's stack
  addi      sp, sp, -8
  sd        a0, (sp)

  // decrement the number of threads to create
  addi      t0, t0, -1

  // increment the number of successfully created threads sofar
  addi      t4, zero, 1
  amoadd.d  zero, t4, (t3)

  // check if we still need to spawn more threads
  bnez      t0, 1b
  j         3f
2:
  // handle clone syscall error by deleting the last memory frame created
  // for the unsuccessfully spawned thread.
  addi      sp, sp, 8       // skip child_thread_id

  // deallocate last allocated tls
  ld        a0, (sp)
  jal       ra, _dealloc_mem
  addi      sp, sp, 8

  // deallocate last allocated stack
  ld        a0, (sp)
  jal       ra, _dealloc_mem
  addi      sp, sp, 8
3:
  // finish creating threads
  mv        ra, s0
  ret

//------------------------------------------------------------------------
// _alloc_mem: allocate a memory space with size MEM_SIZE
//
//    This function returns the pointer to the newly allocated memory
//    space in a0
//------------------------------------------------------------------------

_alloc_mem:
  li      a0, 0
  li      a1, MEM_SIZE
  li      a2, MMAP_PROT_FLAGS
  li      a3, MMAP_MAP_FLAGS
  li      a4, -1
  li      a5, 0
  li      a7, SYSCALL_MMAP
  ecall
  ret

//------------------------------------------------------------------------
// _delete_threads: deallocate all child threads
//
//    This function assumes the following structure in the calling thread's
//    stack frame
//
//    | child_stack_ptr_0       |  << fp: frame pointer
//    | child_tls_ptr_0         |
//    | child_thread_id_0       |
//    | saved_child_thread_id_0 |
//    | child_stack_ptr_1       |
//    | child_tls_ptr_1         |
//    | child_thread_id_1       |
//    | saved_child_thread_id_1 |
//    | ...                     |  << sp: stack pointer
//
//    This function takes the number of threads to delete in a0
//------------------------------------------------------------------------

_delete_threads:
  mv      t0, a0                  // get the number of threads to delete
  mv      s0, ra                  // save return register
1:
  addi    sp, sp, 8               // skip saved_child_thread_id
  addi    sp, sp, 8               // skip child_thread_id

  // deallocate thread's tls
  ld      a0, (sp)
  jal     ra, _dealloc_mem
  addi    sp, sp, 8

  // deallocate thread's stack
  ld      a0, (sp)
  jal     ra, _dealloc_mem
  addi    sp, sp, 8

  // decrement the number of threads to delete
  addi    t0, t0, -1
  bnez    t0, 1b

  // finish deleting all threads
  mv      ra, s0                  // restore return register
  ret

//------------------------------------------------------------------------
// _dealloc_mem: deallocate memory space of size MEM_SIZE
//
//    This function takes the pointer to the memory space in a0
//------------------------------------------------------------------------

_dealloc_mem:
  li      a1, MEM_SIZE
  li      a7, SYSCALL_MUNMAP
  ecall
  ret

//------------------------------------------------------------------------
// _join: wait for all child threads to exit
//
//    Child threads are created with CLONE_CHILD_CLEARTID flag, so when
//    they exit, they will clear the ctid/ptid variable and wake up their
//    parent thread.
//
//    This function assumes the following structure in the calling thread's
//    stack frame
//
//    | child_stack_ptr_0       |  << fp: frame pointer
//    | child_tls_ptr_0         |
//    | child_thread_id_0       |
//    | saved_child_thread_id_0 |
//    | child_stack_ptr_1       |
//    | child_tls_ptr_1         |
//    | child_thread_id_1       |
//    | saved_child_thread_id_1 |
//    | ...                     |  << sp: stack pointer
//
//    This function takes a number of threads to wait in a0
//------------------------------------------------------------------------

_join:
  mv      t0, a0          // get the number of threads
  mv      s0, ra          // save return register
  mv      s1, sp          // save stack pointer
1:
  // Calling futex_wait on ctidptr
  ld      a2, (sp)                // get child thread ID from
                                  // saved_child_thread_id
  addi    sp, sp, 8
  mv      a0, sp                  // futex address (child_thread_id)
  li      a1, FUTEX_WAIT_PRIVATE
  li      a7, SYSCALL_FUTEX
  ecall

  addi    sp, sp, 8              // skip child_tls_ptr
  addi    sp, sp, 8              // skip child_stack_ptr

  // decrement the number of threads to wait for
  addi    t0, t0, -1
  bnez    t0, 1b

  // finish waiting for all threads
  mv      ra, s0                  // restore return register
  mv      sp, s1                  // restore stack pointer
  ret

#define MT_DATA                                                           \
  n_worker_threads:     .dword    0;                                      \
  shared_var:           .dword    0;                                      \
  barrier:              .dword    0;                                      \
  array:                .dword    0x00000000deadbeef,                     \
                                  0xdeadbeefdeadbeef,                     \
                                  0x12343eeaaf423451;                     \

#endif
