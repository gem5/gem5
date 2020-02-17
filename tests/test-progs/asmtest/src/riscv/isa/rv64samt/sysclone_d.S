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
// sysclone_d tests basic functionalities of clone system call:
//    - create a new thread
//    - assign a new per-thread stack frame to the child thread
//    - assign a new per-thread TLS to the child thread
//
// In addition to testing clone(), sysclone_d partially checks
// functionalities of futex and exit system calls that are used to
// facilitate thread exit and synchronization.
//------------------------------------------------------------------------

#include "riscv_test.h"
#include "test_macros.h"
#include "test_macros_mt_ecall.h"

  RVTEST_RV64U
  RVTEST_CODE_BEGIN

#define MAX_NUM_THREADS 20

//------------------------------------------------------------------------
// Master thread creates new threads, waits for all threads to complete,
// deallocates threads and checks result
//------------------------------------------------------------------------
  li      a0, MAX_NUM_THREADS
  call    _create_threads

  la      t6, n_worker_threads
  ld      a0, (t6)
  beqz    a0, _fail                   // exit if there's no worker thread
  call    _join

  la      t6, n_worker_threads
  ld      a0, (t6)
  call    _check

  la      t6, n_worker_threads
  ld      a0, (t6)
  call    _delete_threads

  li      a0, SUCCESS

  RVTEST_CODE_END

//------------------------------------------------------------------------
// mt_test function executed by child threads
//------------------------------------------------------------------------
_mt_test:
  // get this thread's TID
  li      a7, SYSCALL_GETTID
  ecall

  // store the TID to both stack and TLS of this thread
  addi    sp, sp, -8
  sd      a0, (sp)
  sd      a0, (tp)

  RVTEST_CODE_END

//------------------------------------------------------------------------
// _check:
//    The master thread looks into the stack and TLS of each child thread
//    and check if the child thread's TID was written in both places.
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
//    This function takes a number of threads to check in a0
//------------------------------------------------------------------------

_check:
  mv      t0, a0          // get the number of threads
  mv      s0, ra          // save return register
  mv      s1, sp          // save stack pointer
1:
  ld      t1, (sp)        // get child_thread_saved_id

  addi    sp, sp, 8
  ld      t2, (sp)        // get child_thread_id
  bnez    t2, _fail       // this child_thread_id should have been cleared

  addi    sp, sp, 8
  ld      t3, (sp)        // get child_tls_ptr
  ld      t3, (t3)        // get the first value stored in child's TLS
  bne     t1, t3, _fail   // child_tid should have been saved in the TLS

  addi    sp, sp, 8
  ld      t4, (sp)        // get child_stack_ptr
  li      t5, MEM_SIZE
  add     t4, t4, t5      // get the high address of child's stack
  ld      t4, -8(t4)      // get the first value stored in child's stack
  bne     t1, t4, _fail   // child_tid should have been saved in the stack

  addi    sp, sp, 8

  // decrement the number of threads to wait for
  addi    t0, t0, -1
  bnez    t0, 1b

  // finish checking all threads
  mv      ra, s0                  // restore return register
  mv      sp, s1                  // restore stack pointer
  ret

_fail:
  li        a0, FAILURE
  RVTEST_CODE_END

  .data

MT_DATA
