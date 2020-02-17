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
// sysfutex3_d tests FUTEX_CMP_REQUEUE functionalities of futex system
// call:
//    - make worker threads waiting on futex 1
//    - wake up 1 thread, requeue the rest of the threads to futex 2
//    - wake all threads waiting on futex 1 and futex 2
//------------------------------------------------------------------------

#include "riscv_test.h"
#include "test_macros.h"
#include "test_macros_mt_ecall.h"

  RVTEST_RV64U
  RVTEST_CODE_BEGIN

#define NUM_THREADS 20

//------------------------------------------------------------------------
// Master thread creates new threads, call _master function, waits for all
// threads to complete, deallocates threads and checks result
//------------------------------------------------------------------------
  li      a0, NUM_THREADS
  call    _create_threads

  la      t6, n_worker_threads
  ld      a0, (t6)
  beqz    a0, _fail        // exit if there's no worker thread

  call    _master_work

  la      t6, n_worker_threads
  ld      a0, (t6)
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
// master_work function executed by the parent/master thread
//------------------------------------------------------------------------
_master_work:
  mv    s0, ra                  // save return address
  li    t0, 0                   // number of threads that have been waken
  la    t1, n_worker_threads
  ld    t1, (t1)

1:
  // futex(futex_X, FUTEX_CMP_REQUEUE, 1, INT_MAX, futex_Y, *futex_X)
  la    a0, futex_X
  li    a1, FUTEX_CMP_REQUEUE
  li    a2, 1                   // wake up at most 1 thread
  li    a3, 1000                // practically is INT_MAX
  la    a4, futex_Y             // move all other waiter to futex_Y
  li    a5, 0
  li    a7, SYSCALL_FUTEX
  ecall

  // loop until wake up one thread, all other waiter are requeued to
  // futex_Y
  beqz  a0, 1b

  addi  t0, t0, 1

2:
  // alternating between futex_X and futex_Y
  // because there could be new threads added to futex_X's queue
  // after our futex_requeue

  // futex(futex_Y, FUTEX_WAKE_PRIVATE, 0)
  la    a0, futex_Y
  li    a1, FUTEX_WAKE
  li    a2, 1                   // wake up at most 1 thread
  li    a7, SYSCALL_FUTEX
  ecall

  add   t0, t0, a0              // track the number of waken threads so far

  // futex(futex_X, FUTEX_WAKE_PRIVATE, 0)
  la    a0, futex_X
  li    a1, FUTEX_WAKE
  li    a2, 1                   // wake up at most 1 thread
  li    a7, SYSCALL_FUTEX
  ecall

  add   t0, t0, a0              // track the number of waken threads so far

  // keep waking up until all threads are waken up
  blt   t0, t1, 2b

  // restore return address and return
  mv    ra, s0
  ret

//------------------------------------------------------------------------
// mt_test function executed by child threads
//
//    Wait on futex_X
//------------------------------------------------------------------------
_mt_test:
  // futex(futex_X, FUTEX_WAIT_PRIVATE, 1)
  la    a0, futex_X
  li    a1, FUTEX_WAIT
  li    a2, 0                   // expected val of futex_X
  li    a7, SYSCALL_FUTEX
  ecall

  RVTEST_CODE_END

//------------------------------------------------------------------------
// _check:
//    counter should equals to number of child threads
//------------------------------------------------------------------------

_check:
  ret

_fail:
  li        a0, FAILURE
  RVTEST_CODE_END

  .data

futex_X:  .dword  0
futex_Y:  .dword  0

MT_DATA
