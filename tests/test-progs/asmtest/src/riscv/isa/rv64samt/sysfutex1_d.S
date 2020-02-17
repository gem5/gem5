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
// sysfutex1_d tests basic functionalities of futex system call:
//    - make some threads wait on a variable
//    - wake up all threads waiting on a variable
//------------------------------------------------------------------------

#include "riscv_test.h"
#include "test_macros.h"
#include "test_macros_mt_ecall.h"

  RVTEST_RV64U
  RVTEST_CODE_BEGIN

#define MAX_NUM_THREADS 20

//------------------------------------------------------------------------
// Master thread creates new threads, call _master function, waits for all
// threads to complete, deallocates threads and checks result
//------------------------------------------------------------------------
  li      a0, MAX_NUM_THREADS
  call    _create_threads

  la      t6, n_worker_threads
  ld      a0, (t6)
  beqz    a0, _fail                   // exit if there's no worker thread

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
//
//    - wake up all threads waiting on futex_X
//------------------------------------------------------------------------
_master_work:
  mv    s0, ra                  // save return address
  li    t0, 0                   // number of threads that have been waken
  la    t1, n_worker_threads
  ld    t1, (t1)

1:
  // futex(futex_X, FUTEX_WAKE_PRIVATE, n_worker_threads)
  la    a0, futex_X
  li    a1, FUTEX_WAKE_PRIVATE
  li    a2, 1                   // wake up at most 1 thread
  li    a7, SYSCALL_FUTEX
  ecall

  add   t0, t0, a0              // track the number of waken threads so far

  // keep waking up until all threads are waken up
  blt   t0, t1, 1b

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
  li    a1, FUTEX_WAIT_PRIVATE
  li    a2, 0                   // expected val of futex_X
  li    a7, SYSCALL_FUTEX
  ecall

  RVTEST_CODE_END

//------------------------------------------------------------------------
// _check:
//    Each thread should do LOOP_COUNT iterations
//------------------------------------------------------------------------

_check:
  ret

_fail:
  li        a0, FAILURE
  RVTEST_CODE_END

  .data

futex_X:  .dword  0
futex_Y:  .dword  0

count_master:   .dword  0
count_child:    .dword  0

MT_DATA
