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
 *
 * Authors: Tuan Ta
 */

//------------------------------------------------------------------------
// This code tests amoswap.d instruction in multi-threading system.
// All threads execute a critical section LOOP_COUNT times. A thread
// gets into a critical section by acquiring a lock variable (i.e.,
// shared_var) and checking return value.
// 0 means the lock is not being locked. Each thread increments
// a variable (i.e., var) inside the critical section and releases the
// lock by swapping back 0 to the lock variable.
// The master thread (i.e., thread 0) waits for all threads to complete
// and compare the var's value to the expected result.
//------------------------------------------------------------------------

#include "riscv_test.h"
#include "test_macros.h"
#include "test_macros_mt.h"

  RVTEST_RV64U
  RVTEST_CODE_BEGIN

#define LOOP_COUNT  1000
#define RESULT      NUM_THREADS * LOOP_COUNT

//------------------------------------------------------------------------
// Master thread creates new threads, waits for all threads to complete,
// deallocates threads and checks result
//------------------------------------------------------------------------
  call _create_threads
  call _join
  call _delete_threads
  call _check

  RVTEST_CODE_END

//------------------------------------------------------------------------
// mt_test function executed in child threads
// A child thread signals its completion by atomicaly adding 1 to barrier
//------------------------------------------------------------------------
_mt_test:
  li        t0, 1               // initialize the swap value (1-locked)
  li        t1, LOOP_COUNT
  la        t2, var             // load the var's address
  la        a0, shared_var

1:
  amoswap.d.aq  s2, t0, (a0)    // try to swap t0 with the lock
  bnez          s2, 1b          // retry if the lock is being held

  lw            t3, (t2)        // load the var's value
  addi          t3, t3, 1       // add 1 to the value
  sw            t3, (t2)        // store the new value to var

  amoswap.d.rl  zero, zero, (a0)// release the lock by swapping back 0

  addi          t1, t1, -1      // decrement the loop_count
  bnez          t1, 1b          // repeat if not done yet

  la            a0, barrier
  amoadd.d      zero, t0, (a0)  // signal this thread's completion

  RVTEST_CODE_END

//------------------------------------------------------------------------
// Master thread checks result
//------------------------------------------------------------------------
_check:
  la        a0, var
  li        a1, RESULT
  ld        a0, (a0)
  bne       a0, a1, _fail
  li        a0, SUCCESS
  ret

_fail:
  li        a0, FAILURE
  ret

  .data

MT_DATA
var: .dword   0
