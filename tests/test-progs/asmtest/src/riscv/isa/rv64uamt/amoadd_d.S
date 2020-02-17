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
// This code tests amoadd_d instruction in multi-threading system.
// Each thread increments a globally shared variable LOOP_COUNT times.
// Once a thread completes, it signals its completition by atomically
// incrementing a barrier variable.
// Master thread (i.e., thread 0) waits for all threads to complete by
// spinning on the barrier variable until all threads update the variable.
// Then, the master thread checks the shared variable's value.
//------------------------------------------------------------------------

#include "riscv_test.h"
#include "test_macros.h"
#include "test_macros_mt.h"

  RVTEST_RV64U
  RVTEST_CODE_BEGIN

#define LOOP_COUNT  1000
#define RESULT      LOOP_COUNT * NUM_THREADS

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
  li        t0, 1               // one operand of amoadd_w
  li        t1, LOOP_COUNT      // loop count
  la        a0, shared_var
1:
  amoadd.d  zero, t0, (a0)
  addi      t1, t1, -1
  bnez      t1, 1b

  la        a0, barrier
  amoadd.d  zero, t0, (a0)

  RVTEST_CODE_END

//------------------------------------------------------------------------
// Master thread checks result
//------------------------------------------------------------------------
_check:
  la        a0, shared_var
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
