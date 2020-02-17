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
// threads (i.e., 1 master thread and 3 child threads)
//------------------------------------------------------------------------

#ifndef __TEST_MACROS_MT_H
#define __TEST_MACROS_MT_H

#define SYSCALL_MMAP    222
#define SYSCALL_MUNMAP  215
#define SYSCALL_CLONE   220

#define STACK_SIZE      (4096 * 1024)

#define PROT_READ	      0x1
#define PROT_WRITE	    0x2
#define MMAP_PROT_FLAGS (PROT_READ | PROT_WRITE)

#define MAP_PRIVATE	    0x02
#define MAP_ANONYMOUS	  0x20
#define MAP_STACK	      0x20000
#define MMAP_MAP_FLAGS  (MAP_PRIVATE | MAP_ANONYMOUS | MAP_STACK)

#define CLONE_VM	      0x00000100
#define CLONE_FS	      0x00000200
#define CLONE_FILES	    0x00000400
#define CLONE_SIGHAND	  0x00000800
#define CLONE_PARENT	  0x00008000
#define CLONE_THREAD	  0x00010000
#define CLONE_IO		    0x80000000
#define CLONE_FLAGS     (CLONE_VM | CLONE_FS | CLONE_FILES | CLONE_SIGHAND\
                         | CLONE_PARENT | CLONE_THREAD | CLONE_IO)

#define NUM_THREADS     3

#define FAILURE         1
#define SUCCESS         0

#define HARTID          0xF14

//------------------------------------------------------------------------
// create NUM_THREADS child threads
//------------------------------------------------------------------------
_create_threads:
  li      t0, NUM_THREADS
  mv      s0, ra                  // save return register
1:
  jal     ra, _alloc_stack
  addi    sp, sp, -8
  sd      a0, (sp)                // save pointer to the new stack
  jal     ra, _clone_thread       // clone a new thread
  addi    t0, t0, -1
  bnez    t0, 1b
  mv      ra, s0                  // restore return register
  ret

_alloc_stack:
  li      a0, 0
  li      a1, STACK_SIZE
  li      a2, MMAP_PROT_FLAGS
  li      a3, MMAP_MAP_FLAGS
  li      a4, -1
  li      a5, 0
  li      a7, SYSCALL_MMAP
  ecall
  ret

_clone_thread:
  li      a1, STACK_SIZE
  add     a1, a1, a0
  li      a0, CLONE_FLAGS
  li      a7, SYSCALL_CLONE
  ecall
  beqz    a0, _mt_test
  ret

//------------------------------------------------------------------------
// wait for all child threads to exit
//------------------------------------------------------------------------
_join:
  la      t0, barrier
  li      t1, NUM_THREADS
1:
  ld      t2, (t0)
  bne     t1, t2, 1b
  ret

//------------------------------------------------------------------------
// deallocate NUM_THREADS child threads
//------------------------------------------------------------------------
_delete_threads:
  li      t0, NUM_THREADS
  mv      s0, ra                  // save return register
1:
  ld      a0, (sp)                // pop the new stack's pointer
  addi    sp, sp, 8
  jal     ra, _dealloc_stack
  addi    t0, t0, -1
  bnez    t0, 1b
  mv      ra, s0                  // restore return register
  ret

_dealloc_stack:
  li      a1, STACK_SIZE
  li      a7, SYSCALL_MUNMAP
  ecall
  ret

#define MT_DATA                                                           \
  shared_var:   .dword    0;                                              \
  barrier:      .dword    0;                                              \
  array:        .dword    0x00000000deadbeef,                             \
                          0xdeadbeefdeadbeef,                             \
                          0x12343eeaaf423451;                             \

#endif
