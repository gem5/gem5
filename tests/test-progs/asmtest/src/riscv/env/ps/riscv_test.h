// See LICENSE for license details.

#ifndef _ENV_PHYSICAL_SINGLE_CORE_H
#define _ENV_PHYSICAL_SINGLE_CORE_H

#include "../encoding.h"

//-----------------------------------------------------------------------
// Begin Macro
//-----------------------------------------------------------------------

#define RVTEST_RV64U                                                    \
  .macro init;                                                          \
  .endm

#define RVTEST_RV64UF                                                   \
  .macro init;                                                          \
  RVTEST_FP_ENABLE;                                                     \
  .endm

#define RVTEST_RV32U                                                    \
  .macro init;                                                          \
  .endm

#define RVTEST_RV32UF                                                   \
  .macro init;                                                          \
  RVTEST_FP_ENABLE;                                                     \
  .endm

#define RVTEST_RV64M                                                    \
  .macro init;                                                          \
  RVTEST_ENABLE_MACHINE;                                                \
  .endm

#define RVTEST_RV64S                                                    \
  .macro init;                                                          \
  RVTEST_ENABLE_SUPERVISOR;                                             \
  .endm

#define RVTEST_RV32M                                                    \
  .macro init;                                                          \
  RVTEST_ENABLE_MACHINE;                                                \
  .endm

#define RVTEST_RV32S                                                    \
  .macro init;                                                          \
  RVTEST_ENABLE_SUPERVISOR;                                             \
  .endm

#if __riscv_xlen == 64
# define CHECK_XLEN li a0, 1; slli a0, a0, 31; bgez a0, 1f; RVTEST_PASS; 1:
#else
# define CHECK_XLEN li a0, 1; slli a0, a0, 31; bltz a0, 1f; RVTEST_PASS; 1:
#endif

#define INIT_PMP                                                        \
  la t0, 1f;                                                            \
  csrw mtvec, t0;                                                       \
  li t0, -1;        /* Set up a PMP to permit all accesses */           \
  csrw pmpaddr0, t0;                                                    \
  li t0, PMP_NAPOT | PMP_R | PMP_W | PMP_X;                             \
  csrw pmpcfg0, t0;                                                     \
  .align 2;                                                             \
1:

#define INIT_SATP                                                      \
  la t0, 1f;                                                            \
  csrw mtvec, t0;                                                       \
  csrwi sptbr, 0;                                                       \
  .align 2;                                                             \
1:

#define DELEGATE_NO_TRAPS                                               \
  la t0, 1f;                                                            \
  csrw mtvec, t0;                                                       \
  csrwi medeleg, 0;                                                     \
  csrwi mideleg, 0;                                                     \
  csrwi mie, 0;                                                         \
  .align 2;                                                             \
1:

#define RVTEST_ENABLE_SUPERVISOR                                        \
  li a0, MSTATUS_MPP & (MSTATUS_MPP >> 1);                              \
  csrs mstatus, a0;                                                     \
  li a0, SIP_SSIP | SIP_STIP;                                           \
  csrs mideleg, a0;                                                     \

#define RVTEST_ENABLE_MACHINE                                           \
  li a0, MSTATUS_MPP;                                                   \
  csrs mstatus, a0;                                                     \

#define RVTEST_FP_ENABLE                                                \
  li a0, MSTATUS_FS & (MSTATUS_FS >> 1);                                \
  csrs mstatus, a0;                                                     \
  csrwi fcsr, 0

#define RISCV_MULTICORE_DISABLE                                         \
  csrr a0, mhartid;                                                     \
  1: bnez a0, 1b

#define EXTRA_TVEC_USER
#define EXTRA_TVEC_MACHINE
#define EXTRA_INIT
#define EXTRA_INIT_TIMER

//-----------------------------------------------------------------------
// Begin Macro
//    Jump to the first test case
//-----------------------------------------------------------------------

#define RVTEST_CODE_BEGIN                                               \
        .section .text.init;                                            \
        .align  6;                                                      \
        .weak stvec_handler;                                            \
        .weak mtvec_handler;                                            \
        .globl _start;                                                  \
_start:                                                                 \
        la t0, 1f;                                                      \
        jr t0;                                                          \
        .align 2;                                                       \
1:

//-----------------------------------------------------------------------
// RVTEST_CODE_END Macro
//    Call exit syscall to terminate the simulation
//-----------------------------------------------------------------------

#define EXIT_SYSCALL 93
#define RVTEST_CODE_END                                                 \
        li a7, EXIT_SYSCALL;                                            \
        ecall

//-----------------------------------------------------------------------
// RVTEST_PASS Macro
//    Pass 0 as a return code to an EXIT ecall
//-----------------------------------------------------------------------

#define TESTNUM gp
#define RVTEST_PASS                                                     \
        fence;                                                          \
        li a0, 0;

//-----------------------------------------------------------------------
// RVTEST_FAIL Macro
//    Pass test case number as a return code to an EXIT ecall
//-----------------------------------------------------------------------

#define TESTNUM gp
#define RVTEST_FAIL                                                     \
        fence;                                                          \
        mv a0, TESTNUM;                                                 \
        RVTEST_CODE_END

//-----------------------------------------------------------------------
// Data Section Macro
//-----------------------------------------------------------------------

#define EXTRA_DATA

#define RVTEST_DATA_BEGIN                                               \
        EXTRA_DATA                                                      \
        .pushsection .tohost,"aw",@progbits;                            \
        .align 6; .global tohost; tohost: .dword 0;                     \
        .align 6; .global fromhost; fromhost: .dword 0;                 \
        .popsection;                                                    \
        .align 4; .global begin_signature; begin_signature:

#define RVTEST_DATA_END .align 4; .global end_signature; end_signature:

#endif
