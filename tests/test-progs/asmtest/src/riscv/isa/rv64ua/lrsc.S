# See LICENSE for license details.

#*****************************************************************************
# lrsr.S
#-----------------------------------------------------------------------------
#
# Test LR/SC instructions.
#

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

# get a unique core id
la a0, coreid
li a1, 1
amoadd.w a2, a1, (a0)

# for now, only run this on core 0
1:li a3, 1
bgeu a2, a3, 1b

1: lw a1, (a0)
bltu a1, a3, 1b

# make sure that sc without a reservation fails.
TEST_CASE( 2, a4, 1, \
  la a0, foo; \
  sc.w a4, x0, (a0); \
)

# make sure that sc with the wrong reservation fails.
# TODO is this actually mandatory behavior?
TEST_CASE( 3, a4, 1, \
  la a0, foo; \
  add a1, a0, 1024; \
  lr.w a1, (a1); \
  sc.w a4, a1, (a0); \
)

#define LOG_ITERATIONS 10

# have each core add its coreid+1 to foo 1024 times
la a0, foo
li a1, 1<<LOG_ITERATIONS
addi a2, a2, 1
1: lr.w a4, (a0)
add a4, a4, a2
sc.w a4, a4, (a0)
bnez a4, 1b
add a1, a1, -1
bnez a1, 1b

# wait for all cores to finish
la a0, barrier
li a1, 1
amoadd.w x0, a1, (a0)
1: lw a1, (a0)
blt a1, a3, 1b
fence

# expected result is 512*ncores*(ncores+1)
TEST_CASE( 4, a0, 0, \
  lw a0, foo; \
  slli a1, a3, LOG_ITERATIONS-1; \
1:sub a0, a0, a1; \
  addi a3, a3, -1; \
  bgez a3, 1b
)

TEST_PASSFAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

coreid: .word 0
barrier: .word 0
foo: .word 0
RVTEST_DATA_END
