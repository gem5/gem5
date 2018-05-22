# See LICENSE for license details.

#*****************************************************************************
# addi.S
#-----------------------------------------------------------------------------
#
# Test addi instruction.
#

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  li    a2, 100
  li    a0, 0
  li    a0, 0
  li    a0, 0
  li    a0, 0

loop:
  addi  a1, a0, 1
  addi  a1, a0, 1
  addi  a1, a0, 1
  addi  a1, a0, 1
  addi  a1, a0, 1
  addi  a1, a0, 1
  addi  a1, a0, 1
  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
  addi  a2, a2, -1
  bnez  a2, loop

#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  mul   a1, a2, a3
#  mul   a1, a2, a3
#  mul   a1, a2, a3
#  mul   a1, a2, a3
#  mul   a1, a2, a3
#  mul   a1, a2, a3
#  mul   a1, a2, a3
#  mul   a1, a2, a3
#  mul   a1, a2, a3
#  mul   a1, a2, a3
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1
#  addi  a1, a0, 1


RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

RVTEST_DATA_END
