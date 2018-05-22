# See LICENSE for license details.

#*****************************************************************************
# amoadd_d.S
#-----------------------------------------------------------------------------
#
# Test amoadd.d instruction.
#

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

la        a0, shared_var
amoadd.w  t0, t1, 0(a0)
lr.w      t2, 0(a0)
//sc.w      t0, t1, 0(a0)

RVTEST_CODE_END

  .data
shared_var:     .dword    0
non_shared_var: .dword    0
