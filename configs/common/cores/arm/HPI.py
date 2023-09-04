# Copyright (c) 2014-2017, 2020 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""The High-Performance In-order (HPI) CPU timing model is tuned to be
representative of a modern in-order ARMv8-A implementation. The HPI
core and its supporting simulation scripts, namely starter_se.py and
starter_fs.py (under /configs/example/arm/) are part of the ARM
Research Starter Kit on System Modeling. More information can be found
at: http://www.arm.com/ResearchEnablement/SystemModeling

"""

from m5.objects import *

# Simple function to allow a string of [01x_] to be converted into a
# mask and value for use with MinorFUTiming
def make_implicant(implicant_string):
    ret_mask = 0
    ret_match = 0

    shift = False
    for char in implicant_string:
        char = char.lower()
        if shift:
            ret_mask <<= 1
            ret_match <<= 1

        shift = True
        if char == "_":
            shift = False
        elif char == "0":
            ret_mask |= 1
        elif char == "1":
            ret_mask |= 1
            ret_match |= 1
        elif char == "x":
            pass
        else:
            print("Can't parse implicant character", char)

    return (ret_mask, ret_match)


#                          ,----- 36 thumb
#                          | ,--- 35 bigThumb
#                          | |,-- 34 aarch64
a64_inst = make_implicant("0_01xx__xxxx_xxxx_xxxx_xxxx__xxxx_xxxx_xxxx_xxxx")
a32_inst = make_implicant("0_00xx__xxxx_xxxx_xxxx_xxxx__xxxx_xxxx_xxxx_xxxx")
t32_inst = make_implicant("1_10xx__xxxx_xxxx_xxxx_xxxx__xxxx_xxxx_xxxx_xxxx")
t16_inst = make_implicant("1_00xx__xxxx_xxxx_xxxx_xxxx__xxxx_xxxx_xxxx_xxxx")
any_inst = make_implicant("x_xxxx__xxxx_xxxx_xxxx_xxxx__xxxx_xxxx_xxxx_xxxx")
#                          | ||
any_a64_inst = make_implicant(
    "x_x1xx__xxxx_xxxx_xxxx_xxxx__xxxx_xxxx_xxxx_xxxx"
)
any_non_a64_inst = make_implicant(
    "x_x0xx__xxxx_xxxx_xxxx_xxxx__xxxx_xxxx_xxxx_xxxx"
)


def encode_opcode(pattern):
    def encode(opcode_string):
        a64_mask, a64_match = pattern
        mask, match = make_implicant(opcode_string)
        return (a64_mask | mask), (a64_match | match)

    return encode


a64_opcode = encode_opcode(a64_inst)
a32_opcode = encode_opcode(a32_inst)
t32_opcode = encode_opcode(t32_inst)
t16_opcode = encode_opcode(t16_inst)

# These definitions (in some form) should probably be part of TimingExpr


def literal(value):
    def body(env):
        ret = TimingExprLiteral()
        ret.value = value
        return ret

    return body


def bin(op, left, right):
    def body(env):
        ret = TimingExprBin()
        ret.op = "timingExpr" + op
        ret.left = left(env)
        ret.right = right(env)
        return ret

    return body


def un(op, arg):
    def body(env):
        ret = TimingExprUn()
        ret.op = "timingExpr" + op
        ret.arg = arg(env)
        return ret

    return body


def ref(name):
    def body(env):
        if name in env:
            ret = TimingExprRef()
            ret.index = env[name]
        else:
            print("Invalid expression name", name)
            ret = TimingExprNull()
        return ret

    return body


def if_expr(cond, true_expr, false_expr):
    def body(env):
        ret = TimingExprIf()
        ret.cond = cond(env)
        ret.trueExpr = true_expr(env)
        ret.falseExpr = false_expr(env)
        return ret

    return body


def src_reg(index):
    def body(env):
        ret = TimingExprSrcReg()
        ret.index = index
        return ret

    return body


def let(bindings, expr):
    def body(env):
        ret = TimingExprLet()
        let_bindings = []
        new_env = {}
        i = 0

        # Make the sub-expression as null to start with
        for name, binding in bindings:
            new_env[name] = i
            i += 1

        defns = []
        # Then apply them to the produced new env
        for i in range(0, len(bindings)):
            name, binding_expr = bindings[i]
            defns.append(binding_expr(new_env))

        ret.defns = defns
        ret.expr = expr(new_env)

        return ret

    return body


def expr_top(expr):
    return expr([])


class HPI_DefaultInt(MinorFUTiming):
    description = "HPI_DefaultInt"
    mask, match = any_non_a64_inst
    srcRegsRelativeLats = [3, 3, 2, 2, 2, 1, 0]


class HPI_DefaultA64Int(MinorFUTiming):
    description = "HPI_DefaultA64Int"
    mask, match = any_a64_inst
    # r, l, (c)
    srcRegsRelativeLats = [2, 2, 2, 0]


class HPI_DefaultMul(MinorFUTiming):
    description = "HPI_DefaultMul"
    mask, match = any_non_a64_inst
    # f, f, f, r, l, a?
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 0]


class HPI_DefaultA64Mul(MinorFUTiming):
    description = "HPI_DefaultA64Mul"
    mask, match = any_a64_inst
    # a (zr for mul), l, r
    srcRegsRelativeLats = [0, 0, 0, 0]
    # extraCommitLat = 1


class HPI_DefaultVfp(MinorFUTiming):
    description = "HPI_DefaultVfp"
    mask, match = any_non_a64_inst
    # cpsr, z, z, z, cpacr, fpexc, l_lo, r_lo, l_hi, r_hi (from vadd2h)
    srcRegsRelativeLats = [5, 5, 5, 5, 5, 5, 2, 2, 2, 2, 2, 2, 2, 2, 0]


class HPI_DefaultA64Vfp(MinorFUTiming):
    description = "HPI_DefaultA64Vfp"
    mask, match = any_a64_inst
    # cpsr, cpacr_el1, fpscr_exc, ...
    srcRegsRelativeLats = [5, 5, 5, 2]


class HPI_FMADD_A64(MinorFUTiming):
    description = "HPI_FMADD_A64"
    mask, match = a64_opcode("0001_1111_0x0x_xxxx__0xxx_xxxx_xxxx_xxxx")
    #                                    t
    # cpsr, cpacr_el1, fpscr_exc, 1, 1, 2, 2, 3, 3, fpscr_exc, d, d, d, d
    srcRegsRelativeLats = [5, 5, 5, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0]


class HPI_FMSUB_D_A64(MinorFUTiming):
    description = "HPI_FMSUB_D_A64"
    mask, match = a64_opcode("0001_1111_0x0x_xxxx__1xxx_xxxx_xxxx_xxxx")
    #                                    t
    # cpsr, cpacr_el1, fpscr_exc, 1, 1, 2, 2, 3, 3, fpscr_exc, d, d, d, d
    srcRegsRelativeLats = [5, 5, 5, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0]


class HPI_FMOV_A64(MinorFUTiming):
    description = "HPI_FMOV_A64"
    mask, match = a64_opcode("0001_1110_0x10_0000__0100_00xx_xxxx_xxxx")
    # cpsr, cpacr_el1, fpscr_exc, 1, 1, 2, 2, 3, 3, fpscr_exc, d, d, d, d
    srcRegsRelativeLats = [5, 5, 5, 0]


class HPI_ADD_SUB_vector_scalar_A64(MinorFUTiming):
    description = "HPI_ADD_SUB_vector_scalar_A64"
    mask, match = a64_opcode("01x1_1110_xx1x_xxxx__1000_01xx_xxxx_xxxx")
    # cpsr, z, z, z, cpacr, fpexc, l0, r0, l1, r1, l2, r2, l3, r3 (for vadd2h)
    srcRegsRelativeLats = [5, 5, 5, 4]


class HPI_ADD_SUB_vector_vector_A64(MinorFUTiming):
    description = "HPI_ADD_SUB_vector_vector_A64"
    mask, match = a64_opcode("0xx0_1110_xx1x_xxxx__1000_01xx_xxxx_xxxx")
    # cpsr, z, z, z, cpacr, fpexc, l0, r0, l1, r1, l2, r2, l3, r3 (for vadd2h)
    srcRegsRelativeLats = [5, 5, 5, 4]


class HPI_FDIV_scalar_32_A64(MinorFUTiming):
    description = "HPI_FDIV_scalar_32_A64"
    mask, match = a64_opcode("0001_1110_001x_xxxx__0001_10xx_xxxx_xxxx")
    extraCommitLat = 6
    srcRegsRelativeLats = [0, 0, 0, 20, 4]


class HPI_FDIV_scalar_64_A64(MinorFUTiming):
    description = "HPI_FDIV_scalar_64_A64"
    mask, match = a64_opcode("0001_1110_011x_xxxx__0001_10xx_xxxx_xxxx")
    extraCommitLat = 15
    srcRegsRelativeLats = [0, 0, 0, 20, 4]


# CINC CINV CSEL CSET CSETM CSINC CSINC CSINV CSINV CSNEG
class HPI_Cxxx_A64(MinorFUTiming):
    description = "HPI_Cxxx_A64"
    mask, match = a64_opcode("xx01_1010_100x_xxxx_xxxx__0xxx_xxxx_xxxx")
    srcRegsRelativeLats = [3, 3, 3, 2, 2]


class HPI_DefaultMem(MinorFUTiming):
    description = "HPI_DefaultMem"
    mask, match = any_non_a64_inst
    srcRegsRelativeLats = [1, 1, 1, 1, 1, 2]
    # Assume that LDR/STR take 2 cycles for resolving dependencies
    # (1 + 1 of the FU)
    extraAssumedLat = 2


class HPI_DefaultMem64(MinorFUTiming):
    description = "HPI_DefaultMem64"
    mask, match = any_a64_inst
    srcRegsRelativeLats = [2]
    # Assume that LDR/STR take 2 cycles for resolving dependencies
    # (1 + 1 of the FU)
    extraAssumedLat = 3


class HPI_DataProcessingMovShiftr(MinorFUTiming):
    description = "HPI_DataProcessingMovShiftr"
    mask, match = a32_opcode("xxxx_0001_101x_xxxx__xxxx_xxxx_xxx1_xxxx")
    srcRegsRelativeLats = [3, 3, 2, 2, 2, 1, 0]


class HPI_DataProcessingMayShift(MinorFUTiming):
    description = "HPI_DataProcessingMayShift"
    mask, match = a32_opcode("xxxx_000x_xxxx_xxxx__xxxx_xxxx_xxxx_xxxx")
    srcRegsRelativeLats = [3, 3, 2, 2, 1, 1, 0]


class HPI_DataProcessingNoShift(MinorFUTiming):
    description = "HPI_DataProcessingNoShift"
    mask, match = a32_opcode("xxxx_000x_xxxx_xxxx__xxxx_0000_0xx0_xxxx")
    srcRegsRelativeLats = [3, 3, 2, 2, 2, 1, 0]


class HPI_DataProcessingAllowShifti(MinorFUTiming):
    description = "HPI_DataProcessingAllowShifti"
    mask, match = a32_opcode("xxxx_000x_xxxx_xxxx__xxxx_xxxx_xxx0_xxxx")
    srcRegsRelativeLats = [3, 3, 2, 2, 1, 1, 0]


class HPI_DataProcessingSuppressShift(MinorFUTiming):
    description = "HPI_DataProcessingSuppressShift"
    mask, match = a32_opcode("xxxx_000x_xxxx_xxxx__xxxx_xxxx_xxxx_xxxx")
    srcRegsRelativeLats = []
    suppress = True


class HPI_DataProcessingSuppressBranch(MinorFUTiming):
    description = "HPI_DataProcessingSuppressBranch"
    mask, match = a32_opcode("xxxx_1010_xxxx_xxxx__xxxx_xxxx_xxxx_xxxx")
    srcRegsRelativeLats = []
    suppress = True


class HPI_BFI_T1(MinorFUTiming):
    description = "HPI_BFI_T1"
    mask, match = t32_opcode("1111_0x11_0110_xxxx__0xxx_xxxx_xxxx_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 1, 1, 0]


class HPI_BFI_A1(MinorFUTiming):
    description = "HPI_BFI_A1"
    mask, match = a32_opcode("xxxx_0111_110x_xxxx__xxxx_xxxx_x001_xxxx")
    # f, f, f, dest, src
    srcRegsRelativeLats = [0, 0, 0, 1, 1, 0]


class HPI_CLZ_T1(MinorFUTiming):
    description = "HPI_CLZ_T1"
    mask, match = t32_opcode("1111_1010_1011_xxxx__1111_xxxx_1000_xxxx")
    srcRegsRelativeLats = [3, 3, 2, 2, 2, 1, 0]


class HPI_CLZ_A1(MinorFUTiming):
    description = "HPI_CLZ_A1"
    mask, match = a32_opcode("xxxx_0001_0110_xxxx__xxxx_xxxx_0001_xxxx")
    srcRegsRelativeLats = [3, 3, 2, 2, 2, 1, 0]


class HPI_CMN_immediate_A1(MinorFUTiming):
    description = "HPI_CMN_immediate_A1"
    mask, match = a32_opcode("xxxx_0011_0111_xxxx__xxxx_xxxx_xxxx_xxxx")
    srcRegsRelativeLats = [3, 3, 3, 2, 2, 3, 3, 3, 0]


class HPI_CMN_register_A1(MinorFUTiming):
    description = "HPI_CMN_register_A1"
    mask, match = a32_opcode("xxxx_0001_0111_xxxx__xxxx_xxxx_xxx0_xxxx")
    srcRegsRelativeLats = [3, 3, 3, 2, 2, 3, 3, 3, 0]


class HPI_CMP_immediate_A1(MinorFUTiming):
    description = "HPI_CMP_immediate_A1"
    mask, match = a32_opcode("xxxx_0011_0101_xxxx__xxxx_xxxx_xxxx_xxxx")
    srcRegsRelativeLats = [3, 3, 3, 2, 2, 3, 3, 3, 0]


class HPI_CMP_register_A1(MinorFUTiming):
    description = "HPI_CMP_register_A1"
    mask, match = a32_opcode("xxxx_0001_0101_xxxx__xxxx_xxxx_xxx0_xxxx")
    srcRegsRelativeLats = [3, 3, 3, 2, 2, 3, 3, 3, 0]


class HPI_MLA_T1(MinorFUTiming):
    description = "HPI_MLA_T1"
    mask, match = t32_opcode("1111_1011_0000_xxxx__xxxx_xxxx_0000_xxxx")
    # z, z, z, a, l?, r?
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 2, 0]


class HPI_MLA_A1(MinorFUTiming):
    description = "HPI_MLA_A1"
    mask, match = a32_opcode("xxxx_0000_001x_xxxx__xxxx_xxxx_1001_xxxx")
    # z, z, z, a, l?, r?
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 2, 0]


class HPI_MADD_A64(MinorFUTiming):
    description = "HPI_MADD_A64"
    mask, match = a64_opcode("x001_1011_000x_xxxx__0xxx_xxxx_xxxx_xxxx")
    # a, l?, r?
    srcRegsRelativeLats = [1, 1, 1, 0]
    extraCommitLat = 1


class HPI_MLS_T1(MinorFUTiming):
    description = "HPI_MLS_T1"
    mask, match = t32_opcode("1111_1011_0000_xxxx__xxxx_xxxx_0001_xxxx")
    # z, z, z, l?, a, r?
    srcRegsRelativeLats = [0, 0, 0, 2, 0, 0, 0]


class HPI_MLS_A1(MinorFUTiming):
    description = "HPI_MLS_A1"
    mask, match = a32_opcode("xxxx_0000_0110_xxxx__xxxx_xxxx_1001_xxxx")
    # z, z, z, l?, a, r?
    srcRegsRelativeLats = [0, 0, 0, 2, 0, 0, 0]


class HPI_MOVT_A1(MinorFUTiming):
    description = "HPI_MOVT_A1"
    mask, match = t32_opcode("xxxx_0010_0100_xxxx__xxxx_xxxx_xxxx_xxxx")


class HPI_MUL_T1(MinorFUTiming):
    description = "HPI_MUL_T1"
    mask, match = t16_opcode("0100_0011_01xx_xxxx")


class HPI_MUL_T2(MinorFUTiming):
    description = "HPI_MUL_T2"
    mask, match = t32_opcode("1111_1011_0000_xxxx_1111_xxxx_0000_xxxx")


class HPI_PKH_T1(MinorFUTiming):
    description = "HPI_PKH_T1"
    mask, match = t32_opcode("1110_1010_110x_xxxx__xxxx_xxxx_xxxx_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 2, 1, 0]


class HPI_PKH_A1(MinorFUTiming):
    description = "HPI_PKH_A1"
    mask, match = a32_opcode("xxxx_0110_1000_xxxx__xxxx_xxxx_xx01_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 2, 1, 0]


class HPI_QADD_QSUB_T1(MinorFUTiming):
    description = "HPI_QADD_QSUB_T1"
    mask, match = t32_opcode("1111_1010_1000_xxxx__1111_xxxx_10x0_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 1, 1, 0]


class HPI_QADD_QSUB_A1(MinorFUTiming):
    description = "HPI_QADD_QSUB_A1"
    mask, match = a32_opcode("xxxx_0001_00x0_xxxx__xxxx_xxxx_0101_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 1, 1, 0]


# T1 QADD16 QADD8 QSUB16 QSUB8 UQADD16 UQADD8 UQSUB16 UQSUB8
class HPI_QADD_ETC_T1(MinorFUTiming):
    description = "HPI_QADD_ETC_T1"
    mask, match = t32_opcode("1111_1010_1x0x_xxxx__1111_xxxx_0x01_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 1, 1, 0]


# A1 QADD16 QADD8 QSAX QSUB16 QSUB8 UQADD16 UQADD8 UQASX UQSAX UQSUB16 UQSUB8
class HPI_QADD_ETC_A1(MinorFUTiming):
    description = "HPI_QADD_ETC_A1"
    mask, match = a32_opcode("xxxx_0110_0x10_xxxx__xxxx_xxxx_xxx1_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 1, 1, 0]


class HPI_QASX_QSAX_UQASX_UQSAX_T1(MinorFUTiming):
    description = "HPI_QASX_QSAX_UQASX_UQSAX_T1"
    mask, match = t32_opcode("1111_1010_1x10_xxxx__1111_xxxx_0x01_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 1, 1, 0]


class HPI_QDADD_QDSUB_T1(MinorFUTiming):
    description = "HPI_QDADD_QDSUB_T1"
    mask, match = t32_opcode("1111_1010_1000_xxxx__1111_xxxx_10x1_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 1, 0]


class HPI_QDADD_QDSUB_A1(MinorFUTiming):
    description = "HPI_QDADD_QSUB_A1"
    mask, match = a32_opcode("xxxx_0001_01x0_xxxx__xxxx_xxxx_0101_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 1, 0]


class HPI_RBIT_A1(MinorFUTiming):
    description = "HPI_RBIT_A1"
    mask, match = a32_opcode("xxxx_0110_1111_xxxx__xxxx_xxxx_0011_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 1, 0]


class HPI_REV_REV16_A1(MinorFUTiming):
    description = "HPI_REV_REV16_A1"
    mask, match = a32_opcode("xxxx_0110_1011_xxxx__xxxx_xxxx_x011_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 1, 0]


class HPI_REVSH_A1(MinorFUTiming):
    description = "HPI_REVSH_A1"
    mask, match = a32_opcode("xxxx_0110_1111_xxxx__xxxx_xxxx_1011_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 1, 0]


class HPI_ADD_ETC_A1(MinorFUTiming):
    description = "HPI_ADD_ETC_A1"
    mask, match = a32_opcode("xxxx_0110_0xx1_xxxx__xxxx_xxxx_x001_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 2, 2, 0]


class HPI_ADD_ETC_T1(MinorFUTiming):
    description = "HPI_ADD_ETC_A1"
    mask, match = t32_opcode("1111_1010_100x_xxxx__1111_xxxx_0xx0_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 2, 2, 0]


class HPI_SASX_SHASX_UASX_UHASX_A1(MinorFUTiming):
    description = "HPI_SASX_SHASX_UASX_UHASX_A1"
    mask, match = a32_opcode("xxxx_0110_0xx1_xxxx__xxxx_xxxx_0011_xxxx")
    srcRegsRelativeLats = [3, 3, 2, 2, 2, 1, 0]


class HPI_SBFX_UBFX_A1(MinorFUTiming):
    description = "HPI_SBFX_UBFX_A1"
    mask, match = a32_opcode("xxxx_0111_1x1x_xxxx__xxxx_xxxx_x101_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 1, 0]


### SDIV

sdiv_lat_expr = expr_top(
    let(
        [
            ("left", un("SignExtend32To64", src_reg(4))),
            ("right", un("SignExtend32To64", src_reg(3))),
            (
                "either_signed",
                bin(
                    "Or",
                    bin("SLessThan", ref("left"), literal(0)),
                    bin("SLessThan", ref("right"), literal(0)),
                ),
            ),
            ("left_size", un("SizeInBits", un("Abs", ref("left")))),
            (
                "signed_adjust",
                if_expr(ref("either_signed"), literal(1), literal(0)),
            ),
            (
                "right_size",
                un(
                    "SizeInBits",
                    bin(
                        "UDiv",
                        un("Abs", ref("right")),
                        if_expr(ref("either_signed"), literal(4), literal(2)),
                    ),
                ),
            ),
            (
                "left_minus_right",
                if_expr(
                    bin("SLessThan", ref("left_size"), ref("right_size")),
                    literal(0),
                    bin("Sub", ref("left_size"), ref("right_size")),
                ),
            ),
        ],
        bin(
            "Add",
            ref("signed_adjust"),
            if_expr(
                bin("Equal", ref("right"), literal(0)),
                literal(0),
                bin("UDiv", ref("left_minus_right"), literal(4)),
            ),
        ),
    )
)

sdiv_lat_expr64 = expr_top(
    let(
        [
            ("left", un("SignExtend32To64", src_reg(0))),
            ("right", un("SignExtend32To64", src_reg(1))),
            (
                "either_signed",
                bin(
                    "Or",
                    bin("SLessThan", ref("left"), literal(0)),
                    bin("SLessThan", ref("right"), literal(0)),
                ),
            ),
            ("left_size", un("SizeInBits", un("Abs", ref("left")))),
            (
                "signed_adjust",
                if_expr(ref("either_signed"), literal(1), literal(0)),
            ),
            (
                "right_size",
                un(
                    "SizeInBits",
                    bin(
                        "UDiv",
                        un("Abs", ref("right")),
                        if_expr(ref("either_signed"), literal(4), literal(2)),
                    ),
                ),
            ),
            (
                "left_minus_right",
                if_expr(
                    bin("SLessThan", ref("left_size"), ref("right_size")),
                    literal(0),
                    bin("Sub", ref("left_size"), ref("right_size")),
                ),
            ),
        ],
        bin(
            "Add",
            ref("signed_adjust"),
            if_expr(
                bin("Equal", ref("right"), literal(0)),
                literal(0),
                bin("UDiv", ref("left_minus_right"), literal(4)),
            ),
        ),
    )
)


class HPI_SDIV_A1(MinorFUTiming):
    description = "HPI_SDIV_A1"
    mask, match = a32_opcode("xxxx_0111_0001_xxxx__xxxx_xxxx_0001_xxxx")
    extraCommitLat = 0
    srcRegsRelativeLats = []
    extraCommitLatExpr = sdiv_lat_expr


class HPI_SDIV_A64(MinorFUTiming):
    description = "HPI_SDIV_A64"
    mask, match = a64_opcode("x001_1010_110x_xxxx__0000_11xx_xxxx_xxxx")
    extraCommitLat = 0
    srcRegsRelativeLats = []
    extraCommitLatExpr = sdiv_lat_expr64


### SEL


class HPI_SEL_A1(MinorFUTiming):
    description = "HPI_SEL_A1"
    mask, match = a32_opcode("xxxx_0110_1000_xxxx__xxxx_xxxx_1011_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 2, 2, 0]


class HPI_SEL_A1_Suppress(MinorFUTiming):
    description = "HPI_SEL_A1_Suppress"
    mask, match = a32_opcode("xxxx_0110_1000_xxxx__xxxx_xxxx_1011_xxxx")
    srcRegsRelativeLats = []
    suppress = True


class HPI_SHSAX_SSAX_UHSAX_USAX_A1(MinorFUTiming):
    description = "HPI_SHSAX_SSAX_UHSAX_USAX_A1"
    mask, match = a32_opcode("xxxx_0110_0xx1_xxxx__xxxx_xxxx_0101_xxxx")
    # As Default
    srcRegsRelativeLats = [3, 3, 2, 2, 2, 1, 0]


class HPI_USUB_ETC_A1(MinorFUTiming):
    description = "HPI_USUB_ETC_A1"
    mask, match = a32_opcode("xxxx_0110_0xx1_xxxx__xxxx_xxxx_x111_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 2, 2, 0]


class HPI_SMLABB_T1(MinorFUTiming):
    description = "HPI_SMLABB_T1"
    mask, match = t32_opcode("1111_1011_0001_xxxx__xxxx_xxxx_00xx_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 2, 0]


class HPI_SMLABB_A1(MinorFUTiming):
    description = "HPI_SMLABB_A1"
    mask, match = a32_opcode("xxxx_0001_0000_xxxx__xxxx_xxxx_1xx0_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 2, 0]


class HPI_SMLAD_T1(MinorFUTiming):
    description = "HPI_SMLAD_T1"
    mask, match = t32_opcode("1111_1011_0010_xxxx__xxxx_xxxx_000x_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 2, 0]


class HPI_SMLAD_A1(MinorFUTiming):
    description = "HPI_SMLAD_A1"
    mask, match = a32_opcode("xxxx_0111_0000_xxxx__xxxx_xxxx_00x1_xxxx")
    # z, z, z, l, r, a
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 2, 0]


class HPI_SMLAL_T1(MinorFUTiming):
    description = "HPI_SMLAL_T1"
    mask, match = t32_opcode("1111_1011_1100_xxxx__xxxx_xxxx_0000_xxxx")


class HPI_SMLAL_A1(MinorFUTiming):
    description = "HPI_SMLAL_A1"
    mask, match = a32_opcode("xxxx_0000_111x_xxxx__xxxx_xxxx_1001_xxxx")


class HPI_SMLALBB_T1(MinorFUTiming):
    description = "HPI_SMLALBB_T1"
    mask, match = t32_opcode("1111_1011_1100_xxxx__xxxx_xxxx_10xx_xxxx")


class HPI_SMLALBB_A1(MinorFUTiming):
    description = "HPI_SMLALBB_A1"
    mask, match = a32_opcode("xxxx_0001_0100_xxxx__xxxx_xxxx_1xx0_xxxx")


class HPI_SMLALD_T1(MinorFUTiming):
    description = "HPI_SMLALD_T1"
    mask, match = t32_opcode("1111_1011_1100_xxxx__xxxx_xxxx_110x_xxxx")


class HPI_SMLALD_A1(MinorFUTiming):
    description = "HPI_SMLALD_A1"
    mask, match = a32_opcode("xxxx_0111_0100_xxxx__xxxx_xxxx_00x1_xxxx")


class HPI_SMLAWB_T1(MinorFUTiming):
    description = "HPI_SMLAWB_T1"
    mask, match = t32_opcode("1111_1011_0011_xxxx__xxxx_xxxx_000x_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 2, 0]


class HPI_SMLAWB_A1(MinorFUTiming):
    description = "HPI_SMLAWB_A1"
    mask, match = a32_opcode("xxxx_0001_0010_xxxx__xxxx_xxxx_1x00_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 2, 0]


class HPI_SMLSD_A1(MinorFUTiming):
    description = "HPI_SMLSD_A1"
    mask, match = a32_opcode("xxxx_0111_0000_xxxx__xxxx_xxxx_01x1_xxxx")


class HPI_SMLSLD_T1(MinorFUTiming):
    description = "HPI_SMLSLD_T1"
    mask, match = t32_opcode("1111_1011_1101_xxxx__xxxx_xxxx_110x_xxxx")


class HPI_SMLSLD_A1(MinorFUTiming):
    description = "HPI_SMLSLD_A1"
    mask, match = a32_opcode("xxxx_0111_0100_xxxx__xxxx_xxxx_01x1_xxxx")


class HPI_SMMLA_T1(MinorFUTiming):
    description = "HPI_SMMLA_T1"
    mask, match = t32_opcode("1111_1011_0101_xxxx__xxxx_xxxx_000x_xxxx")
    #                                              ^^^^ != 1111
    srcRegsRelativeLats = [0, 0, 0, 2, 0, 0, 0]


class HPI_SMMLA_A1(MinorFUTiming):
    description = "HPI_SMMLA_A1"
    # Note that this must be after the encoding for SMMUL
    mask, match = a32_opcode("xxxx_0111_0101_xxxx__xxxx_xxxx_00x1_xxxx")
    #                                              ^^^^ != 1111
    srcRegsRelativeLats = [0, 0, 0, 2, 0, 0, 0]


class HPI_SMMLS_T1(MinorFUTiming):
    description = "HPI_SMMLS_T1"
    mask, match = t32_opcode("1111_1011_0110_xxxx__xxxx_xxxx_000x_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 2, 0, 0, 0]


class HPI_SMMLS_A1(MinorFUTiming):
    description = "HPI_SMMLS_A1"
    mask, match = a32_opcode("xxxx_0111_0101_xxxx__xxxx_xxxx_11x1_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 2, 0, 0, 0]


class HPI_SMMUL_T1(MinorFUTiming):
    description = "HPI_SMMUL_T1"
    mask, match = t32_opcode("1111_1011_0101_xxxx__1111_xxxx_000x_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0]


class HPI_SMMUL_A1(MinorFUTiming):
    description = "HPI_SMMUL_A1"
    mask, match = a32_opcode("xxxx_0111_0101_xxxx__1111_xxxx_00x1_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0]


class HPI_SMUAD_T1(MinorFUTiming):
    description = "HPI_SMUAD_T1"
    mask, match = t32_opcode("1111_1011_0010_xxxx__1111_xxxx_000x_xxxx")


class HPI_SMUAD_A1(MinorFUTiming):
    description = "HPI_SMUAD_A1"
    mask, match = a32_opcode("xxxx_0111_0000_xxxx__1111_xxxx_00x1_xxxx")


class HPI_SMULBB_T1(MinorFUTiming):
    description = "HPI_SMULBB_T1"
    mask, match = t32_opcode("1111_1011_0001_xxxx__1111_xxxx_00xx_xxxx")


class HPI_SMULBB_A1(MinorFUTiming):
    description = "HPI_SMULBB_A1"
    mask, match = a32_opcode("xxxx_0001_0110_xxxx__xxxx_xxxx_1xx0_xxxx")


class HPI_SMULL_T1(MinorFUTiming):
    description = "HPI_SMULL_T1"
    mask, match = t32_opcode("1111_1011_1000_xxxx__xxxx_xxxx_0000_xxxx")


class HPI_SMULL_A1(MinorFUTiming):
    description = "HPI_SMULL_A1"
    mask, match = a32_opcode("xxxx_0000_110x_xxxx__xxxx_xxxx_1001_xxxx")


class HPI_SMULWB_T1(MinorFUTiming):
    description = "HPI_SMULWB_T1"
    mask, match = t32_opcode("1111_1011_0011_xxxx__1111_xxxx_000x_xxxx")


class HPI_SMULWB_A1(MinorFUTiming):
    description = "HPI_SMULWB_A1"
    mask, match = a32_opcode("xxxx_0001_0010_xxxx__xxxx_xxxx_1x10_xxxx")


class HPI_SMUSD_T1(MinorFUTiming):
    description = "HPI_SMUSD_T1"
    mask, match = t32_opcode("1111_1011_0100_xxxx__1111_xxxx_000x_xxxx")


class HPI_SMUSD_A1(MinorFUTiming):
    description = "HPI_SMUSD_A1"
    mask, match = a32_opcode("xxxx_0111_0000_xxxx__1111_xxxx_01x1_xxxx")


class HPI_SSAT_USAT_no_shift_A1(MinorFUTiming):
    description = "HPI_SSAT_USAT_no_shift_A1"
    # Order *before* shift
    mask, match = a32_opcode("xxxx_0110_1x1x_xxxx__xxxx_0000_0001_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 2, 0]


class HPI_SSAT_USAT_shift_A1(MinorFUTiming):
    description = "HPI_SSAT_USAT_shift_A1"
    # Order after shift
    mask, match = a32_opcode("xxxx_0110_1x1x_xxxx__xxxx_xxxx_xx01_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 1, 0]


class HPI_SSAT16_USAT16_A1(MinorFUTiming):
    description = "HPI_SSAT16_USAT16_A1"
    mask, match = a32_opcode("xxxx_0110_1x10_xxxx__xxxx_xxxx_0011_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 2, 0]


class HPI_SXTAB_T1(MinorFUTiming):
    description = "HPI_SXTAB_T1"
    mask, match = t32_opcode("1111_1010_0100_xxxx__1111_xxxx_1xxx_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 1, 2, 0]


class HPI_SXTAB_SXTAB16_SXTAH_UXTAB_UXTAB16_UXTAH_A1(MinorFUTiming):
    description = "HPI_SXTAB_SXTAB16_SXTAH_UXTAB_UXTAB16_UXTAH_A1"
    # Place AFTER HPI_SXTB_SXTB16_SXTH_UXTB_UXTB16_UXTH_A1
    # e6[9d][^f]0070 are undefined
    mask, match = a32_opcode("xxxx_0110_1xxx_xxxx__xxxx_xxxx_0111_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 1, 2, 0]


class HPI_SXTAB16_T1(MinorFUTiming):
    description = "HPI_SXTAB16_T1"
    mask, match = t32_opcode("1111_1010_0010_xxxx__1111_xxxx_1xxx_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 1, 2, 0]


class HPI_SXTAH_T1(MinorFUTiming):
    description = "HPI_SXTAH_T1"
    mask, match = t32_opcode("1111_1010_0000_xxxx__1111_xxxx_1xxx_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 1, 2, 0]


class HPI_SXTB_T1(MinorFUTiming):
    description = "HPI_SXTB_T1"
    mask, match = t16_opcode("1011_0010_01xx_xxxx")


class HPI_SXTB_T2(MinorFUTiming):
    description = "HPI_SXTB_T2"
    mask, match = t32_opcode("1111_1010_0100_1111__1111_xxxx_1xxx_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 1, 2, 0]


class HPI_SXTB_SXTB16_SXTH_UXTB_UXTB16_UXTH_A1(MinorFUTiming):
    description = "HPI_SXTB_SXTB16_SXTH_UXTB_UXTB16_UXTH_A1"
    # e6[9d]f0070 are undefined
    mask, match = a32_opcode("xxxx_0110_1xxx_1111__xxxx_xxxx_0111_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 2, 0]


class HPI_SXTB16_T1(MinorFUTiming):
    description = "HPI_SXTB16_T1"
    mask, match = t32_opcode("1111_1010_0010_1111__1111_xxxx_1xxx_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 1, 2, 0]


class HPI_SXTH_T1(MinorFUTiming):
    description = "HPI_SXTH_T1"
    mask, match = t16_opcode("1011_0010_00xx_xxxx")


class HPI_SXTH_T2(MinorFUTiming):
    description = "HPI_SXTH_T2"
    mask, match = t32_opcode("1111_1010_0000_1111__1111_xxxx_1xxx_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 1, 2, 0]


class HPI_UDIV_T1(MinorFUTiming):
    description = "HPI_UDIV_T1"
    mask, match = t32_opcode("1111_1011_1011_xxxx__xxxx_xxxx_1111_xxxx")


udiv_lat_expr = expr_top(
    let(
        [
            ("left", src_reg(4)),
            ("right", src_reg(3)),
            ("left_size", un("SizeInBits", ref("left"))),
            (
                "right_size",
                un("SizeInBits", bin("UDiv", ref("right"), literal(2))),
            ),
            (
                "left_minus_right",
                if_expr(
                    bin("SLessThan", ref("left_size"), ref("right_size")),
                    literal(0),
                    bin("Sub", ref("left_size"), ref("right_size")),
                ),
            ),
        ],
        if_expr(
            bin("Equal", ref("right"), literal(0)),
            literal(0),
            bin("UDiv", ref("left_minus_right"), literal(4)),
        ),
    )
)


class HPI_UDIV_A1(MinorFUTiming):
    description = "HPI_UDIV_A1"
    mask, match = a32_opcode("xxxx_0111_0011_xxxx__xxxx_xxxx_0001_xxxx")
    extraCommitLat = 0
    srcRegsRelativeLats = []
    extraCommitLatExpr = udiv_lat_expr


class HPI_UMAAL_T1(MinorFUTiming):
    description = "HPI_UMAAL_T1"
    mask, match = t32_opcode("1111_1011_1110_xxxx__xxxx_xxxx_0110_xxxx")
    # z, z, z, dlo, dhi, l, r
    extraCommitLat = 1
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 0, 0]


class HPI_UMAAL_A1(MinorFUTiming):
    description = "HPI_UMAAL_A1"
    mask, match = a32_opcode("xxxx_0000_0100_xxxx__xxxx_xxxx_1001_xxxx")
    # z, z, z, dlo, dhi, l, r
    extraCommitLat = 1
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 0, 0]


class HPI_UMLAL_T1(MinorFUTiming):
    description = "HPI_UMLAL_T1"
    mask, match = t32_opcode("1111_1011_1110_xxxx__xxxx_xxxx_0000_xxxx")


class HPI_UMLAL_A1(MinorFUTiming):
    description = "HPI_UMLAL_A1"
    mask, match = t32_opcode("xxxx_0000_101x_xxxx__xxxx_xxxx_1001_xxxx")


class HPI_UMULL_T1(MinorFUTiming):
    description = "HPI_UMULL_T1"
    mask, match = t32_opcode("1111_1011_1010_xxxx__xxxx_xxxx_0000_xxxx")


class HPI_UMULL_A1(MinorFUTiming):
    description = "HPI_UMULL_A1"
    mask, match = a32_opcode("xxxx_0000_100x_xxxx__xxxx_xxxx_1001_xxxx")


class HPI_USAD8_USADA8_A1(MinorFUTiming):
    description = "HPI_USAD8_USADA8_A1"
    mask, match = a32_opcode("xxxx_0111_1000_xxxx__xxxx_xxxx_0001_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 2, 0]


class HPI_USAD8_USADA8_A1_Suppress(MinorFUTiming):
    description = "HPI_USAD8_USADA8_A1_Suppress"
    mask, match = a32_opcode("xxxx_0111_1000_xxxx__xxxx_xxxx_0001_xxxx")
    srcRegsRelativeLats = []
    suppress = True


class HPI_VMOV_immediate_A1(MinorFUTiming):
    description = "HPI_VMOV_register_A1"
    mask, match = a32_opcode("1111_0010_0x10_xxxx_xxxx_0001_xxx1_xxxx")
    # cpsr, z, z, z, hcptr, nsacr, cpacr, fpexc, scr
    srcRegsRelativeLats = [5, 5, 5, 5, 5, 5, 5, 5, 5, 0]


class HPI_VMRS_A1(MinorFUTiming):
    description = "HPI_VMRS_A1"
    mask, match = a32_opcode("xxxx_1110_1111_0001_xxxx_1010_xxx1_xxxx")
    # cpsr,z,z,z,hcptr,nsacr,cpacr,scr,r42
    srcRegsRelativeLats = [5, 5, 5, 5, 5, 5, 5, 5, 5, 0]


class HPI_VMOV_register_A2(MinorFUTiming):
    description = "HPI_VMOV_register_A2"
    mask, match = a32_opcode("xxxx_1110_1x11_0000_xxxx_101x_01x0_xxxx")
    # cpsr, z, r39, z, hcptr, nsacr, cpacr, fpexc, scr, f4, f5, f0, f1
    srcRegsRelativeLats = [
        5,
        5,
        5,
        5,
        5,
        5,
        5,
        5,
        5,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        0,
    ]


# VADD.I16 D/VADD.F32 D/VADD.I8 D/VADD.I32 D
class HPI_VADD2H_A32(MinorFUTiming):
    description = "Vadd2hALU"
    mask, match = a32_opcode("1111_0010_0xxx_xxxx__xxxx_1000_xxx0_xxxx")
    # cpsr, z, z, z, cpacr, fpexc, l0, r0, l1, r1, l2, r2, l3, r3 (for vadd2h)
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 4, 0]


# VAQQHN.I16 Q/VAQQHN.I32 Q/VAQQHN.I64 Q
class HPI_VADDHN_A32(MinorFUTiming):
    description = "VaddhnALU"
    mask, match = a32_opcode("1111_0010_1xxx_xxxx__xxxx_0100_x0x0_xxxx")
    # cpsr, z, z, z, cpacr, fpexc, l0, l1, l2, l3, r0, r1, r2, r3
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3, 3, 3, 3, 0]


class HPI_VADDL_A32(MinorFUTiming):
    description = "VaddlALU"
    mask, match = a32_opcode("1111_001x_1xxx_xxxx__xxxx_0000_x0x0_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3, 3, 3, 3, 0]


class HPI_VADDW_A32(MinorFUTiming):
    description = "HPI_VADDW_A32"
    mask, match = a32_opcode("1111_001x_1xxx_xxxx__xxxx_0001_x0x0_xxxx")
    # cpsr, z, z, z, cpacr, fpexc, l0, l1, l2, l3, r0, r1
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 3, 3, 0]


# VHADD/VHSUB S8,S16,S32,U8,U16,U32 Q and D
class HPI_VHADD_A32(MinorFUTiming):
    description = "HPI_VHADD_A32"
    mask, match = a32_opcode("1111_001x_0xxx_xxxx__xxxx_00x0_xxx0_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 4, 0]


class HPI_VPADAL_A32(MinorFUTiming):
    description = "VpadalALU"
    mask, match = a32_opcode("1111_0011_1x11_xx00__xxxx_0110_xxx0_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 0]


# VPADDH.I16
class HPI_VPADDH_A32(MinorFUTiming):
    description = "VpaddhALU"
    mask, match = a32_opcode("1111_0010_0xxx_xxxx__xxxx_1011_xxx1_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3, 3, 3, 3, 0]


# VPADDH.F32
class HPI_VPADDS_A32(MinorFUTiming):
    description = "VpaddsALU"
    mask, match = a32_opcode("1111_0011_0x0x_xxxx__xxxx_1101_xxx0_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 0]


# VPADDL.S16
class HPI_VPADDL_A32(MinorFUTiming):
    description = "VpaddlALU"
    mask, match = a32_opcode("1111_0011_1x11_xx00__xxxx_0010_xxx0_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3, 3, 3, 3, 0]


# VRADDHN.I16
class HPI_VRADDHN_A32(MinorFUTiming):
    description = "HPI_VRADDHN_A32"
    mask, match = a32_opcode("1111_0011_1xxx_xxxx__xxxx_0100_x0x0_xxxx")
    # cpsr, z, z, z, cpacr, fpexc, l0, l1, l2, l3, r0, r1, r2, r3
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 4, 0]


class HPI_VRHADD_A32(MinorFUTiming):
    description = "VrhaddALU"
    mask, match = a32_opcode("1111_001x_0xxx_xxxx__xxxx_0001_xxx0_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 4, 0]


class HPI_VQADD_A32(MinorFUTiming):
    description = "VqaddALU"
    mask, match = a32_opcode("1111_001x_0xxx_xxxx__xxxx_0000_xxx1_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3, 3, 3, 3, 0]


class HPI_VANDQ_A32(MinorFUTiming):
    description = "VandqALU"
    mask, match = a32_opcode("1111_0010_0x00_xxxx__xxxx_0001_xxx1_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 5, 5, 5, 5, 5, 5, 5, 5, 0]


# VMUL (integer)
class HPI_VMULI_A32(MinorFUTiming):
    description = "VmuliALU"
    mask, match = a32_opcode("1111_001x_0xxx_xxxx__xxxx_1001_xxx1_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 0]


# VBIC (reg)
class HPI_VBIC_A32(MinorFUTiming):
    description = "VbicALU"
    mask, match = a32_opcode("1111_0010_0x01_xxxx__xxxx_0001_xxx1_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 5, 5, 5, 5, 5, 5, 5, 5, 0]


# VBIF VBIT VBSL
class HPI_VBIF_ETC_A32(MinorFUTiming):
    description = "VbifALU"
    mask, match = a32_opcode("1111_0011_0xxx_xxxx__xxxx_0001_xxx1_xxxx")
    srcRegsRelativeLats = [
        0,
        0,
        0,
        0,
        0,
        0,
        5,
        5,
        5,
        5,
        5,
        5,
        5,
        5,
        5,
        5,
        5,
        5,
        0,
    ]


class HPI_VACGE_A32(MinorFUTiming):
    description = "VacgeALU"
    mask, match = a32_opcode("1111_0011_0xxx_xxxx__xxxx_1110_xxx1_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 0]


# VCEQ.F32
class HPI_VCEQ_A32(MinorFUTiming):
    description = "VceqALU"
    mask, match = a32_opcode("1111_0010_0x0x_xxxx__xxxx_1110_xxx0_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 0]


# VCEQ.[IS]... register
class HPI_VCEQI_A32(MinorFUTiming):
    description = "VceqiALU"
    mask, match = a32_opcode("1111_0011_0xxx_xxxx__xxxx_1000_xxx1_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 0]


# VCEQ.[IS]... immediate
class HPI_VCEQII_A32(MinorFUTiming):
    description = "HPI_VCEQII_A32"
    mask, match = a32_opcode("1111_0011_1x11_xx01__xxxx_0x01_0xx0_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 0]


class HPI_VTST_A32(MinorFUTiming):
    description = "HPI_VTST_A32"
    mask, match = a32_opcode("1111_0010_0xxx_xxxx__xxxx_1000_xxx1_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3, 3, 3, 3, 3, 0]


class HPI_VCLZ_A32(MinorFUTiming):
    description = "HPI_VCLZ_A32"
    mask, match = a32_opcode("1111_0011_1x11_xx00__xxxx_0100_1xx0_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 0]


class HPI_VCNT_A32(MinorFUTiming):
    description = "HPI_VCNT_A32"
    mask, match = a32_opcode("1111_0011_1x11_xx00__xxxx_0101_0xx0_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 0]


class HPI_VEXT_A32(MinorFUTiming):
    description = "HPI_VCNT_A32"
    mask, match = a32_opcode("1111_0010_1x11_xxxx__xxxx_xxxx_xxx0_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 0]


# VMAX VMIN integer
class HPI_VMAXI_A32(MinorFUTiming):
    description = "HPI_VMAXI_A32"
    mask, match = a32_opcode("1111_001x_0xxx_xxxx__xxxx_0110_xxxx_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 0]


# VMAX VMIN float
class HPI_VMAXS_A32(MinorFUTiming):
    description = "HPI_VMAXS_A32"
    mask, match = a32_opcode("1111_0010_0xxx_xxxx__xxxx_1111_xxx0_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0]


# VNEG integer
class HPI_VNEGI_A32(MinorFUTiming):
    description = "HPI_VNEGI_A32"
    mask, match = a32_opcode("1111_0011_1x11_xx01__xxxx_0x11_1xx0_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 0]


# VNEG float
class HPI_VNEGF_A32(MinorFUTiming):
    description = "HPI_VNEGF_A32"
    mask, match = a32_opcode("xxxx_1110_1x11_0001__xxxx_101x_01x0_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0]


# VREV16 VREV32 VREV64
class HPI_VREVN_A32(MinorFUTiming):
    description = "HPI_VREVN_A32"
    mask, match = a32_opcode("1111_0011_1x11_xx00__xxxx_000x_xxx0_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 0]


class HPI_VQNEG_A32(MinorFUTiming):
    description = "HPI_VQNEG_A32"
    mask, match = a32_opcode("1111_0011_1x11_xx00__xxxx_0111_1xx0_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3, 3, 3, 3, 3, 0]


class HPI_VSWP_A32(MinorFUTiming):
    description = "HPI_VSWP_A32"
    mask, match = a32_opcode("1111_0011_1x11_xx10__xxxx_0000_0xx0_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 4, 0]


class HPI_VTRN_A32(MinorFUTiming):
    description = "HPI_VTRN_A32"
    mask, match = a32_opcode("1111_0011_1x11_xx10__xxxx_0000_1xx0_xxxx")
    # cpsr, z, z, z, cpact, fpexc, o0, d0, o1, d1, o2, d2, o3, d3
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 0]


# VQMOVN VQMOVUN
class HPI_VQMOVN_A32(MinorFUTiming):
    description = "HPI_VQMOVN_A32"
    mask, match = a32_opcode("1111_0011_1x11_xx10__xxxx_0010_xxx0_xxxx")
    # cpsr, z, z, z, cpact, fpexc, o[0], o[1], o[2], o[3], fpscr
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 0]


# VUZP double word
class HPI_VUZP_A32(MinorFUTiming):
    description = "HPI_VUZP_A32"
    mask, match = a32_opcode("1111_0011_1x11_xx10__xxxx_0001_00x0_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3, 3, 3, 3, 0]


# VDIV.F32
class HPI_VDIV32_A32(MinorFUTiming):
    description = "HPI_VDIV32_A32"
    mask, match = a32_opcode("xxxx_1110_1x00_xxxx__xxxx_1010_x0x0_xxxx")
    # cpsr, z, z, z, cpact, fpexc, fpscr_exc, l, r
    extraCommitLat = 9
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 20, 4, 4, 0]


# VDIV.F64
class HPI_VDIV64_A32(MinorFUTiming):
    description = "HPI_VDIV64_A32"
    mask, match = a32_opcode("xxxx_1110_1x00_xxxx__xxxx_1011_x0x0_xxxx")
    # cpsr, z, z, z, cpact, fpexc, fpscr_exc, l, r
    extraCommitLat = 18
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 20, 4, 4, 0]


class HPI_VZIP_A32(MinorFUTiming):
    description = "HPI_VZIP_A32"
    mask, match = a32_opcode("1111_0011_1x11_xx10__xxxx_0001_1xx0_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 4, 0]


# VPMAX integer
class HPI_VPMAX_A32(MinorFUTiming):
    description = "HPI_VPMAX_A32"
    mask, match = a32_opcode("1111_001x_0xxx_xxxx__xxxx_1010_xxxx_xxxx")
    # cpsr, z, z, z, cpact, fpexc, l0, r0, l1, r1, fpscr
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 4, 0]


# VPMAX float
class HPI_VPMAXF_A32(MinorFUTiming):
    description = "HPI_VPMAXF_A32"
    mask, match = a32_opcode("1111_0011_0xxx_xxxx__xxxx_1111_xxx0_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 0]


class HPI_VMOVN_A32(MinorFUTiming):
    description = "HPI_VMOVN_A32"
    mask, match = a32_opcode("1111_0011_1x11_xx10__xxxx_0010_00x0_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 0]


class HPI_VMOVL_A32(MinorFUTiming):
    description = "HPI_VMOVL_A32"
    mask, match = a32_opcode("1111_001x_1xxx_x000__xxxx_1010_00x1_xxxx")
    srcRegsRelativeLats = [0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 0]


# VSQRT.F64
class HPI_VSQRT64_A32(MinorFUTiming):
    description = "HPI_VSQRT64_A32"
    mask, match = a32_opcode("xxxx_1110_1x11_0001__xxxx_1011_11x0_xxxx")
    extraCommitLat = 18
    srcRegsRelativeLats = []


# VSQRT.F32
class HPI_VSQRT32_A32(MinorFUTiming):
    description = "HPI_VSQRT32_A32"
    mask, match = a32_opcode("xxxx_1110_1x11_0001__xxxx_1010_11x0_xxxx")
    extraCommitLat = 9
    srcRegsRelativeLats = []


class HPI_FloatSimdFU(MinorFU):
    opClasses = minorMakeOpClassSet(
        [
            "FloatAdd",
            "FloatCmp",
            "FloatCvt",
            "FloatMult",
            "FloatDiv",
            "FloatSqrt",
            "FloatMisc",
            "FloatMultAcc",
            "SimdAdd",
            "SimdAddAcc",
            "SimdAlu",
            "SimdCmp",
            "SimdCvt",
            "SimdMisc",
            "SimdMult",
            "SimdMultAcc",
            "SimdMatMultAcc",
            "SimdShift",
            "SimdShiftAcc",
            "SimdSqrt",
            "SimdFloatAdd",
            "SimdFloatAlu",
            "SimdFloatCmp",
            "SimdFloatCvt",
            "SimdFloatDiv",
            "SimdFloatMisc",
            "SimdFloatMult",
            "SimdFloatMultAcc",
            "SimdFloatMatMultAcc",
            "SimdFloatSqrt",
        ]
    )

    timings = [
        # VUZP and VZIP must be before VADDW/L
        HPI_VUZP_A32(),
        HPI_VZIP_A32(),
        HPI_VADD2H_A32(),
        HPI_VADDHN_A32(),
        HPI_VADDL_A32(),
        HPI_VADDW_A32(),
        HPI_VHADD_A32(),
        HPI_VPADAL_A32(),
        HPI_VPADDH_A32(),
        HPI_VPADDS_A32(),
        HPI_VPADDL_A32(),
        HPI_VRADDHN_A32(),
        HPI_VRHADD_A32(),
        HPI_VQADD_A32(),
        HPI_VANDQ_A32(),
        HPI_VBIC_A32(),
        HPI_VBIF_ETC_A32(),
        HPI_VACGE_A32(),
        HPI_VCEQ_A32(),
        HPI_VCEQI_A32(),
        HPI_VCEQII_A32(),
        HPI_VTST_A32(),
        HPI_VCLZ_A32(),
        HPI_VCNT_A32(),
        HPI_VEXT_A32(),
        HPI_VMAXI_A32(),
        HPI_VMAXS_A32(),
        HPI_VNEGI_A32(),
        HPI_VNEGF_A32(),
        HPI_VREVN_A32(),
        HPI_VQNEG_A32(),
        HPI_VSWP_A32(),
        HPI_VTRN_A32(),
        HPI_VPMAX_A32(),
        HPI_VPMAXF_A32(),
        HPI_VMOVN_A32(),
        HPI_VMRS_A1(),
        HPI_VMOV_immediate_A1(),
        HPI_VMOV_register_A2(),
        HPI_VQMOVN_A32(),
        HPI_VMOVL_A32(),
        HPI_VDIV32_A32(),
        HPI_VDIV64_A32(),
        HPI_VSQRT32_A32(),
        HPI_VSQRT64_A32(),
        HPI_VMULI_A32(),
        # Add before here
        HPI_FMADD_A64(),
        HPI_FMSUB_D_A64(),
        HPI_FMOV_A64(),
        HPI_ADD_SUB_vector_scalar_A64(),
        HPI_ADD_SUB_vector_vector_A64(),
        HPI_FDIV_scalar_32_A64(),
        HPI_FDIV_scalar_64_A64(),
        HPI_DefaultA64Vfp(),
        HPI_DefaultVfp(),
    ]

    opLat = 6


class HPI_IntFU(MinorFU):
    opClasses = minorMakeOpClassSet(["IntAlu"])
    # IMPORTANT! Keep the order below, add new entries *at the head*
    timings = [
        HPI_SSAT_USAT_no_shift_A1(),
        HPI_SSAT_USAT_shift_A1(),
        HPI_SSAT16_USAT16_A1(),
        HPI_QADD_QSUB_A1(),
        HPI_QADD_QSUB_T1(),
        HPI_QADD_ETC_A1(),
        HPI_QASX_QSAX_UQASX_UQSAX_T1(),
        HPI_QADD_ETC_T1(),
        HPI_USUB_ETC_A1(),
        HPI_ADD_ETC_A1(),
        HPI_ADD_ETC_T1(),
        HPI_QDADD_QDSUB_A1(),
        HPI_QDADD_QDSUB_T1(),
        HPI_SASX_SHASX_UASX_UHASX_A1(),
        HPI_SHSAX_SSAX_UHSAX_USAX_A1(),
        HPI_SXTB_SXTB16_SXTH_UXTB_UXTB16_UXTH_A1(),
        # Must be after HPI_SXTB_SXTB16_SXTH_UXTB_UXTB16_UXTH_A1
        HPI_SXTAB_SXTAB16_SXTAH_UXTAB_UXTAB16_UXTAH_A1(),
        HPI_SXTAB_T1(),
        HPI_SXTAB16_T1(),
        HPI_SXTAH_T1(),
        HPI_SXTB_T2(),
        HPI_SXTB16_T1(),
        HPI_SXTH_T2(),
        HPI_PKH_A1(),
        HPI_PKH_T1(),
        HPI_SBFX_UBFX_A1(),
        HPI_SEL_A1(),
        HPI_RBIT_A1(),
        HPI_REV_REV16_A1(),
        HPI_REVSH_A1(),
        HPI_USAD8_USADA8_A1(),
        HPI_BFI_A1(),
        HPI_BFI_T1(),
        HPI_CMN_register_A1(),
        HPI_CMN_immediate_A1(),
        HPI_CMP_register_A1(),
        HPI_CMP_immediate_A1(),
        HPI_DataProcessingNoShift(),
        HPI_DataProcessingMovShiftr(),
        HPI_DataProcessingMayShift(),
        HPI_Cxxx_A64(),
        HPI_DefaultA64Int(),
        HPI_DefaultInt(),
    ]
    opLat = 3


class HPI_Int2FU(MinorFU):
    opClasses = minorMakeOpClassSet(["IntAlu"])
    # IMPORTANT! Keep the order below, add new entries *at the head*
    timings = [
        HPI_SSAT_USAT_no_shift_A1(),
        HPI_SSAT_USAT_shift_A1(),
        HPI_SSAT16_USAT16_A1(),
        HPI_QADD_QSUB_A1(),
        HPI_QADD_QSUB_T1(),
        HPI_QADD_ETC_A1(),
        HPI_QASX_QSAX_UQASX_UQSAX_T1(),
        HPI_QADD_ETC_T1(),
        HPI_USUB_ETC_A1(),
        HPI_ADD_ETC_A1(),
        HPI_ADD_ETC_T1(),
        HPI_QDADD_QDSUB_A1(),
        HPI_QDADD_QDSUB_T1(),
        HPI_SASX_SHASX_UASX_UHASX_A1(),
        HPI_SHSAX_SSAX_UHSAX_USAX_A1(),
        HPI_SXTB_SXTB16_SXTH_UXTB_UXTB16_UXTH_A1(),
        # Must be after HPI_SXTB_SXTB16_SXTH_UXTB_UXTB16_UXTH_A1
        HPI_SXTAB_SXTAB16_SXTAH_UXTAB_UXTAB16_UXTAH_A1(),
        HPI_SXTAB_T1(),
        HPI_SXTAB16_T1(),
        HPI_SXTAH_T1(),
        HPI_SXTB_T2(),
        HPI_SXTB16_T1(),
        HPI_SXTH_T2(),
        HPI_PKH_A1(),
        HPI_PKH_T1(),
        HPI_SBFX_UBFX_A1(),
        HPI_SEL_A1_Suppress(),
        HPI_RBIT_A1(),
        HPI_REV_REV16_A1(),
        HPI_REVSH_A1(),
        HPI_USAD8_USADA8_A1_Suppress(),
        HPI_BFI_A1(),
        HPI_BFI_T1(),
        HPI_CMN_register_A1(),  # Need to check for shift
        HPI_CMN_immediate_A1(),
        HPI_CMP_register_A1(),  # Need to check for shift
        HPI_CMP_immediate_A1(),
        HPI_DataProcessingNoShift(),
        HPI_DataProcessingAllowShifti(),
        # HPI_DataProcessingAllowMovShiftr(),
        # Data processing ops that match SuppressShift but are *not*
        # to be suppressed here
        HPI_CLZ_A1(),
        HPI_CLZ_T1(),
        HPI_DataProcessingSuppressShift(),
        # Can you dual issue a branch?
        # HPI_DataProcessingSuppressBranch(),
        HPI_Cxxx_A64(),
        HPI_DefaultA64Int(),
        HPI_DefaultInt(),
    ]
    opLat = 3


class HPI_IntMulFU(MinorFU):
    opClasses = minorMakeOpClassSet(["IntMult"])
    timings = [
        HPI_MLA_A1(),
        HPI_MLA_T1(),
        HPI_MLS_A1(),
        HPI_MLS_T1(),
        HPI_SMLABB_A1(),
        HPI_SMLABB_T1(),
        HPI_SMLAWB_A1(),
        HPI_SMLAWB_T1(),
        HPI_SMLAD_A1(),
        HPI_SMLAD_T1(),
        HPI_SMMUL_A1(),
        HPI_SMMUL_T1(),
        # SMMUL_A1 must be before SMMLA_A1
        HPI_SMMLA_A1(),
        HPI_SMMLA_T1(),
        HPI_SMMLS_A1(),
        HPI_SMMLS_T1(),
        HPI_UMAAL_A1(),
        HPI_UMAAL_T1(),
        HPI_MADD_A64(),
        HPI_DefaultA64Mul(),
        HPI_DefaultMul(),
    ]
    opLat = 3
    cantForwardFromFUIndices = [0, 1, 5]  # Int1, Int2, Mem


class HPI_IntDivFU(MinorFU):
    opClasses = minorMakeOpClassSet(["IntDiv"])
    timings = [HPI_SDIV_A1(), HPI_UDIV_A1(), HPI_SDIV_A64()]
    issueLat = 3
    opLat = 3


class HPI_MemFU(MinorFU):
    opClasses = minorMakeOpClassSet(
        ["MemRead", "MemWrite", "FloatMemRead", "FloatMemWrite"]
    )
    timings = [HPI_DefaultMem(), HPI_DefaultMem64()]
    opLat = 1
    cantForwardFromFUIndices = [5]  # Mem (this FU)


class HPI_MiscFU(MinorFU):
    opClasses = minorMakeOpClassSet(["IprAccess", "InstPrefetch"])
    opLat = 1


class HPI_FUPool(MinorFUPool):
    funcUnits = [
        HPI_IntFU(),  # 0
        HPI_Int2FU(),  # 1
        HPI_IntMulFU(),  # 2
        HPI_IntDivFU(),  # 3
        HPI_FloatSimdFU(),  # 4
        HPI_MemFU(),  # 5
        HPI_MiscFU(),  # 6
    ]


class HPI_MMU(ArmMMU):
    itb = ArmTLB(entry_type="instruction", size=256)
    dtb = ArmTLB(entry_type="data", size=256)


class HPI_BTB(SimpleBTB):
    numEntries = 128
    tagBits = 18


class HPI_BP(TournamentBP):
    btb = HPI_BTB()
    localPredictorSize = 64
    localCtrBits = 2
    localHistoryTableSize = 64
    globalPredictorSize = 1024
    globalCtrBits = 2
    choicePredictorSize = 1024
    choiceCtrBits = 2
    RASSize = 8
    instShiftAmt = 2


class HPI_ICache(Cache):
    data_latency = 1
    tag_latency = 1
    response_latency = 1
    mshrs = 2
    tgts_per_mshr = 8
    size = "32kB"
    assoc = 2
    # No prefetcher, this is handled by the core


class HPI_DCache(Cache):
    data_latency = 1
    tag_latency = 1
    response_latency = 1
    mshrs = 4
    tgts_per_mshr = 8
    size = "32kB"
    assoc = 4
    write_buffers = 4
    prefetcher = StridePrefetcher(queue_size=4, degree=4)


class HPI_L2(Cache):
    data_latency = 13
    tag_latency = 13
    response_latency = 5
    mshrs = 4
    tgts_per_mshr = 8
    size = "1024kB"
    assoc = 16
    write_buffers = 16
    # prefetcher FIXME


class HPI(ArmMinorCPU):
    # Inherit the doc string from the module to avoid repeating it
    # here.
    __doc__ = __doc__

    fetch1LineSnapWidth = 0
    fetch1LineWidth = 0
    fetch1FetchLimit = 1
    fetch1ToFetch2ForwardDelay = 1
    fetch1ToFetch2BackwardDelay = 1

    fetch2InputBufferSize = 2
    fetch2ToDecodeForwardDelay = 1
    fetch2CycleInput = True

    decodeInputBufferSize = 3
    decodeToExecuteForwardDelay = 1
    decodeInputWidth = 2
    decodeCycleInput = True

    executeInputWidth = 2
    executeCycleInput = True
    executeIssueLimit = 2

    # Only allow one ld/st to be issued but the second ld/st FU allows
    # back-to-back loads
    executeMemoryIssueLimit = 1

    executeCommitLimit = 2
    executeMemoryCommitLimit = 1
    executeInputBufferSize = 7

    executeMaxAccessesInMemory = 2

    executeLSQMaxStoreBufferStoresPerCycle = 2
    executeLSQRequestsQueueSize = 1
    executeLSQTransfersQueueSize = 2
    executeLSQStoreBufferSize = 5
    executeBranchDelay = 1
    executeFuncUnits = HPI_FUPool()
    executeSetTraceTimeOnCommit = True
    executeSetTraceTimeOnIssue = False

    executeAllowEarlyMemoryIssue = True

    enableIdling = True

    branchPred = HPI_BP()

    mmu = HPI_MMU()


__all__ = [
    "HPI_BP",
    "HPI_ITB",
    "HPI_DTB",
    "HPI_ICache",
    "HPI_DCache",
    "HPI_L2",
    "HPI",
]
