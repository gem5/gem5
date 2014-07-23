# Copyright (c) 2013-2014 ARM Limited
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
#
# Authors: Andrew Bardsley

from m5.params import *
from m5.SimObject import SimObject

# These classes define an expression language over uint64_t with only
# a few operators.  This can be used to form expressions for the extra
# delay required in variable execution time instructions.
#
# Expressions, in evaluation, will have access to the ThreadContext and
# a StaticInst

class TimingExpr(SimObject):
    type = 'TimingExpr'
    cxx_header = 'cpu/timing_expr.hh'
    abstract = True;

class TimingExprLiteral(TimingExpr):
    """Literal 64 bit unsigned value"""
    type = 'TimingExprLiteral'
    cxx_header = 'cpu/timing_expr.hh'

    value = Param.UInt64("literal value")

    def set_params(self, value):
        self.value = value
        return self

class TimingExpr0(TimingExprLiteral):
    """Convenient 0"""
    value = 0

class TimingExprSrcReg(TimingExpr):
    """Find the source register number from the current inst"""
    type = 'TimingExprSrcReg'
    cxx_header = 'cpu/timing_expr.hh'

    # index = Param.Unsigned("index into inst src regs")
    index = Param.Unsigned("index into inst src regs")

    def set_params(self, index):
        self.index = index
        return self

class TimingExprReadIntReg(TimingExpr):
    """Read an architectural register"""
    type = 'TimingExprReadIntReg'
    cxx_header = 'cpu/timing_expr.hh'

    reg = Param.TimingExpr("register raw index to read")

    def set_params(self, reg):
        self.reg = reg
        return self

class TimingExprLet(TimingExpr):
    """Block of declarations"""
    type = 'TimingExprLet'
    cxx_header = 'cpu/timing_expr.hh'

    defns = VectorParam.TimingExpr("expressions for bindings")
    expr = Param.TimingExpr("body expression")

    def set_params(self, defns, expr):
        self.defns = defns
        self.expr = expr
        return self

class TimingExprRef(TimingExpr):
    """Value of a bound sub-expression"""
    type = 'TimingExprRef'
    cxx_header = 'cpu/timing_expr.hh'

    index = Param.Unsigned("expression index")

    def set_params(self, index):
        self.index = index
        return self

class TimingExprOp(Enum):
    vals = [
        'timingExprAdd', 'timingExprSub',
        'timingExprUMul', 'timingExprUDiv',
        'timingExprSMul', 'timingExprSDiv',
        'timingExprUCeilDiv', # Unsigned divide rounding up
        'timingExprEqual', 'timingExprNotEqual',
        'timingExprULessThan',
        'timingExprUGreaterThan',
        'timingExprSLessThan',
        'timingExprSGreaterThan',
        'timingExprInvert',
        'timingExprNot',
        'timingExprAnd',
        'timingExprOr',
        'timingExprSizeInBits',
        'timingExprSignExtend32To64',
        'timingExprAbs'
        ]

class TimingExprUn(TimingExpr):
    """Unary operator"""
    type = 'TimingExprUn'
    cxx_header = 'cpu/timing_expr.hh'

    op = Param.TimingExprOp("operator")
    arg = Param.TimingExpr("expression")

    def set_params(self, op, arg):
        self.op = op
        self.arg = arg
        return self

class TimingExprBin(TimingExpr):
    """Binary operator"""
    type = 'TimingExprBin'
    cxx_header = 'cpu/timing_expr.hh'

    op = Param.TimingExprOp("operator")
    left = Param.TimingExpr("LHS expression")
    right = Param.TimingExpr("RHS expression")

    def set_params(self, op, left, right):
        self.op = op
        self.left = left
        self.right = right
        return self

class TimingExprIf(TimingExpr):
    """If-then-else operator"""
    type = 'TimingExprIf'
    cxx_header = 'cpu/timing_expr.hh'

    cond = Param.TimingExpr("condition expression")
    trueExpr = Param.TimingExpr("true expression")
    falseExpr = Param.TimingExpr("false expression")

    def set_params(self, cond, trueExpr, falseExpr):
        self.cond = cond
        self.trueExpr = trueExpr
        self.falseExpr = falseExpr
        return self
