# Copyright (c) 2023 Arm Limited
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
# Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
# Copyright (c) 2009 The Hewlett-Packard Development Company
# All rights reserved.
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
from slicc.ast.ExprAST import ExprAST
from slicc.symbols import Type


class InfixOperatorExprAST(ExprAST):
    def __init__(self, slicc, left, op, right):
        super().__init__(slicc)

        self.left = left
        self.op = op
        self.right = right

    def __repr__(self):
        return f"[InfixExpr: {self.left!r} {self.op} {self.right!r}]"

    def generate(self, code, **kwargs):
        lcode = self.slicc.codeFormatter()
        rcode = self.slicc.codeFormatter()

        ltype = self.left.generate(lcode)
        rtype = self.right.generate(rcode)

        # Figure out what the input and output types should be
        if self.op in ("==", "!=", ">=", "<=", ">", "<"):
            output = "bool"
            if ltype != rtype:
                self.error(
                    "Type mismatch: left and right operands of "
                    + "operator '%s' must be the same type. "
                    + "left: '%s', right: '%s'",
                    self.op,
                    ltype,
                    rtype,
                )
        else:
            expected_types = []
            output = None

            if self.op in ("&&", "||"):
                # boolean inputs and output
                expected_types = [("bool", "bool", "bool")]
            elif self.op in ("<<", ">>"):
                expected_types = [
                    ("int", "int", "int"),
                    ("Cycles", "int", "Cycles"),
                ]
            elif self.op in ("+", "-", "*", "/", "%"):
                expected_types = [
                    ("int", "int", "int"),
                    ("Cycles", "Cycles", "Cycles"),
                    ("Tick", "Tick", "Tick"),
                    ("Addr", "Addr", "Addr"),
                    ("Cycles", "int", "Cycles"),
                    ("Scalar", "int", "Scalar"),
                    ("int", "bool", "int"),
                    ("bool", "int", "int"),
                    ("int", "Cycles", "Cycles"),
                    ("Addr", "int", "Addr"),
                    ("int", "Addr", "Addr"),
                ]
            else:
                self.error(f"No operator matched with {self.op}!")

            for expected_type in expected_types:
                left_input_type = self.symtab.find(expected_type[0], Type)
                right_input_type = self.symtab.find(expected_type[1], Type)

                if (left_input_type == ltype) and (right_input_type == rtype):
                    output = expected_type[2]

            if output == None:
                self.error(
                    "Type mismatch: operands ({}, {}) for operator "
                    "'{}' failed to match with the expected types".format(
                        ltype,
                        rtype,
                        self.op,
                    ),
                )

        # All is well
        fix = code.nofix()
        code("($lcode ${{self.op}} $rcode)")
        code.fix(fix)
        return self.symtab.find(output, Type)


class PrefixOperatorExprAST(ExprAST):
    def __init__(self, slicc, op, operand):
        super().__init__(slicc)

        self.op = op
        self.operand = operand

    def __repr__(self):
        return f"[PrefixExpr: {self.op} {self.operand!r}]"

    def generate(self, code, **kwargs):
        opcode = self.slicc.codeFormatter()
        optype = self.operand.generate(opcode)

        # Figure out what the input and output types should be
        opmap = {"!": "bool", "-": "int", "++": "Scalar"}
        if self.op in opmap:
            output = opmap[self.op]
            type_in_symtab = self.symtab.find(opmap[self.op], Type)
            if optype != type_in_symtab:
                self.error(
                    "Type mismatch: right operand of "
                    + "unary operator '%s' must be of type '%s'. ",
                    self.op,
                    type_in_symtab,
                )
        else:
            self.error("Invalid prefix operator '%s'", self.op)

        # All is well
        fix = code.nofix()
        code("(${{self.op}} $opcode)")
        code.fix(fix)

        return self.symtab.find(output, Type)
