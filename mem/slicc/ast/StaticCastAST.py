# Copyright (c) 2009 Advanced Micro Devices, Inc.
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


class StaticCastAST(ExprAST):
    def __init__(self, slicc, type_ast, type_modifier, expr_ast):
        super().__init__(slicc)

        self.type_ast = type_ast
        self.expr_ast = expr_ast
        self.type_modifier = type_modifier

    def __repr__(self):
        return f"[StaticCastAST: {self.expr_ast!r}]"

    def generate(self, code, **kwargs):
        actual_type, ecode = self.expr_ast.inline(True)
        if self.type_modifier == "pointer":
            code("static_cast<${{self.type_ast.type.c_ident}} *>($ecode)")
        else:
            code("static_cast<${{self.type_ast.type.c_ident}} &>($ecode)")

        if not "interface" in self.type_ast.type:
            self.expr_ast.error(
                "static cast only premitted for those types "
                "that implement inherit an interface"
            )

        # The interface type should match
        if str(actual_type) != str(self.type_ast.type["interface"]):
            self.expr_ast.error(
                "static cast miss-match, type is '%s',"
                "but inherited type is '%s'",
                actual_type,
                self.type_ast.type["interface"],
            )

        return self.type_ast.type
