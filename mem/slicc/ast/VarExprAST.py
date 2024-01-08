# Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
# Copyright (c) 2009 The Hewlett-Packard Development Company
# Copyright (c) 2013 Advanced Micro Devices, Inc.
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
from slicc.symbols import Type, Var


class VarExprAST(ExprAST):
    def __init__(self, slicc, var):
        super().__init__(slicc)
        self._var = var

    def __repr__(self):
        return f"[VarExprAST: {self._var!r}]"

    @property
    def name(self):
        return str(self._var)

    @property
    def var(self):
        var = self.symtab.find(self._var, Var)
        if not var:
            self.error("Unrecognized variable: %s", self._var)
        return var

    def assertType(self, type_ident):
        expected_type = self.symtab.find(type_ident, Type)

        if not expected_type:
            self.error(
                "There must be a type '%s' declared in this scope", type_ident
            )

        if self.var.type != expected_type:
            self.error(
                "Incorrect type: "
                + "'%s' is expected to be type '%s' not '%s'",
                self.var.ident,
                expected_type,
                self.var.type,
            )

    def generate(self, code, **kwargs):
        fix = code.nofix()
        code("${{self.var.code}}")
        code.fix(fix)
        return self.var.type
