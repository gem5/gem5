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

from slicc.ast.StatementAST import StatementAST
from slicc.symbols import Type


class IfStatementAST(StatementAST):
    def __init__(self, slicc, cond, then, else_):
        super().__init__(slicc)

        assert cond is not None
        assert then is not None

        self.cond = cond
        self.then = then
        self.else_ = else_

    def __repr__(self):
        return f"[IfStatement: {self.cond!r}{self.then!r}{self.else_!r}]"

    def generate(self, code, return_type, **kwargs):
        cond_code = self.slicc.codeFormatter()
        cond_type = self.cond.generate(cond_code)

        if cond_type != self.symtab.find("bool", Type):
            self.cond.error(
                "Condition of if stmt must be bool, type was '%s'", cond_type
            )

        # Conditional
        code.indent()
        code("if ($cond_code) {")
        # Then part
        code.indent()
        self.symtab.pushFrame()
        self.then.generate(code, return_type, **kwargs)
        self.symtab.popFrame()
        code.dedent()
        # Else part
        if self.else_:
            code("} else {")
            code.indent()
            self.symtab.pushFrame()
            self.else_.generate(code, return_type, **kwargs)
            self.symtab.popFrame()
            code.dedent()
        code("}")  # End scope

    def findResources(self, resources):
        # Take a worse case look at both paths
        self.then.findResources(resources)
        if self.else_ is not None:
            self.else_.findResources(resources)
