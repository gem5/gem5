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
from slicc.ast.DeclAST import DeclAST
from slicc.ast.TypeAST import TypeAST
from slicc.symbols import Type
from slicc.symbols import Var


class OutPortDeclAST(DeclAST):
    def __init__(self, slicc, ident, msg_type, var_expr, pairs):
        super().__init__(slicc, pairs)

        self.ident = ident
        self.msg_type = msg_type
        self.var_expr = var_expr
        self.queue_type = TypeAST(slicc, "OutPort")

    def __repr__(self):
        return f"[OutPortDecl: {self.ident!r}]"

    def generate(self):
        code = self.slicc.codeFormatter(newlines=False)

        queue_type = self.var_expr.generate(code)
        if not queue_type.isOutPort:
            self.error(
                "The outport queue's type must have the 'outport' "
                + "attribute.  Type '%s' does not have this attribute.",
                (queue_type),
            )

        if not self.symtab.find(self.msg_type.ident, Type):
            self.error(
                "The message type '%s' does not exist.", self.msg_type.ident
            )

        var = Var(
            self.symtab,
            self.ident,
            self.location,
            self.queue_type.type,
            str(code),
            self.pairs,
        )
        self.symtab.newSymbol(var)
