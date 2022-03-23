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
from slicc.symbols.Type import Type


class TypeDeclAST(DeclAST):
    def __init__(self, slicc, type_ast, pairs, field_asts):
        super().__init__(slicc, pairs)

        self.type_ast = type_ast
        self.field_asts = field_asts

    def __repr__(self):
        return f"[TypeDecl: {self.type_ast!r}]"

    def files(self, parent=None):
        if "external" in self:
            return set()

        if parent:
            ident = f"{parent}_{self.type_ast.ident}"
        else:
            ident = self.type_ast.ident
        return set((f"{ident}.hh", f"{ident}.cc"))

    def generate(self):
        ident = str(self.type_ast)
        machine = self.symtab.state_machine

        # Make the new type
        new_type = Type(
            self.symtab,
            ident,
            self.location,
            self.pairs,
            self.shared,
            self.state_machine,
        )

        if machine:
            machine.addType(new_type)

        self.symtab.newSymbol(new_type)
        self.symtab.pushFrame()

        # Add all of the fields of the type to it
        for field in self.field_asts:
            field.generate(new_type)

        self.symtab.popFrame()
