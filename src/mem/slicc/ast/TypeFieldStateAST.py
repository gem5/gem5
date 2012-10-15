# Copyright (c) 2011 Advanced Micro Devices, Inc.
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

from slicc.ast.TypeFieldAST import TypeFieldAST
from slicc.symbols import Event, State

class TypeFieldStateAST(TypeFieldAST):
    def __init__(self, slicc, field_id, perm_ast, pairs_ast):
        super(TypeFieldStateAST, self).__init__(slicc, pairs_ast)

        self.field_id = field_id
        self.perm_ast = perm_ast
        if not (perm_ast.type_ast.ident == "AccessPermission"):
            self.error("AccessPermission enum value must be specified")
        self.pairs_ast = pairs_ast

    def __repr__(self):
        return "[TypeFieldState: %r]" % self.field_id

    def generate(self, type):
        if not str(type) == "State":
            self.error("State Declaration must be of type State.")
        
        # Add enumeration
        if not type.addEnum(self.field_id, self.pairs_ast.pairs):
            self.error("Duplicate enumeration: %s:%s" % (type, self.field_id))

        # Fill machine info
        machine = self.symtab.state_machine

        if not machine:
            self.error("State declaration not part of a machine.")
        s = State(self.symtab, self.field_id, self.location, self.pairs)
        machine.addState(s)

        type.statePermPairAdd(s, self.perm_ast.value)


