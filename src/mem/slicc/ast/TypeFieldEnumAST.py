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
from slicc.ast.TypeFieldAST import TypeFieldAST
from slicc.symbols import Event
from slicc.symbols import RequestType
from slicc.symbols import State


class TypeFieldEnumAST(TypeFieldAST):
    def __init__(self, slicc, field_id, pairs_ast):
        super().__init__(slicc, pairs_ast)

        self.field_id = field_id
        self.pairs_ast = pairs_ast

    def __repr__(self):
        return f"[TypeFieldEnum: {self.field_id!r}]"

    def generate(self, type, **kwargs):
        if str(type) == "State":
            self.error(
                "States must in a State Declaration, not a normal enum."
            )

        # Add enumeration
        if not type.addEnum(self.field_id, self.pairs_ast.pairs):
            self.error(f"Duplicate enumeration: {type}:{self.field_id}")

        # Fill machine info
        machine = self.symtab.state_machine

        if str(type) == "Event":
            if not machine:
                self.error("Event declaration not part of a machine.")
            e = Event(self.symtab, self.field_id, self.location, self.pairs)
            machine.addEvent(e)

        if str(type) == "RequestType":
            if not machine:
                self.error("RequestType declaration not part of a machine.")
            s = RequestType(
                self.symtab, self.field_id, self.location, self.pairs
            )
            machine.addRequestType(s)
