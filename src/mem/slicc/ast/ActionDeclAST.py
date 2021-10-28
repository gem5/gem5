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
from slicc.symbols import Action, Type, Var

class ActionDeclAST(DeclAST):
    def __init__(self, slicc, ident, pairs, statement_list):
        super().__init__(slicc, pairs)
        self.ident = ident
        self.statement_list = statement_list

    def __repr__(self):
        return "[ActionDecl: %r]" % (self.ident)

    def generate(self):
        resources = {}

        machine = self.symtab.state_machine
        if machine is None:
            self.error("Action declaration not part of a machine.")

        if self.statement_list:
            # Add new local vars
            self.symtab.pushFrame()

            addr_type = self.symtab.find("Addr", Type)

            if addr_type is None:
                self.error("Type 'Addr' not declared.")

            var = Var(self.symtab, "address", self.location, addr_type,
                      "addr", self.pairs)
            self.symtab.newSymbol(var)

            if machine.TBEType != None:
                var = Var(self.symtab, "tbe", self.location, machine.TBEType,
                      "m_tbe_ptr", self.pairs)
                self.symtab.newSymbol(var)

            if machine.EntryType != None:
                var = Var(self.symtab, "cache_entry", self.location,
                          machine.EntryType, "m_cache_entry_ptr", self.pairs)
                self.symtab.newSymbol(var)

            # Do not allows returns in actions
            code = self.slicc.codeFormatter()
            self.statement_list.generate(code, None)
            self.pairs["c_code"] = str(code)

            self.statement_list.findResources(resources)

            self.symtab.popFrame()

        action = Action(self.symtab, self.ident, resources, self.location,
                        self.pairs)
        machine.addAction(action)
