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
from slicc.symbols import Var

class ObjDeclAST(DeclAST):
    def __init__(self, slicc, type_ast, ident, pairs, rvalue, pointer):
        super().__init__(slicc, pairs)

        self.type_ast = type_ast
        self.ident = ident
        self.rvalue = rvalue
        self.pointer = pointer

    def __repr__(self):
        return "[ObjDecl: %r]" % self.ident

    def generate(self, parent = None, **kwargs):
        if "network" in self and not ("virtual_network" in self or
                                      "physical_network" in self) :
            self.error("Network queues require a 'virtual_network' attribute")

        type = self.type_ast.type

        # FIXME : should all use accessors here to avoid public member
        # variables
        if self.ident == "version":
            c_code = "m_version"
        elif self.ident == "machineID":
            c_code = "m_machineID"
        elif self.ident == "clusterID":
            c_code = "m_clusterID"
        elif self.ident == "recycle_latency":
            c_code = "m_recycle_latency"
        else:
            c_code = "(*m_%s_ptr)" % (self.ident)

        # check type if this is a initialization
        init_code = ""
        if self.rvalue:
            rvalue_type,init_code = self.rvalue.inline(True)
            if type != rvalue_type:
                self.error("Initialization type mismatch '%s' and '%s'" % \
                           (type, rvalue_type))

        machine = self.symtab.state_machine

        v = Var(self.symtab, self.ident, self.location, type, c_code,
                self.pairs, machine)

        # Add data member to the parent type
        if parent:
            if not parent.addDataMember(self.ident, type, self.pairs, init_code):
                self.error("Duplicate data member: %s:%s" % (parent, self.ident))

        elif machine:
            machine.addObject(v)

        else:
            self.symtab.newSymbol(v)
