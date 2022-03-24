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
from slicc.symbols import StateMachine, Type


class MachineAST(DeclAST):
    def __init__(self, slicc, mtype, pairs_ast, config_parameters, decls):
        super().__init__(slicc, pairs_ast)

        self.ident = mtype.value
        self.pairs_ast = pairs_ast
        self.config_parameters = config_parameters
        self.decls = decls

    def __repr__(self):
        return f"[Machine: {self.ident!r}]"

    def files(self, parent=None):
        file_prefix = f"{self.slicc.protocol}/{self.ident}"
        # Can't have multiple python simobject files with the same name
        # So, we have to prepend the protocol name to the .py file
        py_prefix = f"{self.slicc.protocol}/{self.slicc.protocol}_{self.ident}"
        s = set(
            (
                f"{file_prefix}_Controller.cc",
                f"{file_prefix}_Controller.hh",
                f"{py_prefix}_Controller.py",
                f"{file_prefix}_Transitions.cc",
                f"{file_prefix}_Wakeup.cc",
            )
        )

        s |= self.decls.files(self.ident)
        return s

    def generate(self):
        # Make a new frame
        self.symtab.pushFrame()

        # Create a new machine
        machine = StateMachine(
            self.symtab,
            self.ident,
            self.location,
            self.pairs,
            self.config_parameters,
        )

        self.symtab.newCurrentMachine(machine)

        # Generate code for all the internal decls
        self.decls.generate()

        # Build the transition table
        machine.buildTable()

        # Pop the frame
        self.symtab.popFrame()

    def findMachines(self):
        mtype = self.ident
        machine_type = self.symtab.find("MachineType", Type)
        if not machine_type.checkEnum(mtype):
            self.error(f"Duplicate machine name: {machine_type}:{mtype}")
