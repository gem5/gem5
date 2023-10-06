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
from slicc.symbols import Transition


class TransitionDeclAST(DeclAST):
    def __init__(
        self,
        slicc,
        request_types,
        states,
        events,
        next_state,
        actions,
    ):
        super().__init__(slicc)

        self.request_types = request_types
        self.states = states
        self.events = events
        self.next_state = next_state
        self.actions = actions

    def __repr__(self):
        return "[TransitionDecl: ]"

    def generate(self):
        machine = self.symtab.state_machine

        if machine is None:
            self.error("Transition declaration not part of a machine.")

        for action in self.actions:
            if action not in machine.actions:
                self.error(
                    f"Invalid action: {action} is not part of machine: {machine}",
                )

        for request_type in self.request_types:
            if request_type not in machine.request_types:
                self.error(
                    "Invalid protocol access type: "
                    "%s is not part of machine: %s" % (request_type, machine),
                )

        for state in self.states:
            if state not in machine.states:
                self.error(
                    f"Invalid state: {state} is not part of machine: {machine}",
                )
            next_state = self.next_state or state
            for event in self.events:
                if event not in machine.events:
                    self.error(
                        f"Invalid event: {event} is not part of machine: {machine}",
                    )
                t = Transition(
                    self.symtab,
                    machine,
                    state,
                    event,
                    next_state,
                    self.actions,
                    self.request_types,
                    self.location,
                )
                machine.addTransition(t)
