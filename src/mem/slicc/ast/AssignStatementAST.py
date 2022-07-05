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


class AssignStatementAST(StatementAST):
    def __init__(self, slicc, lvalue, rvalue):
        super().__init__(slicc)
        self.lvalue = lvalue
        self.rvalue = rvalue

    def __repr__(self):
        return "[AssignStatementAST: %r := %r]" % (self.lvalue, self.rvalue)

    def generate(self, code, return_type, **kwargs):
        lcode = self.slicc.codeFormatter()
        rcode = self.slicc.codeFormatter()

        ltype = self.lvalue.generate(lcode)
        rtype = self.rvalue.generate(rcode)

        code("$lcode = $rcode;")

        if not (
            ltype == rtype
            or (ltype.isInterface and ltype["interface"] == rtype.ident)
        ):
            # FIXME - beckmann
            # the following if statement is a hack to allow NetDest objects to
            # be assigned to Sets this allows for the previous Message
            # Destination 'Set class' to migrate to the new Message Destination
            # 'NetDest class'
            if str(ltype) != "NetDest" and str(rtype) != "Set":
                self.error(
                    "Assignment type mismatch '%s' and '%s'", ltype, rtype
                )
