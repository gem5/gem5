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

from slicc.ast.AST import AST
from slicc.symbols import Var

class FormalParamAST(AST):
    def __init__(self, slicc, type_ast, ident, default = None, pointer = False):
        super(FormalParamAST, self).__init__(slicc)
        self.type_ast = type_ast
        self.ident = ident
        self.default = default
        self.pointer = pointer

    def __repr__(self):
        return "[FormalParamAST: %s]" % self.ident

    @property
    def name(self):
        return self.ident

    def generate(self):
        type = self.type_ast.type
        param = "param_%s" % self.ident

        # Add to symbol table
        v = Var(self.symtab, self.ident, self.location, type, param,
                self.pairs)
        self.symtab.newSymbol(v)

        if self.pointer or str(type) == "TBE" or (
           "interface" in type and (
               type["interface"] == "AbstractCacheEntry" or
               type["interface"] == "AbstractEntry")):
            return type, "%s* %s" % (type.c_ident, param)
        else:
            return type, "const %s& %s" % (type.c_ident, param)
