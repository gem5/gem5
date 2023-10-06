# Copyright (c) 2020 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
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
    def __init__(self, slicc, type_ast, ident, default=None, qualifier=""):
        super().__init__(slicc)
        self.type_ast = type_ast
        self.ident = ident
        self.default = default
        self.qualifier = qualifier

    def __repr__(self):
        return f"[FormalParamAST: {self.ident}]"

    @property
    def name(self):
        return self.ident

    def generate(self):
        type = self.type_ast.type
        param = f"param_{self.ident}"

        # Add to symbol table
        v = Var(
            self.symtab,
            self.ident,
            self.location,
            type,
            param,
            self.pairs,
        )
        self.symtab.newSymbol(v)

        # Qualifier is always a pointer for TBE table and Cache entries.
        # It's expected to be left unspecified or specified as ptr.
        qualifier = self.qualifier
        if str(type) == "TBE" or (
            "interface" in type and (type["interface"] == "AbstractCacheEntry")
        ):
            if qualifier not in ["", "PTR"]:
                self.warning(
                    "Parameter '%s' is always pointer. "
                    "%s qualifier ignored" % (self.ident, qualifier),
                )
            qualifier = "PTR"

        # default
        if qualifier == "":
            qualifier = "CONST_REF"

        if qualifier == "PTR":
            return type, f"{type.c_ident}* {param}"
        elif qualifier == "REF":
            return type, f"{type.c_ident}& {param}"
        elif qualifier == "CONST_REF":
            return type, f"const {type.c_ident}& {param}"
        else:
            self.error(f"Invalid qualifier for param '{self.ident}'")
