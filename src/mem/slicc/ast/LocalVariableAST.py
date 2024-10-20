#
# Copyright (c) 2013 Advanced Micro Devices, Inc.
# Copyright (c) 2011 Mark D. Hill and David A. Wood
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
#

from slicc.ast.StatementAST import StatementAST
from slicc.symbols import Var


class LocalVariableAST(StatementAST):
    def __init__(self, slicc, type_ast, ident, pointer=False):
        super().__init__(slicc)
        self.type_ast = type_ast
        self.ident = ident
        self.pointer = pointer

    def __repr__(self):
        return f"[LocalVariableAST: {self.type_ast!r} {self.ident!r}]"

    @property
    def name(self):
        return self.var_name

    def inline(self, get_type=False):
        code = self.slicc.codeFormatter(fix_newlines=False)
        return_type = self.generate(code)
        if get_type:
            return return_type, code
        else:
            return code

    def generate(self, code, **kwargs):
        type = self.type_ast.type
        ident = f"{self.ident}"

        # Add to symbol table
        v = Var(
            self.symtab, self.ident, self.location, type, ident, self.pairs
        )
        self.symtab.newSymbol(v)
        if (
            self.pointer
            or str(type) == "TBE"
            or (
                # Check whether type is Entry by checking interface since
                # entries in protocol files use AbstractCacheEntry as interfaces.
                "interface" in type
                and (type["interface"] == "AbstractCacheEntry")
            )
        ):
            code += f"{type.c_ident}* {ident}"
        elif "implicit_ctor" in type and not "is_assign" in kwargs:
            code += f"{type.c_ident} {ident}({type['implicit_ctor']})"
        else:
            code += f"{type.c_ident} {ident}"
        return type
