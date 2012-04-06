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

from slicc.ast.ExprAST import ExprAST

class MemberExprAST(ExprAST):
    def __init__(self, slicc, expr_ast, field):
        super(MemberExprAST, self).__init__(slicc)

        self.expr_ast = expr_ast
        self.field = field

    def __repr__(self):
        return "[MemberExprAST: %r.%r]" % (self.expr_ast, self.field)

    def generate(self, code):
        return_type, gcode = self.expr_ast.inline(True)
        fix = code.nofix()

        if str(return_type) == "TBE" \
           or ("interface" in return_type and
            (return_type["interface"] == "AbstractCacheEntry" or
             return_type["interface"] == "AbstractEntry")):
            code("(*$gcode).m_${{self.field}}")
        else:
            code("($gcode).m_${{self.field}}")

        code.fix(fix)

        # Verify that this is a valid field name for this type
        if self.field in return_type.data_members:
            # Return the type of the field
            return return_type.data_members[self.field].type
        else:
            if "interface" in return_type:
               interface_type = self.symtab.find(return_type["interface"]);
               if interface_type != None:
                   if self.field in interface_type.data_members:
                       # Return the type of the field
                       return interface_type.data_members[self.field].type
        self.error("Invalid object field: " +
                   "Type '%s' does not have data member %s" % \
                   (return_type, self.field))
