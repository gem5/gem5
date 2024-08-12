# Copyright (c) 2013 Advanced Micro Devices, Inc.
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
from slicc.symbols import Var


class EnqueueStatementAST(StatementAST):
    def __init__(
        self,
        slicc,
        queue_name,
        type_ast,
        lexpr,
        bypass_strict_fifo,
        statements,
    ):
        super().__init__(slicc)

        self.queue_name = queue_name
        self.type_ast = type_ast
        self.latexpr = lexpr
        self.bypass_strict_fifo = bypass_strict_fifo
        self.statements = statements

    def __repr__(self):
        return "[EnqueueStatementAst: {} {} {}]".format(
            self.queue_name,
            self.type_ast.ident,
            self.statements,
        )

    def generate(self, code, return_type, **kwargs):
        code("{")
        code.indent()
        self.symtab.pushFrame()

        msg_type = self.type_ast.type

        # Add new local var to symbol table
        v = Var(
            self.symtab,
            "out_msg",
            self.location,
            msg_type,
            "*out_msg",
            self.pairs,
        )
        self.symtab.newSymbol(v)

        # Declare message
        code(
            "std::shared_ptr<${{msg_type.c_ident}}> out_msg = "
            "std::make_shared<${{msg_type.c_ident}}>(clockEdge(), m_ruby_system->getBlockSizeBytes());"
        )

        # The other statements
        t = self.statements.generate(code, None)
        self.queue_name.assertType("OutPort")

        if self.latexpr != None:
            ret_type, rcode = self.latexpr.inline(True)
            if self.bypass_strict_fifo != None:
                bypass_strict_fifo_code = self.bypass_strict_fifo.inline(False)
                code(
                    "(${{self.queue_name.var.code}}).enqueue("
                    "out_msg, clockEdge(), cyclesToTicks(Cycles($rcode)), $bypass_strict_fifo_code);"
                )
            else:
                code(
                    "(${{self.queue_name.var.code}}).enqueue("
                    "out_msg, clockEdge(), cyclesToTicks(Cycles($rcode)));"
                )
        else:
            code(
                "(${{self.queue_name.var.code}}).enqueue(out_msg, "
                "clockEdge(), cyclesToTicks(Cycles(1)));"
            )

        # End scope
        self.symtab.popFrame()
        code.dedent()
        code("}")

    def findResources(self, resources):
        var = self.queue_name.var
        res_count = int(resources.get(var, 0))
        resources[var] = str(res_count + 1)
