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

class CopyHeadStatementAST(StatementAST):
    def __init__(self, slicc, in_queue, out_queue, pairs):
        super(CopyHeadStatementAST, self).__init__(slicc, pairs)

        self.in_queue = in_queue
        self.out_queue_ptr = out_queue

    def __repr__(self):
        return "[CopyHeadStatementAst: %r %r]" % (self.in_queue,
                                                  self.out_queue)

    def generate(self, code, return_type):
        self.in_queue.assertType("InPort")
        self.out_queue.assertType("OutPort")

        out_code = self.out_queue.var.code
        in_code = self.in_queue.var.code
        latency = self.get("latency", "COPY_HEAD_LATENCY")
        code("$out_code.enqueue($in_code.getMsgPtrCopy(), $latency);")

    def findResources(self, resources):
        var = self.out_queue.var
        resources[var] = str(int(resources.get(var, "0")) + 1)
