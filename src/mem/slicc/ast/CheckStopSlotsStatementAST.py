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

class CheckStopSlotsStatementAST(StatementAST):
    def __init__(self, slicc, variable, condStr, bankStr):
        super(StatementAST, self).__init__(slicc)
        self.variable = variable
        self.condStr = condStr
        self.bankStr = bankStr

    def __repr__(self):
        return "[CheckStopSlotsStatementAst: %r]" % self.variable

    def generate(self, code, return_type):
        # Make sure the variable is valid
        self.variable.var

    def findResources(self, resources):
        var = self.variable.var
        assert var not in self.resources

        check_code = self.slicc.codeFormatter()
        if self.condStr == "((*in_msg_ptr)).m_isOnChipSearch":
            check_code('''
const Response9Msg* in_msg_ptr =
    dynamic_cast<const Response9Msg*>(((*(m_chip_ptr.m_L2Cache_responseToL2Cache9_vec[m_version]))).peek());
assert(in_msg_ptr != NULL);
''')

        vcode = self.variable.inline()
        bank = self.bankStr
        cond = self.condStr

        check_code('''
if ($cond) {
    auto pos = m_chip_ptr.m_DNUCAmover_ptr->getBankPos($bank)

    if (!$vcode.isDisableSPossible(pos)) {
        return TransitionResult_ResourceStall;
    }
} else {
    if (!$vcode.isDisableFPossible(pos)) {
        return TransitionResult_ResourceStall;
    }
}
''')

        resources[var] = str(check_code)
