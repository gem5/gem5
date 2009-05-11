
/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * $Id$
 *
 */

#include "mem/slicc/ast/CheckStopSlotsStatementAST.hh"
#include "mem/slicc/symbols/SymbolTable.hh"
#include "mem/slicc/ast/VarExprAST.hh"
#include "mem/slicc/ast/PairListAST.hh"

CheckStopSlotsStatementAST::CheckStopSlotsStatementAST(VarExprAST* variable, string* condStr, string* bankStr)
  : StatementAST()
{
  m_variable = variable;
  m_condStr_ptr = condStr;
  m_bankStr_ptr = bankStr;
}

CheckStopSlotsStatementAST::~CheckStopSlotsStatementAST()
{
  delete m_variable;
  delete m_condStr_ptr;
  delete m_bankStr_ptr;
}

void CheckStopSlotsStatementAST::generate(string& code, Type* return_type_ptr) const
{

  // Make sure the variable is valid
  m_variable->getVar();

}

void CheckStopSlotsStatementAST::findResources(Map<Var*, string>& resource_list) const
{
  Type* type_ptr;

  Var* var_ptr = m_variable->getVar();
  string check_code;

  if (*m_condStr_ptr == "((*in_msg_ptr)).m_isOnChipSearch") {
    check_code += "    const Response9Msg* in_msg_ptr;\n";
    check_code += "    in_msg_ptr = dynamic_cast<const Response9Msg*>(((*(m_chip_ptr->m_L2Cache_responseToL2Cache9_vec[m_version]))).peek());\n";
    check_code += "    assert(in_msg_ptr != NULL);\n";
  }

  check_code += "    if (";
  check_code += *m_condStr_ptr;
  check_code += ") {\n";

  check_code += "      if (!";
  type_ptr = m_variable->generate(check_code);
  check_code += ".isDisableSPossible((((*(m_chip_ptr->m_DNUCAmover_ptr))).getBankPos(";
  check_code += *m_bankStr_ptr;
  check_code += ")))) {\n";
  if(CHECK_INVALID_RESOURCE_STALLS) {
    check_code += "        assert(priority >= ";
    type_ptr = m_variable->generate(check_code);
    check_code += ".getPriority());\n";
  }
  check_code += "        return TransitionResult_ResourceStall;\n";
  check_code += "      }\n";
  check_code += "    } else {\n";
  check_code += "      if (!";
  type_ptr = m_variable->generate(check_code);
  check_code += ".isDisableFPossible((((*(m_chip_ptr->m_DNUCAmover_ptr))).getBankPos(";
  check_code += *m_bankStr_ptr;
  check_code += ")))) {\n";
  if(CHECK_INVALID_RESOURCE_STALLS) {
    check_code += "        assert(priority >= ";
    type_ptr = m_variable->generate(check_code);
    check_code += ".getPriority());\n";
  }
  check_code += "        return TransitionResult_ResourceStall;\n";
  check_code += "      }\n";
  check_code += "    }\n";

  assert(!resource_list.exist(var_ptr));
  resource_list.add(var_ptr, check_code);

}

void CheckStopSlotsStatementAST::print(ostream& out) const
{
  out << "[CheckStopSlotsStatementAst: " << *m_variable << "]";
}
