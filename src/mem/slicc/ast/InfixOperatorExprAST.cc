
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
 * InfixOperatorExprAST.C
 *
 * Description: See InfixOperatorExprAST.h
 *
 * $Id: InfixOperatorExprAST.C,v 3.2 2004/01/31 20:46:15 milo Exp $
 *
 */

#include "mem/slicc/ast/InfixOperatorExprAST.hh"

InfixOperatorExprAST::InfixOperatorExprAST(ExprAST* left_ptr,
                                           string* op_ptr,
                                           ExprAST* right_ptr)
  : ExprAST()
{
  m_left_ptr = left_ptr;
  m_op_ptr = op_ptr;
  m_right_ptr = right_ptr;
}

InfixOperatorExprAST::~InfixOperatorExprAST()
{
  delete m_left_ptr;
  delete m_op_ptr;
  delete m_right_ptr;
}

Type* InfixOperatorExprAST::generate(string& code) const
{
  code += "(";
  Type* left_type_ptr = m_left_ptr->generate(code);
  code += " " + *m_op_ptr + " ";
  Type* right_type_ptr = m_right_ptr->generate(code);
  code += ")";

  string inputs, output;
  // Figure out what the input and output types should be
  if ((*m_op_ptr == "==" ||
       *m_op_ptr == "!=")) {
    output = "bool";
    if (left_type_ptr != right_type_ptr) {
      error("Type mismatch: left & right operand of operator '" + *m_op_ptr +
            "' must be the same type." +
            "left: '" + left_type_ptr->toString() +
            "', right: '" + right_type_ptr->toString() + "'");
    }
  } else {
    if ((*m_op_ptr == "&&" ||
         *m_op_ptr == "||")) {
      // boolean inputs and output
      inputs = "bool";
      output = "bool";
    } else if ((*m_op_ptr == "==" ||
                *m_op_ptr == "!=" ||
                *m_op_ptr == ">=" ||
                *m_op_ptr == "<=" ||
                *m_op_ptr == ">" ||
                *m_op_ptr == "<")) {
      // Integer inputs, boolean output
      inputs = "int";
      output = "bool";
    } else {
      // integer inputs and output
      inputs = "int";
      output = "int";
    }

    Type* inputs_type = g_sym_table.getType(inputs);

    if (inputs_type != left_type_ptr) {
      m_left_ptr->error("Type mismatch: left operand of operator '" + *m_op_ptr +
                        "' expects input type '" + inputs + "', actual was " + left_type_ptr->toString() + "'");
    }

    if (inputs_type != right_type_ptr) {
      m_right_ptr->error("Type mismatch: right operand of operator '" + *m_op_ptr +
                         "' expects input type '" + inputs + "', actual was '" + right_type_ptr->toString() + "'");
    }
  }

  // All is well
  Type* output_type = g_sym_table.getType(output);
  return output_type;
}


void InfixOperatorExprAST::print(ostream& out) const
{
  out << "[InfixExpr: " << *m_left_ptr
      << *m_op_ptr << *m_right_ptr << "]";
}
