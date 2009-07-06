
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
 * MethodCallExprAST.cc
 *
 * Description: See MethodCallExprAST.hh
 *
 * $Id$
 *
 */

#include "mem/slicc/ast/MethodCallExprAST.hh"

MethodCallExprAST::MethodCallExprAST(ExprAST* obj_expr_ptr,
                                     string* proc_name_ptr,
                                     Vector<ExprAST*>* expr_vec_ptr)
  : ExprAST()
{
  m_obj_expr_ptr = obj_expr_ptr;
  m_type_ptr = NULL;
  m_proc_name_ptr = proc_name_ptr;
  m_expr_vec_ptr = expr_vec_ptr;
}

MethodCallExprAST::MethodCallExprAST(TypeAST* type_ptr,
                                     string* proc_name_ptr,
                                     Vector<ExprAST*>* expr_vec_ptr)
  : ExprAST()
{
  m_obj_expr_ptr = NULL;
  m_type_ptr = type_ptr;
  m_proc_name_ptr = proc_name_ptr;
  m_expr_vec_ptr = expr_vec_ptr;
}

MethodCallExprAST::~MethodCallExprAST()
{
  delete m_obj_expr_ptr;
  delete m_type_ptr;
  delete m_proc_name_ptr;
  int size = m_expr_vec_ptr->size();
  for(int i=0; i<size; i++) {
    delete (*m_expr_vec_ptr)[i];
  }
  delete m_expr_vec_ptr;
}

Type* MethodCallExprAST::generate(string& code) const
{
  Type* obj_type_ptr = NULL;

  if(m_obj_expr_ptr) {
    // member method call
    code += "((";
    obj_type_ptr = m_obj_expr_ptr->generate(code);

    code += ").";
  } else if (m_type_ptr) {
    // class method call
    code += "(" + m_type_ptr->toString() + "::";
    obj_type_ptr = m_type_ptr->lookupType();
  } else {
    // impossible
    assert(0);
  }

  Vector <Type*> paramTypes;

  // generate code
  int actual_size = m_expr_vec_ptr->size();
  code += (*m_proc_name_ptr) + "(";
  for(int i=0; i<actual_size; i++) {
    if (i != 0) {
      code += ", ";
    }
    // Check the types of the parameter
    Type* actual_type_ptr = (*m_expr_vec_ptr)[i]->generate(code);
    paramTypes.insertAtBottom(actual_type_ptr);
  }
  code += "))";

  string methodId = obj_type_ptr->methodId(*m_proc_name_ptr, paramTypes);

  // Verify that this is a method of the object
  if (!obj_type_ptr->methodExist(methodId)) {
    error("Invalid method call: Type '" + obj_type_ptr->toString() + "' does not have a method '" + methodId + "'");
  }

  int expected_size = obj_type_ptr->methodParamType(methodId).size();
  if (actual_size != expected_size) {
    // Right number of parameters
    ostringstream err;
    err << "Wrong number of parameters for function name: '" << *m_proc_name_ptr << "'";
    err << ", expected: ";
    err << expected_size;
    err << ", actual: ";
    err << actual_size;
    error(err.str());
  }

  for(int i=0; i<actual_size; i++) {
    // Check the types of the parameter
    Type* actual_type_ptr = paramTypes[i];
    Type* expected_type_ptr = obj_type_ptr->methodParamType(methodId)[i];
    if (actual_type_ptr != expected_type_ptr) {
      (*m_expr_vec_ptr)[i]->error("Type mismatch: expected: " + expected_type_ptr->toString() +
                                  " actual: " + actual_type_ptr->toString());
    }
  }

  // Return the return type of the method
  return obj_type_ptr->methodReturnType(methodId);
}

void MethodCallExprAST::findResources(Map<Var*, string>& resource_list) const
{

}

void MethodCallExprAST::print(ostream& out) const
{
  out << "[MethodCallExpr: " << *m_proc_name_ptr << *m_obj_expr_ptr << " " << *m_expr_vec_ptr << "]";
}
