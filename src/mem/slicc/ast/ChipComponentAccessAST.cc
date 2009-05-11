
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
 * ChipComponentAccessAST.C
 *
 * Description: See ChipComponentAccessAST.h
 *
 * $Id: ChipComponentAccessAST.C 1.9 04/06/18 21:00:08-00:00 beckmann@cottons.cs.wisc.edu $
 *
 */

#include "ChipComponentAccessAST.hh"

ChipComponentAccessAST::ChipComponentAccessAST(VarExprAST* machine, ExprAST* mach_version, VarExprAST* component, string* proc_name, Vector<ExprAST*>* expr_vec_ptr)

  : ExprAST()
{
  m_chip_ver_expr_ptr = NULL;
  m_mach_var_ptr = machine;
  m_comp_var_ptr = component;
  m_mach_ver_expr_ptr = mach_version;
  m_expr_vec_ptr = expr_vec_ptr;
  m_proc_name_ptr = proc_name;
  m_field_name_ptr = NULL;
}

ChipComponentAccessAST::ChipComponentAccessAST(VarExprAST* machine, ExprAST* mach_version, VarExprAST* component, string* field_name)

  : ExprAST()
{
  m_chip_ver_expr_ptr = NULL;
  m_mach_var_ptr = machine;
  m_comp_var_ptr = component;
  m_mach_ver_expr_ptr = mach_version;
  m_expr_vec_ptr = NULL;
  m_proc_name_ptr = NULL;
  m_field_name_ptr = field_name;
}

ChipComponentAccessAST::ChipComponentAccessAST(ExprAST* chip_version, VarExprAST* machine, ExprAST* mach_version, VarExprAST* component, string* proc_name, Vector<ExprAST*>* expr_vec_ptr)

  : ExprAST()
{
  m_chip_ver_expr_ptr = chip_version;
  m_mach_var_ptr = machine;
  m_comp_var_ptr = component;
  m_mach_ver_expr_ptr = mach_version;
  m_expr_vec_ptr = expr_vec_ptr;
  m_proc_name_ptr = proc_name;
  m_field_name_ptr = NULL;
}

ChipComponentAccessAST::ChipComponentAccessAST(ExprAST* chip_version, VarExprAST* machine, ExprAST* mach_version, VarExprAST* component, string* field_name)

  : ExprAST()
{
  m_chip_ver_expr_ptr = chip_version;
  m_mach_var_ptr = machine;
  m_comp_var_ptr = component;
  m_mach_ver_expr_ptr = mach_version;
  m_expr_vec_ptr = NULL;
  m_proc_name_ptr = NULL;
  m_field_name_ptr = field_name;
}



ChipComponentAccessAST::~ChipComponentAccessAST()
{
  if (m_expr_vec_ptr != NULL) {
    int size = m_expr_vec_ptr->size();
    for(int i=0; i<size; i++) {
      delete (*m_expr_vec_ptr)[i];
    }
  }

  delete m_mach_var_ptr;
  delete m_comp_var_ptr;
  delete m_mach_ver_expr_ptr;

  if (m_proc_name_ptr != NULL) {
    delete m_proc_name_ptr;
  }

  if (m_field_name_ptr != NULL) {
    delete m_field_name_ptr;
  }

  if (m_chip_ver_expr_ptr != NULL) {
    delete m_chip_ver_expr_ptr;
  }
}

Type* ChipComponentAccessAST::generate(string& code) const
{
  Type* void_type_ptr = g_sym_table.getType("void");
  Type* ret_type_ptr;


  code += "(";

  Var* v = g_sym_table.getMachComponentVar(m_mach_var_ptr->getName(), m_comp_var_ptr->getName());

  string orig_code = v->getCode();
  string working_code;

  if (m_chip_ver_expr_ptr != NULL) {
    // replace m_chip_ptr with specified chip

    unsigned int t = orig_code.find("m_chip_ptr");
    assert(t != string::npos);
    string code_temp0 = orig_code.substr(0, t);
    string code_temp1 = orig_code.substr(t+10);

    working_code += code_temp0;
    working_code += "g_system_ptr->getChip(";
    m_chip_ver_expr_ptr->generate(working_code);
    working_code += ")";
    working_code += code_temp1;
  }
  else {
    working_code += orig_code;
  }

  // replace default "m_version" with the version we really want
  unsigned int tmp_uint = working_code.find("m_version");
  assert(tmp_uint != string::npos);
  string code_temp2 = working_code.substr(0, tmp_uint);
  string code_temp3 = working_code.substr(tmp_uint+9);

  code += code_temp2;
  code += "(";
  m_mach_ver_expr_ptr->generate(code);
  code += ")";
  code += code_temp3;
  code += ")";

  if (m_proc_name_ptr != NULL) {
    // method call
    code += ".";

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
    code += ")";

    Type* obj_type_ptr = v->getType();
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
    ret_type_ptr = obj_type_ptr->methodReturnType(methodId);
  }
  else if (m_field_name_ptr != NULL) {
    Type* obj_type_ptr = v->getType();
    code += ").m_" + (*m_field_name_ptr);

    // Verify that this is a valid field name for this type
    if (!obj_type_ptr->dataMemberExist(*m_field_name_ptr)) {
      error("Invalid object field: Type '" + obj_type_ptr->toString() + "' does not have data member " + *m_field_name_ptr);
    }

    // Return the type of the field
    ret_type_ptr = obj_type_ptr->dataMemberType(*m_field_name_ptr);
  }
  else {
    assert(0);
  }

  return ret_type_ptr;
}

void ChipComponentAccessAST::findResources(Map<Var*, string>& resource_list) const
{

}

void ChipComponentAccessAST::print(ostream& out) const
{
  out << "[ChipAccessExpr: " << *m_expr_vec_ptr << "]";
}
