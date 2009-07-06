
#include "mem/slicc/ast/NewExprAST.hh"

Type* NewExprAST::generate(string & code) const
{
  Type* type = m_type_ptr->lookupType();
  code += "new " + type->cIdent();
  return type;
}
