#ifndef NEWEXPRAST_H
#define NEWEXPRAST_H

#include "mem/slicc/ast/ExprAST.hh"
#include "mem/slicc/ast/TypeAST.hh"
#include "mem/slicc/symbols/Type.hh"

class NewExprAST : public ExprAST
{
public:
  NewExprAST(TypeAST* type_ptr) : ExprAST() { m_type_ptr = type_ptr; }
  Type* generate(string & code) const;
  void print(ostream & out) const { out << "[NewExprAST: " << *m_type_ptr << "]"; }
  string getName() const { return m_type_ptr->toString(); }

private:
  TypeAST* m_type_ptr;
};

#endif
