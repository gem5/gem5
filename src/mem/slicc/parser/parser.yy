
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
 * */

%{
#include <string>
#include <stdio.h>
#include <assert.h>
#include "mem/slicc/ast/ASTs.hh"
#include <vector>

#define YYMAXDEPTH 100000
#define YYERROR_VERBOSE

extern char* yytext;

extern "C" void yyerror(char*);
extern "C" int yylex();

%}

%union {
  string* str_ptr;
  Vector<string>* string_vector_ptr;
  std::vector<string*>* stdstring_vector_ptr;

  // Decls
  DeclAST* decl_ptr;
  DeclListAST* decl_list_ptr;
  Vector<DeclAST*>* decl_vector_ptr;

  // TypeField
  TypeFieldAST* type_field_ptr;
  Vector<TypeFieldAST*>* type_field_vector_ptr;

  // Type
  TypeAST* type_ptr;
  Vector<TypeAST*>* type_vector_ptr;

  // Formal Params
  FormalParamAST* formal_param_ptr;
  Vector<FormalParamAST*>* formal_param_vector_ptr;

  // Statements
  StatementAST* statement_ptr;
  StatementListAST* statement_list_ptr;
  Vector<StatementAST*>* statement_vector_ptr;

  // Pairs
  PairAST* pair_ptr;
  PairListAST* pair_list_ptr;

  // Expressions
  VarExprAST* var_expr_ptr;
  ExprAST* expr_ptr;
  Vector<ExprAST*>* expr_vector_ptr;
}

%type <type_ptr> type void type_or_void
%type <type_vector_ptr> types type_list

  // Formal Params
%type <formal_param_ptr> formal_param
%type <formal_param_vector_ptr> formal_params formal_param_list

%type <str_ptr> ident field
%type <string_vector_ptr> ident_list idents

%type <statement_ptr> statement if_statement
%type <statement_list_ptr> statement_list
%type <statement_vector_ptr> statements

%type <decl_ptr> decl
%type <decl_list_ptr> decl_list
%type <decl_vector_ptr> decls

%type <type_field_vector_ptr> type_members type_enums type_methods
%type <type_field_ptr> type_member type_enum type_method

%type <var_expr_ptr> var
%type <expr_ptr> expr literal enumeration
%type <expr_vector_ptr> expr_list

%type <pair_ptr> pair
%type <pair_list_ptr> pair_list pairs

%token <str_ptr> IDENT STRING NUMBER FLOATNUMBER LIT_BOOL VOID
%token <str_ptr> IMBED IMBED_TYPE
%token CHIP THIS
%token ASSIGN DOUBLE_COLON DOT SEMICOLON COLON
%token GLOBAL_DECL MACHINE_DECL IN_PORT_DECL OUT_PORT_DECL
%token PEEK ENQUEUE COPY_HEAD CHECK_ALLOCATE CHECK_STOP_SLOTS
//%token DEQUEUE REMOVE_EARLY SKIP_EARLY PEEK_EARLY
%token DEBUG_EXPR_TOKEN DEBUG_MSG_TOKEN
%token ACTION_DECL TRANSITION_DECL TYPE_DECL STRUCT_DECL EXTERN_TYPE_DECL ENUM_DECL
%token TYPE_FIELD OTHER IF ELSE RETURN NEW

%token <str_ptr> EQ NE '<' '>' LE GE NOT AND OR PLUS DASH STAR SLASH RIGHTSHIFT LEFTSHIFT

%left OR
%left AND
%nonassoc EQ NE
%nonassoc '<' '>' GE LE
%left PLUS DASH
%left STAR SLASH
%nonassoc NOT
%nonassoc DOUBLE_COLON DOT '['

%%

file: decl_list { g_decl_list_ptr = $1; }

decl_list:  decls { $$ = new DeclListAST($1); }

decls: decl decls { $2->insertAtTop($1); $$ = $2; }
     | { $$ = new Vector<DeclAST*>; }
     ;

decl:  MACHINE_DECL     '(' ident pair_list ')' ':' formal_param_list '{' decl_list '}' { $$ = new MachineAST($3, $4, $7, $9); }
    |  ACTION_DECL      '(' ident pair_list ')' statement_list { $$ = new ActionDeclAST($3, $4, $6); }
    |  IN_PORT_DECL     '(' ident ',' type ',' var pair_list ')' statement_list { $$ = new InPortDeclAST($3, $5, $7, $8, $10); }
    |  OUT_PORT_DECL    '(' ident ',' type ',' var pair_list ')' SEMICOLON { $$ = new OutPortDeclAST($3, $5, $7, $8); }
    |  TRANSITION_DECL  '(' ident_list ',' ident_list ',' ident pair_list ')' ident_list { $$ = new TransitionDeclAST($3, $5, $7, $8, $10); }
    |  TRANSITION_DECL  '(' ident_list ',' ident_list           pair_list ')' ident_list { $$ = new TransitionDeclAST($3, $5, NULL, $6, $8); }
    |  EXTERN_TYPE_DECL '(' type pair_list ')' SEMICOLON            { $4->addPair(new PairAST("external", "yes")); $$ = new TypeDeclAST($3, $4, NULL); }
    |  EXTERN_TYPE_DECL '(' type pair_list ')' '{' type_methods '}' { $4->addPair(new PairAST("external", "yes")); $$ = new TypeDeclAST($3, $4, $7); }
    |  GLOBAL_DECL      '(' type pair_list ')' '{' type_members '}' { $4->addPair(new PairAST("global", "yes"));$$ = new TypeDeclAST($3, $4, $7); }
    |  STRUCT_DECL      '(' type pair_list ')' '{' type_members '}' { $$ = new TypeDeclAST($3, $4, $7); }
    |  ENUM_DECL        '(' type pair_list ')' '{' type_enums   '}' { $4->addPair(new PairAST("enumeration", "yes")); $$ = new EnumDeclAST($3, $4, $7); }
    |  type ident pair_list SEMICOLON { $$ = new ObjDeclAST($1, $2, $3); }
    |  type ident '(' formal_param_list ')' pair_list SEMICOLON { $$ = new FuncDeclAST($1, $2, $4, $6, NULL); } // non-void function
    |  void ident '(' formal_param_list ')' pair_list SEMICOLON { $$ = new FuncDeclAST($1, $2, $4, $6, NULL); } // void function
    |  type ident '(' formal_param_list ')' pair_list statement_list { $$ = new FuncDeclAST($1, $2, $4, $6, $7); } // non-void function
    |  void ident '(' formal_param_list ')' pair_list statement_list { $$ = new FuncDeclAST($1, $2, $4, $6, $7); } // void function
    ;

// Type fields

type_members: type_member type_members { $2->insertAtTop($1); $$ = $2; }
            | { $$ = new Vector<TypeFieldAST*>; }
            ;

type_member: type ident pair_list SEMICOLON { $$ = new TypeFieldMemberAST($1, $2, $3, NULL); }
           | type ident ASSIGN expr SEMICOLON { $$ = new TypeFieldMemberAST($1, $2, new PairListAST(), $4); }
           ;

// Methods
type_methods: type_method type_methods { $2->insertAtTop($1); $$ = $2; }
            | { $$ = new Vector<TypeFieldAST*>; }
            ;

type_method: type_or_void ident '(' type_list ')' pair_list SEMICOLON { $$ = new TypeFieldMethodAST($1, $2, $4, $6); }
           ;

// Enum fields
type_enums: type_enum type_enums { $2->insertAtTop($1); $$ = $2; }
          | { $$ = new Vector<TypeFieldAST*>; }
          ;

type_enum: ident pair_list SEMICOLON { $$ = new TypeFieldEnumAST($1, $2); }
         ;

// Type
type_list : types { $$ = $1; }
          | { $$ = new Vector<TypeAST*>; }
          ;

types    : type ',' types { $3->insertAtTop($1); $$ = $3; }
         | type { $$ = new Vector<TypeAST*>; $$->insertAtTop($1); }
         ;

type: ident { $$ = new TypeAST($1); }
    ;

void: VOID { $$ = new TypeAST($1); }
    ;

type_or_void: type { $$ = $1; }
    | void { $$ = $1; }
    ;

// Formal Param
formal_param_list : formal_params { $$ = $1; }
                | { $$ = new Vector<FormalParamAST*>; }
                ;

formal_params : formal_param ',' formal_params { $3->insertAtTop($1); $$ = $3; }
              | formal_param { $$ = new Vector<FormalParamAST*>; $$->insertAtTop($1); }
              ;

formal_param : type ident { $$ = new FormalParamAST($1, $2); }
             ;

// Idents and lists
ident: IDENT { $$ = $1; };

ident_list: '{' idents '}' { $$ = $2; }
          | ident { $$ = new Vector<string>; $$->insertAtTop(*($1)); delete $1; }
          ;

idents:  ident SEMICOLON idents { $3->insertAtTop(*($1)); $$ = $3; delete $1; }
      |  ident ',' idents { $3->insertAtTop(*($1)); $$ = $3; delete $1; }
      |  ident idents { $2->insertAtTop(*($1)); $$ = $2; delete $1; }
      |  { $$ = new Vector<string>; }
      ;

// Pair and pair lists
pair_list: ',' pairs { $$ = $2; }
         | { $$ = new PairListAST(); }

pairs    : pair ',' pairs { $3->addPair($1); $$ = $3; }
         | pair { $$ = new PairListAST(); $$->addPair($1); }
         ;

pair     : ident '=' STRING { $$ = new PairAST($1, $3); }
         | ident '=' ident { $$ = new PairAST($1, $3); }
         | STRING { $$ = new PairAST(new string("short"), $1); }
         ;

// Below are the rules for action descriptions

statement_list:  '{' statements '}' { $$ = new StatementListAST($2); }
              ;

statements: statement statements  { $2->insertAtTop($1); $$ = $2; }
          |  { $$ = new Vector<StatementAST*>; }
          ;

expr_list:  expr ',' expr_list  { $3->insertAtTop($1); $$ = $3; }
         |  expr { $$ = new Vector<ExprAST*>; $$->insertAtTop($1); }
         |  { $$ = new Vector<ExprAST*>; }
         ;

statement:  expr SEMICOLON { $$ = new ExprStatementAST($1); }
         |  expr ASSIGN expr SEMICOLON { $$ = new AssignStatementAST($1, $3); }
         |  ENQUEUE      '(' var ',' type pair_list ')' statement_list { $$ = new EnqueueStatementAST($3, $5, $6, $8); }
         |  PEEK         '(' var ',' type ')' statement_list { $$ = new PeekStatementAST($3, $5, $7, "peek"); }
//         |  PEEK_EARLY   '(' var ',' type ')' statement_list { $$ = new PeekStatementAST($3, $5, $7, "peekEarly"); }
         |  COPY_HEAD    '(' var ',' var pair_list ')' SEMICOLON { $$ = new CopyHeadStatementAST($3, $5, $6); }
         |  CHECK_ALLOCATE '(' var ')' SEMICOLON { $$ = new CheckAllocateStatementAST($3); }
         |  CHECK_STOP_SLOTS '(' var ',' STRING ',' STRING ')' SEMICOLON { $$ = new CheckStopSlotsStatementAST($3, $5, $7); }
         |  if_statement { $$ = $1; }
         |  RETURN expr SEMICOLON { $$ = new ReturnStatementAST($2); }
         ;

if_statement: IF '(' expr ')' statement_list ELSE statement_list { $$ = new IfStatementAST($3, $5, $7); }
            | IF '(' expr ')' statement_list { $$ = new IfStatementAST($3, $5, NULL); }
            | IF '(' expr ')' statement_list ELSE if_statement { $$ = new IfStatementAST($3, $5, new StatementListAST($7)); }
            ;

expr:  var { $$ = $1; }
    |  literal { $$ = $1; }
    |  enumeration { $$ = $1; }
    |  ident '(' expr_list ')' { $$ = new FuncCallExprAST($1, $3); }
    |  NEW type { $$ = new NewExprAST($2); }

// globally access a local chip component and call a method
    |  THIS DOT var '[' expr ']' DOT var DOT ident '(' expr_list ')' { $$ = new ChipComponentAccessAST($3, $5, $8, $10, $12 ); }
// globally access a local chip component and access a data member
    |  THIS DOT var '[' expr ']' DOT var DOT field { $$ = new ChipComponentAccessAST($3, $5, $8, $10 ); }
// globally access a specified chip component and call a method
    |  CHIP '[' expr ']' DOT var '[' expr ']' DOT var DOT ident '(' expr_list ')' { $$ = new ChipComponentAccessAST($3, $6, $8, $11, $13, $15 ); }
// globally access a specified chip component and access a data member
    |  CHIP '[' expr ']' DOT var '[' expr ']' DOT var DOT field { $$ = new ChipComponentAccessAST($3, $6, $8, $11, $13 ); }


    |  expr DOT field { $$ = new MemberExprAST($1, $3); }
    |  expr DOT ident '(' expr_list ')' { $$ = new MethodCallExprAST($1, $3, $5); }
    |  type DOUBLE_COLON ident '(' expr_list ')' { $$ = new MethodCallExprAST($1, $3, $5); }
    |  expr '[' expr_list ']' { $$ = new MethodCallExprAST($1, new string("lookup"), $3); }
    |  expr STAR  expr { $$ = new InfixOperatorExprAST($1, $2, $3); }
    |  expr SLASH expr { $$ = new InfixOperatorExprAST($1, $2, $3); }
    |  expr PLUS  expr { $$ = new InfixOperatorExprAST($1, $2, $3); }
    |  expr DASH  expr { $$ = new InfixOperatorExprAST($1, $2, $3); }
    |  expr '<'   expr { $$ = new InfixOperatorExprAST($1, $2, $3); }
    |  expr '>'   expr { $$ = new InfixOperatorExprAST($1, $2, $3); }
    |  expr LE    expr { $$ = new InfixOperatorExprAST($1, $2, $3); }
    |  expr GE    expr { $$ = new InfixOperatorExprAST($1, $2, $3); }
    |  expr EQ    expr { $$ = new InfixOperatorExprAST($1, $2, $3); }
    |  expr NE    expr { $$ = new InfixOperatorExprAST($1, $2, $3); }
    |  expr AND   expr { $$ = new InfixOperatorExprAST($1, $2, $3); }
    |  expr OR    expr { $$ = new InfixOperatorExprAST($1, $2, $3); }
    |  expr RIGHTSHIFT expr { $$ = new InfixOperatorExprAST($1, $2, $3); }
    |  expr LEFTSHIFT  expr { $$ = new InfixOperatorExprAST($1, $2, $3); }
//    |  NOT expr        { $$ = NULL; }        // FIXME - unary not
//    |  DASH expr %prec NOT { $$ = NULL; }    // FIXME - unary minus
   |  '(' expr ')' { $$ = $2; }
    ;

literal: STRING   { $$ = new LiteralExprAST($1, "string"); }
       | NUMBER   { $$ = new LiteralExprAST($1, "int"); }
       | FLOATNUMBER   { $$ = new LiteralExprAST($1, "int"); }
       | LIT_BOOL { $$ = new LiteralExprAST($1, "bool");  }
       ;

enumeration: ident ':' ident { $$ = new EnumExprAST(new TypeAST($1), $3); }
           ;

var: ident { $$ = new VarExprAST($1); }
   ;

field: ident { $$ = $1; }
     ;

%%

extern FILE *yyin;

DeclListAST* parse(string filename)
{
  FILE *file;
  file = fopen(filename.c_str(), "r");
  if (!file) {
    cerr << "Error: Could not open file: " << filename << endl;
    exit(1);
  }
  g_line_number = 1;
  g_file_name() = filename;
  yyin = file;
  g_decl_list_ptr = NULL;
  yyparse();
  return g_decl_list_ptr;
}

extern "C" void yyerror(char* s)
{
  fprintf(stderr, "%s:%d: %s at %s\n", g_file_name().c_str(), g_line_number, s, yytext);
  exit(1);
}

