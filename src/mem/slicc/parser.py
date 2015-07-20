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
#
# Authors: Nathan Binkert

import os.path
import re
import sys

from m5.util import code_formatter
from m5.util.grammar import Grammar, ParseError

import slicc.ast as ast
import slicc.util as util
from slicc.symbols import SymbolTable

class SLICC(Grammar):
    def __init__(self, filename, base_dir, verbose=False, traceback=False, **kwargs):
        self.protocol = None
        self.traceback = traceback
        self.verbose = verbose
        self.symtab = SymbolTable(self)
        self.base_dir = base_dir

        try:
            self.decl_list = self.parse_file(filename, **kwargs)
        except ParseError, e:
            if not self.traceback:
                sys.exit(str(e))
            raise

    def currentLocation(self):
        return util.Location(self.current_source, self.current_line,
                             no_warning=not self.verbose)

    def codeFormatter(self, *args, **kwargs):
        code = code_formatter(*args, **kwargs)
        code['protocol'] = self.protocol
        return code

    def process(self):
        self.decl_list.generate()

    def writeCodeFiles(self, code_path, includes):
        self.symtab.writeCodeFiles(code_path, includes)

    def writeHTMLFiles(self, html_path):
        self.symtab.writeHTMLFiles(html_path)

    def files(self):
        f = set(['Types.hh'])

        f |= self.decl_list.files()

        return f

    t_ignore = '\t '

    # C or C++ comment (ignore)
    def t_c_comment(self, t):
        r'/\*(.|\n)*?\*/'
        t.lexer.lineno += t.value.count('\n')

    def t_cpp_comment(self, t):
        r'//.*'

    # Define a rule so we can track line numbers
    def t_newline(self, t):
        r'\n+'
        t.lexer.lineno += len(t.value)

    reserved = {
        'protocol' : 'PROTOCOL',
        'include' : 'INCLUDE',
        'global' : 'GLOBAL',
        'machine' : 'MACHINE',
        'in_port' : 'IN_PORT',
        'out_port' : 'OUT_PORT',
        'action' : 'ACTION',
        'transition' : 'TRANS',
        'structure' : 'STRUCT',
        'external_type' : 'EXTERN_TYPE',
        'enumeration' : 'ENUM',
        'state_declaration' : 'STATE_DECL',
        'peek' : 'PEEK',
        'stall_and_wait' : 'STALL_AND_WAIT',
        'enqueue' : 'ENQUEUE',
        'check_allocate' : 'CHECK_ALLOCATE',
        'check_next_cycle' : 'CHECK_NEXT_CYCLE',
        'check_stop_slots' : 'CHECK_STOP_SLOTS',
        'static_cast' : 'STATIC_CAST',
        'if' : 'IF',
        'is_valid' : 'IS_VALID',
        'is_invalid' : 'IS_INVALID',
        'else' : 'ELSE',
        'return' : 'RETURN',
        'void' : 'VOID',
        'new' : 'NEW',
        'OOD' : 'OOD',
    }

    literals = ':[]{}(),='

    tokens = [ 'EQ', 'NE', 'LT', 'GT', 'LE', 'GE',
               'LEFTSHIFT', 'RIGHTSHIFT',
               'NOT', 'AND', 'OR',
               'PLUS', 'DASH', 'STAR', 'SLASH',
               'INCR', 'DECR',
               'DOUBLE_COLON', 'SEMI',
               'ASSIGN', 'DOT',
               'IDENT', 'LIT_BOOL', 'FLOATNUMBER', 'NUMBER', 'STRING' ]
    tokens += reserved.values()

    t_EQ = r'=='
    t_NE = r'!='
    t_LT = r'<'
    t_GT = r'>'
    t_LE = r'<='
    t_GE = r'>='
    t_LEFTSHIFT = r'<<'
    t_RIGHTSHIFT = r'>>'
    t_NOT = r'!'
    t_AND = r'&&'
    t_OR = r'\|\|'
    t_PLUS = r'\+'
    t_DASH = r'-'
    t_STAR = r'\*'
    t_SLASH = r'/'
    t_DOUBLE_COLON = r'::'
    t_SEMI = r';'
    t_ASSIGN = r':='
    t_DOT = r'\.'
    t_INCR = r'\+\+'
    t_DECR = r'--'

    precedence = (
        ('left', 'INCR', 'DECR'),
        ('left', 'AND', 'OR'),
        ('left', 'EQ', 'NE'),
        ('left', 'LT', 'GT', 'LE', 'GE'),
        ('left', 'RIGHTSHIFT', 'LEFTSHIFT'),
        ('left', 'PLUS', 'DASH'),
        ('left', 'STAR', 'SLASH'),
        ('right', 'NOT', 'UMINUS'),
    )

    def t_IDENT(self, t):
        r'[a-zA-Z_][a-zA-Z_0-9]*'
        if t.value == 'true':
            t.type = 'LIT_BOOL'
            t.value = True
            return t

        if t.value == 'false':
            t.type = 'LIT_BOOL'
            t.value = False
            return t

        # Check for reserved words
        t.type = self.reserved.get(t.value, 'IDENT')
        return t

    def t_FLOATNUMBER(self, t):
        '[0-9]+[.][0-9]+'
        try:
            t.value = float(t.value)
        except ValueError:
            raise ParseError("Illegal float", t)
        return t

    def t_NUMBER(self, t):
        r'[0-9]+'
        try:
            t.value = int(t.value)
        except ValueError:
            raise ParseError("Illegal number", t)
        return t

    def t_STRING1(self, t):
        r'\"[^"\n]*\"'
        t.type = 'STRING'
        t.value = t.value[1:-1]
        return t

    def t_STRING2(self, t):
        r"\'[^'\n]*\'"
        t.type = 'STRING'
        t.value = t.value[1:-1]
        return t

    def p_file(self, p):
        "file : decls"
        p[0] = p[1]

    def p_empty(self, p):
        "empty :"

    def p_decls(self, p):
        "decls : declsx"
        p[0] = ast.DeclListAST(self, p[1])

    def p_declsx__list(self, p):
        "declsx : decl declsx"
        if isinstance(p[1], ast.DeclListAST):
            decls = p[1].decls
        elif p[1] is None:
            decls = []
        else:
            decls = [ p[1] ]
        p[0] = decls + p[2]

    def p_declsx__none(self, p):
        "declsx : empty"
        p[0] = []

    def p_decl__protocol(self, p):
        "decl : PROTOCOL STRING SEMI"
        if self.protocol:
            msg = "Protocol can only be set once! Error at %s:%s\n" % \
                (self.current_source, self.current_line)
            raise ParseError(msg)
        self.protocol = p[2]
        p[0] = None

    def p_decl__include(self, p):
        "decl : INCLUDE STRING SEMI"
        dirname = os.path.dirname(self.current_source)
        if os.path.exists(os.path.join(dirname, p[2])):
            filename = os.path.join(dirname, p[2])
        else:
            filename = os.path.join(self.base_dir, p[2])
        p[0] = self.parse_file(filename)

    def p_decl__machine0(self, p):
        "decl : MACHINE '(' enumeration ')' ':' obj_decls '{' decls '}'"
        p[0] = ast.MachineAST(self, p[3], [], p[7], p[9])

    def p_decl__machine1(self, p):
        "decl : MACHINE '(' enumeration pairs ')' ':' obj_decls '{' decls '}'"
        p[0] = ast.MachineAST(self, p[3], p[4], p[7], p[9])

    def p_decl__action(self, p):
        "decl : ACTION '(' ident pairs ')' statements"
        p[0] = ast.ActionDeclAST(self, p[3], p[4], p[6])

    def p_decl__in_port(self, p):
        "decl : IN_PORT '(' ident ',' type ',' var pairs ')' statements"
        p[0] = ast.InPortDeclAST(self, p[3], p[5], p[7], p[8], p[10])

    def p_decl__out_port(self, p):
        "decl : OUT_PORT '(' ident ',' type ',' var pairs ')' SEMI"
        p[0] = ast.OutPortDeclAST(self, p[3], p[5], p[7], p[8])

    def p_decl__trans0(self, p):
        "decl : TRANS '(' idents ',' idents ',' ident_or_star ')' idents"
        p[0] = ast.TransitionDeclAST(self, [], p[3], p[5], p[7], p[9])

    def p_decl__trans1(self, p):
        "decl : TRANS '(' idents ',' idents ')' idents"
        p[0] = ast.TransitionDeclAST(self, [], p[3], p[5], None, p[7])

    def p_decl__trans2(self, p):
        "decl : TRANS '(' idents ',' idents ',' ident_or_star ')' idents idents"
        p[0] = ast.TransitionDeclAST(self, p[9], p[3], p[5], p[7], p[10])

    def p_decl__trans3(self, p):
        "decl : TRANS '(' idents ',' idents ')' idents idents"
        p[0] = ast.TransitionDeclAST(self, p[7], p[3], p[5], None, p[8])

    def p_decl__extern0(self, p):
        "decl : EXTERN_TYPE '(' type pairs ')' SEMI"
        p[4]["external"] = "yes"
        p[0] = ast.TypeDeclAST(self, p[3], p[4], [])

    def p_decl__global(self, p):
        "decl : GLOBAL '(' type pairs ')' '{' type_members '}'"
        p[4]["global"] = "yes"
        p[0] = ast.TypeDeclAST(self, p[3], p[4], p[7])

    def p_decl__struct(self, p):
        "decl : STRUCT '(' type pairs ')' '{' type_members '}'"
        p[0] = ast.TypeDeclAST(self, p[3], p[4], p[7])

    def p_decl__enum(self, p):
        "decl : ENUM '(' type pairs ')' '{' type_enums   '}'"
        p[4]["enumeration"] = "yes"
        p[0] = ast.EnumDeclAST(self, p[3], p[4], p[7])

    def p_decl__state_decl(self, p):
        "decl : STATE_DECL '(' type pairs ')' '{' type_states   '}'"
        p[4]["enumeration"] = "yes"
        p[4]["state_decl"] = "yes"
        p[0] = ast.StateDeclAST(self, p[3], p[4], p[7])

    # Type fields
    def p_obj_decls__list(self, p):
        "obj_decls : obj_decl obj_decls"
        p[0] = [ p[1] ] + p[2]

    def p_obj_decls__empty(self, p):
        "obj_decls : empty"
        p[0] = []

    def p_type_members__list(self, p):
        "type_members : type_member type_members"
        p[0] = [ p[1] ] + p[2]

    def p_type_members__empty(self, p):
        "type_members : empty"
        p[0] = []

    def p_type_member__0(self, p):
        """type_member : obj_decl
                       | func_decl
                       | func_def"""
        p[0] = p[1]

    # Member / Variable declarations
    def p_decl__obj_decl(self, p):
        "decl : obj_decl"
        p[0] = p[1]

    def p_obj_decl__0(self, p):
        "obj_decl : type ident pairs SEMI"
        p[0] = ast.ObjDeclAST(self, p[1], p[2], p[3], None, False)

    def p_obj_decl__1(self, p):
        "obj_decl : type STAR ident pairs SEMI"
        p[0] = ast.ObjDeclAST(self, p[1], p[3], p[4], None, True)

    def p_obj_decl__2(self, p):
        "obj_decl : type ident ASSIGN expr SEMI"
        p[0] = ast.ObjDeclAST(self, p[1], p[2], ast.PairListAST(self), p[4],
                False)

    def p_obj_decl__3(self, p):
        "obj_decl : type STAR ident ASSIGN expr SEMI"
        p[0] = ast.ObjDeclAST(self, p[1], p[3], ast.PairListAST(self), p[5],
                True)

    # Function definition and declaration
    def p_decl__func_decl(self, p):
        "decl : func_decl"
        p[0] = p[1]

    def p_func_decl__0(self, p):
        """func_decl :  void ident '(' params ')' pairs SEMI
                | type ident '(' params ')' pairs SEMI"""
        p[0] = ast.FuncDeclAST(self, p[1], p[2], p[4], p[6], None)

    def p_func_decl__1(self, p):
        """func_decl :  void ident '(' types ')' pairs SEMI
                | type ident '(' types ')' pairs SEMI"""
        p[0] = ast.FuncDeclAST(self, p[1], p[2], p[4], p[6], None)

    def p_decl__func_def(self, p):
        "decl : func_def"
        p[0] = p[1]

    def p_func_def__0(self, p):
        """func_def : void ident '(' params ')' pairs statements
            | type ident '(' params ')' pairs statements"""
        p[0] = ast.FuncDeclAST(self, p[1], p[2], p[4], p[6], p[7])

    # Enum fields
    def p_type_enums__list(self, p):
        "type_enums : type_enum type_enums"
        p[0] = [ p[1] ] + p[2]

    def p_type_enums__empty(self, p):
        "type_enums : empty"
        p[0] = []

    def p_type_enum(self, p):
        "type_enum : ident pairs SEMI"
        p[0] = ast.TypeFieldEnumAST(self, p[1], p[2])

    # States
    def p_type_states__list(self, p):
        "type_states : type_state type_states"
        p[0] = [ p[1] ] + p[2]

    def p_type_states__empty(self, p):
        "type_states : empty"
        p[0] = []

    def p_type_state(self, p):
        "type_state : ident ',' enumeration pairs SEMI"
        p[0] = ast.TypeFieldStateAST(self, p[1], p[3], p[4])

    # Formal Param
    def p_params__many(self, p):
        "params : param ',' params"
        p[0] = [ p[1] ] + p[3]

    def p_params__one(self, p):
        "params : param"
        p[0] = [ p[1] ]

    def p_params__none(self, p):
        "params : empty"
        p[0] = []

    def p_param(self, p):
        "param : type ident"
        p[0] = ast.FormalParamAST(self, p[1], p[2])

    def p_param__pointer(self, p):
        "param : type STAR ident"
        p[0] = ast.FormalParamAST(self, p[1], p[3], None, True)

    def p_param__pointer_default(self, p):
        "param : type STAR ident ASSIGN STRING"
        p[0] = ast.FormalParamAST(self, p[1], p[3], p[5], True)

    def p_param__default_number(self, p):
        "param : type ident ASSIGN NUMBER"
        p[0] = ast.FormalParamAST(self, p[1], p[2], p[4])

    def p_param__default_bool(self, p):
        "param : type ident ASSIGN LIT_BOOL"
        p[0] = ast.FormalParamAST(self, p[1], p[2], p[4])

    def p_param__default_string(self, p):
        "param : type ident ASSIGN STRING"
        p[0] = ast.FormalParamAST(self, p[1], p[2], p[4])

    # Type
    def p_types__multiple(self, p):
        "types : type ',' types"
        p[0] = [ p[1] ] + p[3]

    def p_types__one(self, p):
        "types : type"
        p[0] = [ p[1] ]

    def p_types__empty(self, p):
        "types : empty"
        p[0] = []

    def p_typestr__multi(self, p):
        "typestr : typestr DOUBLE_COLON ident"
        p[0] = '%s::%s' % (p[1], p[3])

    def p_typestr__single(self, p):
        "typestr : ident"
        p[0] = p[1]

    def p_type__one(self, p):
        "type : typestr"
        p[0] = ast.TypeAST(self, p[1])

    def p_void(self, p):
        "void : VOID"
        p[0] = ast.TypeAST(self, p[1])

    # Idents and lists
    def p_idents__braced(self, p):
        "idents : '{' identx '}'"
        p[0] = p[2]

    def p_idents__bare(self, p):
        "idents : ident"
        p[0] = [ p[1] ]

    def p_identx__multiple_1(self, p):
        """identx : ident SEMI identx
                  | ident ',' identx"""
        p[0] = [ p[1] ] + p[3]

    def p_identx__multiple_2(self, p):
        "identx : ident identx"
        p[0] = [ p[1] ] + p[2]

    def p_identx__single(self, p):
        "identx : empty"
        p[0] = [ ]

    def p_ident(self, p):
        "ident : IDENT"
        p[0] = p[1]

    def p_ident_or_star(self, p):
        """ident_or_star : ident
                         | STAR"""
        p[0] = p[1]

    # Pair and pair lists
    def p_pairs__list(self, p):
        "pairs : ',' pairsx"
        p[0] = p[2]

    def p_pairs__empty(self, p):
        "pairs : empty"
        p[0] = ast.PairListAST(self)

    def p_pairsx__many(self, p):
        "pairsx : pair ',' pairsx"
        p[0] = p[3]
        p[0].addPair(p[1])

    def p_pairsx__one(self, p):
        "pairsx : pair"
        p[0] = ast.PairListAST(self)
        p[0].addPair(p[1])

    def p_pair__assign(self, p):
        """pair : ident '=' STRING
                | ident '=' ident
                | ident '=' NUMBER"""
        p[0] = ast.PairAST(self, p[1], p[3])

    def p_pair__literal(self, p):
        "pair : STRING"
        p[0] = ast.PairAST(self, "short", p[1])

    # Below are the rules for action descriptions
    def p_statements__inner(self, p):
        "statements : '{' statements_inner '}'"
        p[0] = ast.StatementListAST(self, p[2])

    def p_statements__none(self, p):
        "statements : '{' '}'"
        p[0] = ast.StatementListAST(self, [])

    def p_statements_inner__many(self, p):
        "statements_inner : statement statements_inner"
        p[0] = [ p[1] ] + p[2]

    def p_statements_inner__one(self, p):
        "statements_inner : statement"
        p[0] = [ p[1] ]

    def p_exprs__multiple(self, p):
        "exprs : expr ',' exprs"
        p[0] = [ p[1] ] + p[3]

    def p_exprs__one(self, p):
        "exprs : expr"
        p[0] = [ p[1] ]

    def p_exprs__empty(self, p):
        "exprs : empty"""
        p[0] = []

    def p_statement__expression(self, p):
        "statement : expr SEMI"
        p[0] = ast.ExprStatementAST(self, p[1])

    def p_statement__assign(self, p):
        "statement : expr ASSIGN expr SEMI"
        p[0] = ast.AssignStatementAST(self, p[1], p[3])

    def p_statement__enqueue(self, p):
        "statement : ENQUEUE '(' var ',' type ')' statements"
        p[0] = ast.EnqueueStatementAST(self, p[3], p[5], None, p[7])

    def p_statement__enqueue_latency(self, p):
        "statement : ENQUEUE '(' var ',' type ',' expr ')' statements"
        p[0] = ast.EnqueueStatementAST(self, p[3], p[5], p[7], p[9])

    def p_statement__stall_and_wait(self, p):
        "statement : STALL_AND_WAIT '(' var ',' var ')' SEMI"
        p[0] = ast.StallAndWaitStatementAST(self, p[3], p[5])

    def p_statement__peek(self, p):
        "statement : PEEK '(' var ',' type pairs ')' statements"
        p[0] = ast.PeekStatementAST(self, p[3], p[5], p[6], p[8], "peek")

    def p_statement__check_allocate(self, p):
        "statement : CHECK_ALLOCATE '(' var ')' SEMI"
        p[0] = ast.CheckAllocateStatementAST(self, p[3])

    def p_statement__check_next_cycle(self, p):
        "statement : CHECK_NEXT_CYCLE '(' ')' SEMI"
        p[0] = ast.CheckNextCycleAST(self)

    def p_statement__check_stop(self, p):
        "statement : CHECK_STOP_SLOTS '(' var ',' STRING ',' STRING ')' SEMI"
        p[0] = ast.CheckStopStatementAST(self, p[3], p[5], p[7])

    def p_statement__return(self, p):
        "statement : RETURN expr SEMI"
        p[0] = ast.ReturnStatementAST(self, p[2])

    def p_statement__if(self, p):
        "statement : if_statement"
        p[0] = p[1]

    def p_if_statement__if(self, p):
        "if_statement : IF '(' expr ')' statements"
        p[0] = ast.IfStatementAST(self, p[3], p[5], None)

    def p_if_statement__if_else(self, p):
        "if_statement : IF '(' expr ')' statements ELSE statements"
        p[0] = ast.IfStatementAST(self, p[3], p[5], p[7])

    def p_statement__if_else_if(self, p):
        "if_statement : IF '(' expr ')' statements ELSE if_statement"
        p[0] = ast.IfStatementAST(self, p[3], p[5],
                                  ast.StatementListAST(self, p[7]))

    def p_expr__static_cast(self, p):
        "aexpr : STATIC_CAST '(' type ',' expr ')'"
        p[0] = ast.StaticCastAST(self, p[3], "ref", p[5])

    def p_expr__static_cast_ptr(self, p):
        "aexpr : STATIC_CAST '(' type ',' STRING ',' expr ')'"
        p[0] = ast.StaticCastAST(self, p[3], p[5], p[7])

    def p_expr__var(self, p):
        "aexpr : var"
        p[0] = p[1]

    def p_expr__localvar(self, p):
        "aexpr : type ident"
        p[0] = ast.LocalVariableAST(self, p[1], p[2])

    def p_expr__literal(self, p):
        "aexpr : literal"
        p[0] = p[1]

    def p_expr__enumeration(self, p):
        "aexpr : enumeration"
        p[0] = p[1]

    def p_expr__func_call(self, p):
        "aexpr : ident '(' exprs ')'"
        p[0] = ast.FuncCallExprAST(self, p[1], p[3])

    def p_expr__new(self, p):
        "aexpr : NEW type"
        p[0] = ast.NewExprAST(self, p[2])

    def p_expr__null(self, p):
        "aexpr : OOD"
        p[0] = ast.OodAST(self)

    def p_expr__member(self, p):
        "aexpr : aexpr DOT ident"
        p[0] = ast.MemberExprAST(self, p[1], p[3])

    def p_expr__member_method_call(self, p):
        "aexpr : aexpr DOT ident '(' exprs ')'"
        p[0] = ast.MemberMethodCallExprAST(self, p[1],
                    ast.FuncCallExprAST(self, p[3], p[5]))

    def p_expr__member_method_call_lookup(self, p):
        "aexpr : aexpr '[' exprs ']'"
        p[0] = ast.MemberMethodCallExprAST(self, p[1],
                    ast.FuncCallExprAST(self, "lookup", p[3]))

    def p_expr__class_method_call(self, p):
        "aexpr : type DOUBLE_COLON ident '(' exprs ')'"
        p[0] = ast.ClassMethodCallExprAST(self, p[1],
                    ast.FuncCallExprAST(self, p[3], p[5]))

    def p_expr__aexpr(self, p):
        "expr : aexpr"
        p[0] = p[1]

    def p_expr__binary_op(self, p):
        """expr : expr STAR  expr
                | expr SLASH expr
                | expr PLUS  expr
                | expr DASH  expr
                | expr LT    expr
                | expr GT    expr
                | expr LE    expr
                | expr GE    expr
                | expr EQ    expr
                | expr NE    expr
                | expr AND   expr
                | expr OR    expr
                | expr RIGHTSHIFT expr
                | expr LEFTSHIFT  expr"""
        p[0] = ast.InfixOperatorExprAST(self, p[1], p[2], p[3])

    # FIXME - unary not
    def p_expr__unary_op(self, p):
        """expr : NOT expr
                | INCR expr
                | DECR expr
                | DASH expr %prec UMINUS"""
        p[0] = ast.PrefixOperatorExprAST(self, p[1], p[2])

    def p_expr__parens(self, p):
        "aexpr : '(' expr ')'"
        p[0] = p[2]

    def p_expr__is_valid_ptr(self, p):
        "aexpr : IS_VALID '(' var ')'"
        p[0] = ast.IsValidPtrExprAST(self, p[3], True)

    def p_expr__is_invalid_ptr(self, p):
        "aexpr : IS_INVALID '(' var ')'"
        p[0] = ast.IsValidPtrExprAST(self, p[3], False)

    def p_literal__string(self, p):
        "literal : STRING"
        p[0] = ast.LiteralExprAST(self, p[1], "std::string")

    def p_literal__number(self, p):
        "literal : NUMBER"
        p[0] = ast.LiteralExprAST(self, p[1], "int")

    def p_literal__float(self, p):
        "literal : FLOATNUMBER"
        p[0] = ast.LiteralExprAST(self, p[1], "int")

    def p_literal__bool(self, p):
        "literal : LIT_BOOL"
        p[0] = ast.LiteralExprAST(self, p[1], "bool")

    def p_enumeration(self, p):
        "enumeration : ident ':' ident"
        p[0] = ast.EnumExprAST(self, ast.TypeAST(self, p[1]), p[3])

    def p_var(self, p):
        "var : ident"
        p[0] = ast.VarExprAST(self, p[1])
