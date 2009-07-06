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

from ply import lex, yacc
import re

t_ignore = '\t '

# C or C++ comment (ignore)
def t_c_comment(t):
    r'/\*(.|\n)*?\*/'
    t.lexer.lineno += t.value.count('\n')

def t_cpp_comment(t):
    r'//.*'
    pass

# Define a rule so we can track line numbers
def t_newline(t):
    r'\n+'
    t.lexer.lineno += len(t.value)

reserved = {
    'global' : 'GLOBAL',
    'machine' : 'MACHINE',
    'in_port' : 'IN_PORT',
    'out_port' : 'OUT_PORT',
    'action' : 'ACTION',
    'transition' : 'TRANS',
    'structure' : 'STRUCT',
    'external_type' : 'EXTERN_TYPE',
    'enumeration' : 'ENUM',
    'peek' : 'PEEK',
    'enqueue' : 'ENQUEUE',
    'copy_head' : 'COPY_HEAD',
    'check_allocate' : 'CHECK_ALLOCATE',
    'check_stop_slots' : 'CHECK_STOP_SLOTS',
    'if' : 'IF',
    'else' : 'ELSE',
    'return' : 'RETURN',
    'THIS' : 'THIS',
    'CHIP' : 'CHIP',
    'void' : 'VOID',
    'new' : 'NEW',
}

literals = ':[]{}(),='

tokens = [ 'EQ', 'NE', 'LT', 'GT', 'LE', 'GE',
           'LEFTSHIFT', 'RIGHTSHIFT',
           'NOT', 'AND', 'OR',
           'PLUS', 'DASH', 'STAR', 'SLASH',
           'DOUBLE_COLON', 'SEMICOLON',
           'ASSIGN', 'DOT', 'LATENCY',
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
t_SEMICOLON = r';'
t_ASSIGN = r':='
t_DOT = r'\.'

class TokenError(Exception): pass
class ParseError(Exception): pass

def t_error(t):
    raise TokenError("Illegal character", t)

def t_IDENT(t):
    r'[a-zA-Z_][a-zA-Z_0-9]*'
    if t.value == 'true':
        t.type = 'LIT_BOOL'
        t.value = True
        return t

    if t.value == 'false':
        t.type = 'LIT_BOOL'
        t.value = False
        return t

    if t.value.startswith('LATENCY_'):
        t.type = 'LATENCY'
        return t

    t.type = reserved.get(t.value, 'IDENT')    # Check for reserved words
    return t

def t_FLOATNUMBER(t):
    '[0-9]+[.][0-9]+'
    try:
        t.value = float(t.value)
    except ValueError:
        raise TokenError("Illegal float", t)
    return t

def t_NUMBER(t):
    r'[0-9]+'
    try:
        t.value = int(t.value)
    except ValueError:
        raise TokenError("Illegal number", t)
    return t

def t_STRING1(t):
    r'\"[^"\n]*\"'
    t.type = 'STRING'
    return t

def t_STRING2(t):
    r"\'[^'\n]*\'"
    t.type = 'STRING'
    return t


def p_file(p):
    "file : decl_l"
    p[0] = [ x for x in p[1] if x is not None ]

def p_error(t):
    raise ParseError(t)

def p_empty(p):
    "empty :"
    pass

def p_decl_l(p):
    "decl_l : decls"
    p[0] = p[1]

def p_decls(p):
    """decls : decl decls
             | empty"""
    if len(p) == 3:
        p[0] = [ p[1] ] + p[2]
    elif len(p) == 2:
        p[0] = []

def p_decl(p):
    """decl : d_machine
            | d_action
            | d_in_port
            | d_out_port
            | t_trans
            | d_extern
            | d_global
            | d_struct
            | d_enum
            | d_object
            | d_func_decl
            | d_func_def"""
    p[0] = p[1]

def p_latency(p):
    """latency : LATENCY"""
    pass

def p_latencies(p):
    """latencies : latency latencies
                 | empty"""
    return []

def p_d_machine(p):
    """d_machine : MACHINE '(' ident pair_l ')' '{' decl_l '}'
           | MACHINE '(' ident pair_l ')' ':' type_members '{' decl_l '}'
           | MACHINE '(' ident pair_l ')' ':' latencies '{' decl_l '}'"""

    if len(p) == 9:
        decl_l = p[7]
    elif len(p) == 11:
        decl_l = p[9]
    decls = [ x for x in decl_l if x is not None ]
    p[0] = Machine(p[3], decls)

def p_d_action(p):
    "d_action : ACTION '(' ident pair_l ')' statement_l"
    p[0] = Action(p[3])

def p_d_in_port(p):
    "d_in_port : IN_PORT '(' ident ',' type ',' var pair_l ')' statement_l"
    p[0] = InPort(p[3])

def p_d_out_port(p):
    "d_out_port : OUT_PORT '(' ident ',' type ',' var pair_l ')' SEMICOLON"
    p[0] = OutPort(p[3])

def p_t_trans(p):
    """t_trans : TRANS '(' ident_l ',' ident_l ',' ident pair_l ')' ident_l
               | TRANS '(' ident_l ',' ident_l           pair_l ')' ident_l"""
    p[0] = Transition("transition")

def p_d_extern(p):
    """d_extern : EXTERN_TYPE '(' type pair_l ')' SEMICOLON
                | EXTERN_TYPE '(' type pair_l ')' '{' type_methods '}'"""
    p[0] = Extern(p[3])

def p_d_global(p):
    "d_global : GLOBAL '(' type pair_l ')' '{' type_members '}'"
    p[0] = Global(p[3])

def p_d_struct(p):
    "d_struct : STRUCT '(' type pair_l ')' '{' type_members '}'"
    p[0] = Struct(p[3])

def p_d_enum(p):
    "d_enum : ENUM '(' type pair_l ')' '{' type_enums   '}'"
    p[0] = Enum(p[3])

def p_d_object(p):
    "d_object : type ident pair_l SEMICOLON"
    p[0] = Object(p[2])

def p_d_func_decl(p):
    """d_func_decl : void ident '(' param_l ')' pair_l SEMICOLON
                   | type ident '(' param_l ')' pair_l SEMICOLON"""
    pass

def p_d_func_def(p):
    """d_func_def : void ident '(' param_l ')' pair_l statement_l
                  | type ident '(' param_l ')' pair_l statement_l"""
    p[0] = Function(p[2])

# Type fields
def p_type_members(p):
    """type_members : type_member type_members
                    | empty"""
    pass

def p_type_member(p):
    """type_member : type ident pair_l SEMICOLON
                   | type ident ASSIGN expr SEMICOLON"""
    pass

# Methods
def p_type_methods(p):
    """type_methods : type_method type_methods
                    | empty"""
    pass

def p_type_method(p):
    "type_method : type_or_void ident '(' type_l ')' pair_l SEMICOLON"
    pass

# Enum fields
def p_type_enums(p):
    """type_enums : type_enum type_enums
                  | empty"""
    pass

def p_type_enum(p):
    "type_enum : ident pair_l SEMICOLON"
    pass

# Type
def p_type_l(p):
    """type_l : types
                 | empty"""
    pass

def p_types(p):
    """types : type ',' types
             | type"""
    pass

def p_type(p):
    "type : ident"
    p[0] = p[1]

def p_void(p):
    "void : VOID"
    p[0] = None

def p_type_or_void(p):
    """type_or_void : type
                    | void"""
    p[0] = p[1]

# Formal Param
def p_param_l(p):
    """param_l : params
               | empty"""
    pass

def p_params(p):
    """params : param ',' params
              | param"""
    pass

def p_param(p):
    "param : type ident"
    pass

# Idents and lists
def p_ident(p):
    "ident : IDENT"
    p[0] = p[1]

def p_ident_l(p):
    """ident_l : '{' idents '}'
                  | ident"""
    p[0] = p[1]

def p_idents(p):
    """idents : ident SEMICOLON idents
              | ident ',' idents
              | ident idents
              | empty"""
    pass

# Pair and pair lists
def p_pair_l(p):
    """pair_l : ',' pairs
                 | empty"""
    if len(p) == 3:
        p[0] = p[2]
    elif len(p) == 2:
        p[0] = None

def p_pairs(p):
    """pairs : pair ',' pairs
             | pair"""
    if len(p) == 4:
        p[3].append(p[1])
        p[0] = p[3]
    elif len(p) == 2:
        p[0] = [ p[1] ]

def p_pair(p):
    """pair : ident '=' STRING
            | ident '=' ident
            | STRING"""
    if len(p) == 4:
        p[0] = p[1], p[3]
    elif len(p) == 2:
        p[0] = "short", p[1]

# Below are the rules for action descriptions
def p_statement_l(p):
    "statement_l : '{' statements '}'"
    pass

def p_statements(p):
    """statements : statement statements
                  | empty"""
    pass

def p_expr_l(p):
    """expr_l : expr ',' expr_l
                 | expr
                 | empty"""
    pass

def p_statement(p):
    """statement : expr SEMICOLON
                 | expr ASSIGN expr SEMICOLON
                 | ENQUEUE '(' var ',' type pair_l ')' statement_l
                 | PEEK '(' var ',' type ')' statement_l
                 | COPY_HEAD '(' var ',' var pair_l ')' SEMICOLON
                 | CHECK_ALLOCATE '(' var ')' SEMICOLON
                 | CHECK_STOP_SLOTS '(' var ',' STRING ',' STRING ')' SEMICOLON
                 | if_statement
                 | RETURN expr SEMICOLON"""
    pass

def p_if_statement(p):
    """if_statement : IF '(' expr ')' statement_l ELSE statement_l
                    | IF '(' expr ')' statement_l
                    | IF '(' expr ')' statement_l ELSE if_statement"""
    pass

def p_expr(p):
    """expr :  var
            | literal
            | enumeration
            | ident '(' expr_l ')'
            | NEW type
            | THIS DOT var '[' expr ']' DOT var DOT ident '(' expr_l ')'
            | THIS DOT var '[' expr ']' DOT var DOT ident
            | CHIP '[' expr ']' DOT var '[' expr ']' DOT var DOT ident '(' expr_l ')'
            | CHIP '[' expr ']' DOT var '[' expr ']' DOT var DOT ident
            | expr DOT ident
            | expr DOT ident '(' expr_l ')'
            | type DOUBLE_COLON ident '(' expr_l ')'
            | expr '[' expr_l ']'
            | expr STAR  expr
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
            | NOT expr
            | expr RIGHTSHIFT expr
            | expr LEFTSHIFT  expr
            | '(' expr ')'"""
    pass

def p_literal(p):
    """literal : STRING
               | NUMBER
               | FLOATNUMBER
               | LIT_BOOL"""
    pass

def p_enumeration(p):
    "enumeration : ident ':' ident"
    pass

def p_var(p):
    "var : ident"
    pass

lex.lex()
yacc.yacc(write_tables=0)

slicc_generated_cc = set([
    'ControllerFactory.cc',
    'MachineType.cc'])

slicc_generated_hh = set([
    'ControllerFactory.hh',
    'MachineType.hh',
    'Types.hh',
    'protocol_name.hh' ])

class Machine(object):
    def __init__(self, name, decls):
        self.name = name
        self.decls = decls

    def add(self, hh, cc):
        hh.add('%s_Controller.hh' % self.name)
        hh.add('%s_Profiler.hh' % self.name)

        cc.add('%s_Controller.cc' % self.name)
        cc.add('%s_Profiler.cc' % self.name)
        cc.add('%s_Transitions.cc' % self.name)
        cc.add('%s_Wakeup.cc' % self.name)

        for decl in self.decls:
            decl.add(hh, cc, self.name)

class Declaration(object):
    hh = False
    cc = False
    def __init__(self, name):
        self.name = name

    def add(self, hh, cc, name=None):
        #print '>>>', type(self).__name__, self.name
        if name:
            name += '_'
        else:
            name = ""
        if self.hh:
            hh.add('%s%s.hh' % (name, self.name))
        if self.cc:
            cc.add('%s%s.cc' % (name, self.name))

class Action(Declaration): pass
class InPort(Declaration): pass
class OutPort(Declaration): pass
class Transition(Declaration): pass
class Extern(Declaration): pass
class Global(Declaration):
    hh = True
    cc = True
class Struct(Declaration):
    hh = True
    cc = True
class Enum(Declaration):
    hh = True
    cc = True
class Object(Declaration): pass
class Function(Declaration):
    cc = True

def read_slicc(sources):
    if not isinstance(sources, (list,tuple)):
        sources = [ sources ]

    sm_files = []
    for source in sources:
        for sm_file in file(source, "r"):
            sm_file = sm_file.strip()
            if not sm_file:
                continue
            if sm_file.startswith("#"):
                continue
            sm_files.append(sm_file)

    return sm_files

def scan(filenames):
    hh = slicc_generated_hh.copy()
    cc = slicc_generated_cc.copy()

    for filename in filenames:
        lex.lexer.lineno = 1
        try:
            results = yacc.parse(file(filename, 'r').read())
        except (TokenError, ParseError), e:
            raise type(e), tuple([filename] + [ i for i in e ])

        for result in results:
            result.add(hh, cc)

    return list(hh), list(cc)

if __name__ == '__main__':
    import sys

    hh, cc = scan(read_slicc(sys.argv[1:]))
    hh.sort()
    cc.sort()
    print 'Headers:'
    for i in hh:
        print '    %s' % i

    print 'Sources:'
    for i in cc:
        print '    %s' % i
