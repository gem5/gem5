# Copyright (c) 2003-2005 The Regents of The University of Michigan
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
# Authors: Gabe Black

import os
import sys
import re
import string
import traceback
# get type names
from types import *

# Prepend the directory where the PLY lex & yacc modules are found
# to the search path.
sys.path[0:0] = [os.environ['M5_PLY']]

from ply import lex
from ply import yacc

##########################################################################
#
# Base classes for use outside of the assembler
#
##########################################################################

class Micro_Container(object):
    def __init__(self, name):
        self.microops = []
        self.name = name
        self.directives = {}
        self.micro_classes = {}
        self.labels = {}

    def add_microop(self, microop):
        self.microops.append(microop)

    def __str__(self):
        string = "%s:\n" % self.name
        for microop in self.microops:
            string += "  %s\n" % microop
        return string

class Macroop(Micro_Container):
    pass

class Rom(Micro_Container):
    def __init__(self, name):
        super(Rom, self).__init__(name)
        self.externs = {}

##########################################################################
#
# Support classes
#
##########################################################################

class Label(object):
    def __init__(self):
        self.extern = False
        self.name = ""

class Block(object):
    def __init__(self):
        self.statements = []

class Statement(object):
    def __init__(self):
        self.is_microop = False
        self.is_directive = False

class Microop(Statement):
    def __init__(self):
        super(Microop, self).__init__()
        self.mnemonic = ""
        self.labels = []
        self.is_microop = True
        self.params = ""

class Directive(Statement):
    def __init__(self):
        super(Directive, self).__init__()
        self.name = ""
        self.is_directive = True

##########################################################################
#
# Functions that handle common tasks
#
##########################################################################

def print_error(message):
    print
    print "*** %s" % message
    print

def handle_statement(parser, container, statement):
    if statement.is_microop:
        try:
            microop = eval('parser.microops[statement.mnemonic](%s)' %
                    statement.params)
        except:
            print_error("Error creating microop object.")
            raise
        try:
            for label in statement.labels:
                container.labels[label.name] = microop
                if label.extern:
                    container.externs[label.name] = microop
            container.add_microop(microop)
        except:
            print_error("Error adding microop.")
            raise
    elif statement.is_directive:
        try:
            eval('container.%s()' % statement.name)
        except:
            print_error("Error executing directive.")
            print container.directives
            raise
    else:
        raise Exception, "Didn't recognize the type of statement", statement

##########################################################################
#
# Lexer specification
#
##########################################################################

# Error handler.  Just call exit.  Output formatted to work under
# Emacs compile-mode.  Optional 'print_traceback' arg, if set to True,
# prints a Python stack backtrace too (can be handy when trying to
# debug the parser itself).
def error(lineno, string, print_traceback = False):
    # Print a Python stack backtrace if requested.
    if (print_traceback):
        traceback.print_exc()
    if lineno != 0:
        line_str = "%d:" % lineno
    else:
        line_str = ""
    sys.exit("%s %s" % (line_str, string))

reserved = ('DEF', 'MACROOP', 'ROM', 'EXTERN')

tokens = reserved + (
        # identifier
        'ID',
        # arguments for microops and directives
        'PARAMS',

        'LPAREN', 'RPAREN',
        'LBRACE', 'RBRACE',
        #'COMMA',
        'COLON', 'SEMI', 'DOT',
        'NEWLINE'
        )

# New lines are ignored at the top level, but they end statements in the
# assembler
states = (
    ('asm', 'exclusive'),
    ('params', 'exclusive'),
)

reserved_map = { }
for r in reserved:
    reserved_map[r.lower()] = r

def t_params_COLON(t):
    r':'
    t.lexer.begin('asm')
    return t

def t_asm_ID(t):
    r'[A-Za-z_]\w*'
    t.type = reserved_map.get(t.value, 'ID')
    t.lexer.begin('params')
    return t

def t_ANY_ID(t):
    r'[A-Za-z_]\w*'
    t.type = reserved_map.get(t.value, 'ID')
    return t

def t_params_PARAMS(t):
    r'([^\n;]|((?<=\\)[\n;]))+'
    t.lineno += t.value.count('\n')
    t.lexer.begin('asm')
    return t

def t_INITIAL_LBRACE(t):
    r'\{'
    t.lexer.begin('asm')
    return t

def t_asm_RBRACE(t):
    r'\}'
    t.lexer.begin('INITIAL')
    return t

def t_INITIAL_NEWLINE(t):
    r'\n+'
    t.lineno += t.value.count('\n')

def t_asm_NEWLINE(t):
    r'\n+'
    t.lineno += t.value.count('\n')
    return t

def t_params_NEWLINE(t):
    r'\n+'
    t.lineno += t.value.count('\n')
    t.lexer.begin('asm')
    return t

def t_params_SEMI(t):
    r';'
    t.lexer.begin('asm')
    return t

# Basic regular expressions to pick out simple tokens
t_ANY_LPAREN = r'\('
t_ANY_RPAREN = r'\)'
#t_COMMA  = r','
t_ANY_SEMI   = r';'
t_ANY_DOT    = r'\.'

t_ANY_ignore = ' \t\x0c'

def t_ANY_error(t):
    error(t.lineno, "illegal character '%s'" % t.value[0])
    t.skip(1)

##########################################################################
#
# Parser specification
#
##########################################################################

# Start symbol for a file which may have more than one macroop or rom
# specification.
def p_file(t):
    'file : opt_rom_or_macros'

def p_opt_rom_or_macros_0(t):
    'opt_rom_or_macros : '

def p_opt_rom_or_macros_1(t):
    'opt_rom_or_macros : rom_or_macros'

def p_rom_or_macros_0(t):
    'rom_or_macros : rom_or_macro'

def p_rom_or_macros_1(t):
    'rom_or_macros : rom_or_macros rom_or_macro'

def p_rom_or_macro_0(t):
    '''rom_or_macro : rom_block'''

def p_rom_or_macro_1(t):
    '''rom_or_macro : macroop_def'''

# A block of statements
def p_block(t):
    'block : LBRACE statements RBRACE'
    block = Block()
    block.statements = t[2]
    t[0] = block

# Defines a section of microcode that should go in the current ROM
def p_rom_block(t):
    'rom_block : DEF ROM block SEMI'
    for statement in t[3].statements:
        handle_statement(t.parser, t.parser.rom, statement)
    t[0] = t.parser.rom

# Defines a macroop that jumps to an external label in the ROM
def p_macroop_def_0(t):
    'macroop_def : DEF MACROOP LPAREN ID RPAREN SEMI'
    t[0] = t[4]

# Defines a macroop that is combinationally generated
def p_macroop_def_1(t):
    'macroop_def : DEF MACROOP ID block SEMI'
    try:
        curop = t.parser.macro_type(t[3])
    except TypeError:
        print_error("Error creating macroop object.")
        raise
    for statement in t[4].statements:
        handle_statement(t.parser, curop, statement)
    t.parser.macroops.append(curop)

def p_statements_0(t):
    'statements : statement'
    if t[1]:
        t[0] = [t[1]]
    else:
        t[0] = []

def p_statements_1(t):
    'statements : statements statement'
    if t[2]:
        t[1].append(t[2])
    t[0] = t[1]

def p_statement(t):
    'statement : content_of_statement end_of_statement'
    t[0] = t[1]

# A statement can be a microop or an assembler directive
def p_content_of_statement_0(t):
    '''content_of_statement : microop
                            | directive'''
    t[0] = t[1]

def p_content_of_statement_1(t):
    'content_of_statement : '
    pass

# Statements are ended by newlines or a semi colon
def p_end_of_statement(t):
    '''end_of_statement : NEWLINE
                        | SEMI'''
    pass

def p_microop_0(t):
    'microop : labels ID'
    microop = Microop()
    microop.labels = t[1]
    microop.mnemonic = t[2]
    t[0] = microop

def p_microop_1(t):
    'microop : ID'
    microop = Microop()
    microop.mnemonic = t[1]
    t[0] = microop

def p_microop_2(t):
    'microop : labels ID PARAMS'
    microop = Microop()
    microop.labels = t[1]
    microop.mnemonic = t[2]
    microop.params = t[3]
    t[0] = microop

def p_microop_3(t):
    'microop : ID PARAMS'
    microop = Microop()
    microop.mnemonic = t[1]
    microop.params = t[2]
    t[0] = microop

def p_labels_0(t):
    'labels : label'
    t[0] = [t[1]]

def p_labels_1(t):
    'labels : labels label'
    t[1].append(t[2])
    t[0] = t[1]

def p_label_0(t):
    'label : ID COLON'
    label = Label()
    label.is_extern = False
    label.text = t[1]
    t[0] = label

def p_label_1(t):
    'label : EXTERN ID COLON'
    label = Label()
    label.is_extern = True
    label.text = t[2]
    t[0] = label

def p_directive(t):
    'directive : DOT ID'
    directive = Directive()
    directive.name = t[2]
    t[0] = directive

# Parse error handler.  Note that the argument here is the offending
# *token*, not a grammar symbol (hence the need to use t.value)
def p_error(t):
    if t:
        error(t.lineno, "syntax error at '%s'" % t.value)
    else:
        error(0, "unknown syntax error", True)

class MicroAssembler(object):

    def __init__(self, macro_type, microops, rom):
        self.lexer = lex.lex()
        self.parser = yacc.yacc()
        self.parser.macro_type = macro_type
        self.parser.macroops = []
        self.parser.microops = microops
        self.parser.rom = rom

    def assemble(self, asm):
        self.parser.parse(asm, lexer=self.lexer)
        for macroop in self.parser.macroops:
            print macroop
        print self.parser.rom
        macroops = self.parser.macroops
        self.parser.macroops = []
        return macroops
