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

import os
import sys
import re
import traceback

# get type names
from types import *

from ply import lex
from ply import yacc

##########################################################################
#
# Base classes for use outside of the assembler
#
##########################################################################


class MicroContainer:
    def __init__(self, name):
        self.microops = []
        self.name = name
        self.directives = {}
        self.micro_classes = {}
        self.labels = {}

    def add_microop(self, mnemonic, microop):
        microop.mnemonic = mnemonic
        microop.micropc = len(self.microops)
        self.microops.append(microop)

    def __str__(self):
        string = f"{self.name}:\n"
        for microop in self.microops:
            string += f"  {microop}\n"
        return string


class CombinationalMacroop(MicroContainer):
    pass


class RomMacroop:
    def __init__(self, name, target):
        self.name = name
        self.target = target

    def __str__(self):
        return f"{self.name}: {self.target}\n"


class Rom(MicroContainer):
    def __init__(self, name):
        super().__init__(name)
        self.externs = {}


##########################################################################
#
# Support classes
#
##########################################################################


class Label:
    def __init__(self):
        self.extern = False
        self.name = ""


class Block:
    def __init__(self):
        self.statements = []


class Statement:
    def __init__(self):
        self.is_microop = False
        self.is_directive = False
        self.params = ""


class Microop(Statement):
    def __init__(self):
        super().__init__()
        self.mnemonic = ""
        self.labels = []
        self.is_microop = True


class Directive(Statement):
    def __init__(self):
        super().__init__()
        self.name = ""
        self.is_directive = True


##########################################################################
#
# Functions that handle common tasks
#
##########################################################################


def print_error(message):
    print()
    print(f"*** {message}")
    print()


def handle_statement(parser, container, statement):
    if statement.is_microop:
        if statement.mnemonic not in parser.microops.keys():
            raise Exception(f"Unrecognized mnemonic: {statement.mnemonic}")
        parser.symbols[
            "__microopClassFromInsideTheAssembler"
        ] = parser.microops[statement.mnemonic]
        try:
            microop = eval(
                f"__microopClassFromInsideTheAssembler({statement.params})",
                {},
                parser.symbols,
            )
        except:
            print_error(
                f"Error creating microop object with mnemonic {statement.mnemonic}."
            )
            raise
        try:
            for label in statement.labels:
                container.labels[label.text] = microop
                if label.is_extern:
                    container.externs[label.text] = microop
            container.add_microop(statement.mnemonic, microop)
        except:
            print_error("Error adding microop.")
            raise
    elif statement.is_directive:
        if statement.name not in container.directives.keys():
            raise Exception(f"Unrecognized directive: {statement.name}")
        parser.symbols[
            "__directiveFunctionFromInsideTheAssembler"
        ] = container.directives[statement.name]
        try:
            eval(
                f"__directiveFunctionFromInsideTheAssembler({statement.params})",
                {},
                parser.symbols,
            )
        except:
            print_error("Error executing directive.")
            print(container.directives)
            raise
    else:
        raise Exception(f"Didn't recognize the type of statement {statement}")


##########################################################################
#
# Lexer specification
#
##########################################################################


# Error handler.  Just call exit.  Output formatted to work under
# Emacs compile-mode.  Optional 'print_traceback' arg, if set to True,
# prints a Python stack backtrace too (can be handy when trying to
# debug the parser itself).
def error(lineno, string, print_traceback=False):
    # Print a Python stack backtrace if requested.
    if print_traceback:
        traceback.print_exc()
    if lineno != 0:
        line_str = "%d:" % lineno
    else:
        line_str = ""
    sys.exit(f"{line_str} {string}")


reserved = ("DEF", "MACROOP", "ROM", "EXTERN")

tokens = reserved + (
    # identifier
    "ID",
    # arguments for microops and directives
    "PARAMS",
    "LPAREN",
    "RPAREN",
    "LBRACE",
    "RBRACE",
    "COLON",
    "SEMI",
    "DOT",
    "NEWLINE",
)

# New lines are ignored at the top level, but they end statements in the
# assembler
states = (
    ("asm", "exclusive"),
    ("params", "exclusive"),
    ("header", "exclusive"),
)

reserved_map = {}
for r in reserved:
    reserved_map[r.lower()] = r


# Ignore comments
def t_ANY_COMMENT(t):
    r"\#[^\n]*(?=\n)"


def t_ANY_MULTILINECOMMENT(t):
    r"/\*([^/]|((?<!\*)/))*\*/"


# A colon marks the end of a label. It should follow an ID which will
# put the lexer in the "params" state. Seeing the colon will put it back
# in the "asm" state since it knows it saw a label and not a mnemonic.
def t_params_COLON(t):
    r":"
    t.lexer.pop_state()
    return t


# Parameters are a string of text which don't contain an unescaped statement
# statement terminator, ie a newline or semi colon.
def t_params_PARAMS(t):
    r"([^\n;\\]|(\\[\n;\\]))+"
    t.lineno += t.value.count("\n")
    unescapeParamsRE = re.compile(r"(\\[\n;\\])")

    def unescapeParams(mo):
        val = mo.group(0)
        return val[1]

    t.value = unescapeParamsRE.sub(unescapeParams, t.value)
    t.lexer.pop_state()
    return t


# An "ID" in the micro assembler is either a label, directive, or mnemonic
# If it's either a directive or a mnemonic, it will be optionally followed by
# parameters. If it's a label, the following colon will make the lexer stop
# looking for parameters.
def t_asm_ID(t):
    r"[A-Za-z_]\w*"
    t.type = reserved_map.get(t.value, "ID")
    # If the ID is really "extern", we shouldn't start looking for parameters
    # yet. The real ID, the label itself, is coming up.
    if t.type != "EXTERN":
        t.lexer.push_state("params")
    return t


def t_header_ID(t):
    r"[A-Za-z_]\w*"
    return t


# If there is a label and you're -not- in the assembler (which would be caught
# above), don't start looking for parameters.
def t_ANY_ID(t):
    r"[A-Za-z_]\w*"
    t.type = reserved_map.get(t.value, "ID")
    if t.type == "MACROOP":
        t.lexer.push_state("asm")
        t.lexer.push_state("header")
    elif t.type == "ROM":
        t.lexer.push_state("asm")
        t.lexer.push_state("header")
    return t


# Braces enter and exit micro assembly
def t_header_LBRACE(t):
    r"\{"
    t.lexer.pop_state()
    return t


def t_asm_RBRACE(t):
    r"\}"
    t.lexer.pop_state()
    return t


# In the micro assembler, do line counting but also return a token. The
# token is needed by the parser to detect the end of a statement.
def t_asm_NEWLINE(t):
    r"\n+"
    t.lineno += t.value.count("\n")
    return t


# A newline or semi colon when looking for params signals that the statement
# is over and the lexer should go back to looking for regular assembly.
def t_params_NEWLINE(t):
    r"\n+"
    t.lineno += t.value.count("\n")
    t.lexer.pop_state()
    return t


def t_params_SEMI(t):
    r";"
    t.lexer.pop_state()
    return t


# Unless handled specially above, track newlines only for line counting.
def t_ANY_NEWLINE(t):
    r"\n+"
    t.lineno += t.value.count("\n")


# Basic regular expressions to pick out simple tokens
t_ANY_LPAREN = r"\("
t_ANY_RPAREN = r"\)"
t_ANY_SEMI = r";"
t_ANY_DOT = r"\."

t_ANY_ignore = " \t\x0c"


def t_ANY_error(t):
    error(t.lineno, f"illegal character '{t.value[0]}'")
    t.skip(1)


##########################################################################
#
# Parser specification
#
##########################################################################


# Start symbol for a file which may have more than one macroop or rom
# specification.
def p_file(t):
    "file : opt_rom_or_macros"


def p_opt_rom_or_macros_0(t):
    "opt_rom_or_macros :"


def p_opt_rom_or_macros_1(t):
    "opt_rom_or_macros : rom_or_macros"


def p_rom_or_macros_0(t):
    "rom_or_macros : rom_or_macro"


def p_rom_or_macros_1(t):
    "rom_or_macros : rom_or_macros rom_or_macro"


def p_rom_or_macro_0(t):
    """rom_or_macro : rom_block
    | macroop_def"""


# Defines a section of microcode that should go in the current ROM
def p_rom_block(t):
    "rom_block : DEF ROM block SEMI"
    if not t.parser.rom:
        print_error("Rom block found, but no Rom object specified.")
        raise TypeError("Rom block found, but no Rom object was specified.")
    for statement in t[3].statements:
        handle_statement(t.parser, t.parser.rom, statement)
    t[0] = t.parser.rom


# Defines a macroop that jumps to an external label in the ROM
def p_macroop_def_0(t):
    "macroop_def : DEF MACROOP ID LPAREN ID RPAREN SEMI"
    if not t.parser.rom_macroop_type:
        print_error(
            "ROM based macroop found, but no ROM macroop "
            + "class was specified."
        )
        raise TypeError(
            "ROM based macroop found, but no ROM macroop "
            + "class was specified."
        )
    macroop = t.parser.rom_macroop_type(t[3], t[5])
    t.parser.macroops[t[3]] = macroop


# Defines a macroop that is combinationally generated
def p_macroop_def_1(t):
    "macroop_def : DEF MACROOP ID block SEMI"
    try:
        curop = t.parser.macro_type(t[3])
    except TypeError:
        print_error("Error creating macroop object.")
        raise
    for statement in t[4].statements:
        handle_statement(t.parser, curop, statement)
    t.parser.macroops[t[3]] = curop


# A block of statements
def p_block(t):
    "block : LBRACE statements RBRACE"
    block = Block()
    block.statements = t[2]
    t[0] = block


def p_statements_0(t):
    "statements : statement"
    if t[1]:
        t[0] = [t[1]]
    else:
        t[0] = []


def p_statements_1(t):
    "statements : statements statement"
    if t[2]:
        t[1].append(t[2])
    t[0] = t[1]


def p_statement(t):
    "statement : content_of_statement end_of_statement"
    t[0] = t[1]


# A statement can be a microop or an assembler directive
def p_content_of_statement_0(t):
    """content_of_statement : microop
    | directive"""
    t[0] = t[1]


# Ignore empty statements
def p_content_of_statement_1(t):
    "content_of_statement :"
    pass


# Statements are ended by newlines or a semi colon
def p_end_of_statement(t):
    """end_of_statement : NEWLINE
    | SEMI"""
    pass


# Different flavors of microop to avoid shift/reduce errors
def p_microop_0(t):
    "microop : labels ID"
    microop = Microop()
    microop.labels = t[1]
    microop.mnemonic = t[2]
    t[0] = microop


def p_microop_1(t):
    "microop : ID"
    microop = Microop()
    microop.mnemonic = t[1]
    t[0] = microop


def p_microop_2(t):
    "microop : labels ID PARAMS"
    microop = Microop()
    microop.labels = t[1]
    microop.mnemonic = t[2]
    microop.params = t[3]
    t[0] = microop


def p_microop_3(t):
    "microop : ID PARAMS"
    microop = Microop()
    microop.mnemonic = t[1]
    microop.params = t[2]
    t[0] = microop


# Labels in the microcode
def p_labels_0(t):
    "labels : label"
    t[0] = [t[1]]


def p_labels_1(t):
    "labels : labels label"
    t[1].append(t[2])
    t[0] = t[1]


# labels on lines by themselves are attached to the following instruction.
def p_labels_2(t):
    "labels : labels NEWLINE"
    t[0] = t[1]


def p_label_0(t):
    "label : ID COLON"
    label = Label()
    label.is_extern = False
    label.text = t[1]
    t[0] = label


def p_label_1(t):
    "label : EXTERN ID COLON"
    label = Label()
    label.is_extern = True
    label.text = t[2]
    t[0] = label


# Directives for the macroop
def p_directive_0(t):
    "directive : DOT ID"
    directive = Directive()
    directive.name = t[2]
    t[0] = directive


def p_directive_1(t):
    "directive : DOT ID PARAMS"
    directive = Directive()
    directive.name = t[2]
    directive.params = t[3]
    t[0] = directive


# Parse error handler.  Note that the argument here is the offending
# *token*, not a grammar symbol (hence the need to use t.value)
def p_error(t):
    if t:
        error(t.lineno, f"syntax error at '{t.value}'")
    else:
        error(0, "unknown syntax error", True)


class MicroAssembler:
    def __init__(self, macro_type, microops, rom=None, rom_macroop_type=None):
        self.lexer = lex.lex()
        self.parser = yacc.yacc(write_tables=False)
        self.parser.macro_type = macro_type
        self.parser.macroops = {}
        self.parser.microops = microops
        self.parser.rom = rom
        self.parser.rom_macroop_type = rom_macroop_type
        self.parser.symbols = {}
        self.symbols = self.parser.symbols

    def assemble(self, asm):
        self.parser.parse(asm, lexer=self.lexer)
        macroops = self.parser.macroops
        self.parser.macroops = {}
        return macroops
