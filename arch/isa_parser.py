#! /usr/bin/env python

# $Id$

# Copyright (c) 2003 The Regents of The University of Michigan
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
import string
# get type names
from types import *

# Check arguments.  Right now there are only two: the name of the ISA
# description (input) file and the name of the C++ decoder (output) file.
isa_desc_filename = sys.argv[1]
decoder_filename = sys.argv[2]

# Might as well suck the file in while we're here.  This way if it's a
# bad filename we don't waste a lot of time building the parser :-).
input = open(isa_desc_filename)
isa_desc = input.read()
input.close()

# Prepend the directory where the PLY lex & yacc modules are found
# to the search path.  Assumes we're compiling in a subdirectory
# of 'build' in the current tree.
sys.path[0:0] = [os.environ['M5_EXT'] + '/ply']

import lex
import yacc

#####################################################################
#
#                                Lexer
#
# The PLY lexer module takes two things as input:
# - A list of token names (the string list 'tokens')
# - A regular expression describing a match for each token.  The
#   regexp for token FOO can be provided in two ways:
#   - as a string variable named t_FOO
#   - as the doc string for a function named t_FOO.  In this case,
#     the function is also executed, allowing an action to be
#     associated with each token match.
#
#####################################################################

# Reserved words.  These are listed separately as they are matched
# using the same regexp as generic IDs, but distinguished in the
# t_ID() function.  The PLY documentation suggests this approach.
reserved = (
    'BITFIELD', 'DECLARE', 'DECODE', 'DEFAULT', 'DEF', 'FORMAT',
    'LET', 'NAMESPACE', 'SIGNED', 'TEMPLATE'
    )

# List of tokens.  The lex module requires this.
tokens = reserved + (
    # identifier
    'ID',

    # integer literal
    'INTLIT',

    # string literal
    'STRLIT',

    # code literal
    'CODELIT',

    # ( ) [ ] { } < > , ; : :: *
    'LPAREN', 'RPAREN',
# not used any more... commented out to suppress PLY warning
#    'LBRACKET', 'RBRACKET',
    'LBRACE', 'RBRACE',
    'LESS', 'GREATER',
    'COMMA', 'SEMI', 'COLON', 'DBLCOLON',
    'ASTERISK',

    # C preprocessor directives
    'CPPDIRECTIVE'
)

# Regular expressions for token matching
t_LPAREN           = r'\('
t_RPAREN           = r'\)'
# not used any more... commented out to suppress PLY warning
# t_LBRACKET         = r'\['
# t_RBRACKET         = r'\]'
t_LBRACE           = r'\{'
t_RBRACE           = r'\}'
t_LESS             = r'\<'
t_GREATER          = r'\>'
t_COMMA            = r','
t_SEMI             = r';'
t_COLON            = r':'
t_DBLCOLON         = r'::'
t_ASTERISK	   = r'\*'

# Identifiers and reserved words
reserved_map = { }
for r in reserved:
    reserved_map[r.lower()] = r

def t_ID(t):
    r'[A-Za-z_]\w*'
    t.type = reserved_map.get(t.value,'ID')
    return t

# Integer literal
def t_INTLIT(t):
    r'(0x[\da-fA-F]+)|\d+'
    try:
        t.value = int(t.value,0)
    except ValueError:
        error(t.lineno, 'Integer value "%s" too large' % t.value)
        t.value = 0
    return t

# String literal.  Note that these use only single quotes, and
# can span multiple lines.
def t_STRLIT(t):
    r"(?m)'([^'])+'"
    # strip off quotes
    t.value = t.value[1:-1]
    t.lineno += t.value.count('\n')
    return t


# "Code literal"... like a string literal, but delimiters are
# '{{' and '}}' so they get formatted nicely under emacs c-mode
def t_CODELIT(t):
    r"(?m)\{\{([^\}]|}(?!\}))+\}\}"
    # strip off {{ & }}
    t.value = t.value[2:-2]
    t.lineno += t.value.count('\n')
    return t

def t_CPPDIRECTIVE(t):
    r'^\#.*\n'
    t.lineno += t.value.count('\n')
    return t

#
# The functions t_NEWLINE, t_ignore, and t_error are
# special for the lex module.
#

# Newlines
def t_NEWLINE(t):
    r'\n+'
    t.lineno += t.value.count('\n')

# Comments
def t_comment(t):
    r'//.*'

# Completely ignored characters
t_ignore           = ' \t\x0c'

# Error handler
def t_error(t):
    error(t.lineno, "illegal character '%s'" % t.value[0])
    t.skip(1)

# Build the lexer
lex.lex()

#####################################################################
#
#                                Parser
#
# Every function whose name starts with 'p_' defines a grammar rule.
# The rule is encoded in the function's doc string, while the
# function body provides the action taken when the rule is matched.
# The argument to each function is a list of the values of the
# rule's symbols: t[0] for the LHS, and t[1..n] for the symbols
# on the RHS.  For tokens, the value is copied from the t.value
# attribute provided by the lexer.  For non-terminals, the value
# is assigned by the producing rule; i.e., the job of the grammar
# rule function is to set the value for the non-terminal on the LHS
# (by assigning to t[0]).
#####################################################################

# Not sure why, but we get a handful of shift/reduce conflicts on DECLARE.
# By default these get resolved as shifts, which is correct, but
# warnings are printed.  Explicitly marking DECLARE as right-associative
# suppresses the warnings.
precedence = (
    ('right', 'DECLARE'),
    )

# The LHS of the first grammar rule is used as the start symbol
# (in this case, 'specification').  Note that this rule enforces
# that there will be exactly one namespace declaration, with 0 or more
# global defs/decls before and after it.  The defs & decls before
# the namespace decl will be outside the namespace; those after
# will be inside.  The decoder function is always inside the namespace.
def p_specification(t):
    'specification : opt_defs_and_declares name_decl opt_defs_and_declares decode_block'
    global_decls1 = t[1]
    isa_name = t[2]
    namespace = isa_name + "Inst"
    global_decls2 = t[3]
    (inst_decls, code) = t[4]
    code = indent(code)
    # grab the last three path components of isa_desc_filename
    filename = '/'.join(isa_desc_filename.split('/')[-3:])
    # if the isa_desc file defines a 'rcs_id' string,
    # echo that into the output too
    try:
        local_rcs_id = rcs_id
        # strip $s out of ID so it doesn't get re-substituted
        local_rcs_id = re.sub(r'\$', '', local_rcs_id)
    except NameError:
        local_rcs_id = 'Id: no RCS id found'
    output = open(decoder_filename, 'w')
    # split string to keep rcs from substituting this file's RCS id in
    print >> output, '/* $Id' + '''$ */

/*
 * Copyright (c) 2003
 * The Regents of The University of Michigan
 * All Rights Reserved
 *
 * This code is part of the M5 simulator, developed by Nathan Binkert,
 * Erik Hallnor, Steve Raasch, and Steve Reinhardt, with contributions
 * from Ron Dreslinski, Dave Greene, and Lisa Hsu.
 *
 * Permission is granted to use, copy, create derivative works and
 * redistribute this software and such derivative works for any
 * purpose, so long as the copyright notice above, this grant of
 * permission, and the disclaimer below appear in all copies made; and
 * so long as the name of The University of Michigan is not used in
 * any advertising or publicity pertaining to the use or distribution
 * of this software without specific, written prior authorization.
 *
 * THIS SOFTWARE IS PROVIDED AS IS, WITHOUT REPRESENTATION FROM THE
 * UNIVERSITY OF MICHIGAN AS TO ITS FITNESS FOR ANY PURPOSE, AND
 * WITHOUT WARRANTY BY THE UNIVERSITY OF MICHIGAN OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE. THE REGENTS OF THE UNIVERSITY OF MICHIGAN SHALL NOT BE
 * LIABLE FOR ANY DAMAGES, INCLUDING DIRECT, SPECIAL, INDIRECT,
 * INCIDENTAL, OR CONSEQUENTIAL DAMAGES, WITH RESPECT TO ANY CLAIM
 * ARISING OUT OF OR IN CONNECTION WITH THE USE OF THE SOFTWARE, EVEN
 * IF IT HAS BEEN OR IS HEREAFTER ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGES.
 */

/*
 * DO NOT EDIT THIS FILE!!!
 *
 * It was automatically generated from this ISA description:
 *  Filename: %(filename)s
 *    RCS %(local_rcs_id)s
 */

#include "base/bitfield.hh"	// required for bitfield support


/////////////////////////////////////
// Global defs (outside namespace) //
/////////////////////////////////////

%(global_decls1)s

/**
 * Namespace for %(isa_name)s static instruction objects.
 */
namespace %(namespace)s
{

/////////////////////////////////////
// Global defs  (within namespace) //
/////////////////////////////////////

%(global_decls2)s

////////////////////////////////////
// Declares from inst definitions //
////////////////////////////////////

%(inst_decls)s

} // namespace %(namespace)s

//////////////////////
// Decoder function //
//////////////////////

StaticInstPtr<%(isa_name)s>
%(isa_name)s::decodeInst(%(isa_name)s::MachInst machInst)
{
    using namespace %(namespace)s;
%(code)s
} // decodeInst
''' % vars()
    output.close()

# ISA name declaration looks like "namespace <foo>;"
def p_name_decl(t):
    'name_decl : NAMESPACE ID SEMI'
    t[0] = t[2]

# 'opt_defs_and_declares' is a possibly empty sequence of
# defs and/or declares.
def p_opt_defs_and_declares_0(t):
    'opt_defs_and_declares : empty'
    t[0] = ''

def p_opt_defs_and_declares_1(t):
    'opt_defs_and_declares : defs_and_declares'
    t[0] = t[1]

def p_defs_and_declares_0(t):
    'defs_and_declares : def_or_declare'
    t[0] = t[1]

def p_defs_and_declares_1(t):
    'defs_and_declares : defs_and_declares def_or_declare'
    t[0] = t[1] + t[2]

# The list of possible definition/declaration statements.
def p_def_or_declare(t):
    '''def_or_declare : def_format
                      | def_bitfield
                      | def_template
                      | global_declare
                      | global_let
                      | cpp_directive'''
    t[0] = t[1]

# preprocessor directives are copied directly to the output.
def p_cpp_directive(t):
    '''cpp_directive : CPPDIRECTIVE'''
    t[0] = t[1]

# Global declares 'declare {{...}}' (C++ code blocks) are copied
# directly to the output.
def p_global_declare(t):
    'global_declare : DECLARE CODELIT SEMI'
    t[0] = substBitOps(t[2])

# global let blocks 'let {{...}}' (Python code blocks) are executed
# directly when seen.  These are typically used to initialize global
# Python variables used in later format definitions.
def p_global_let(t):
    'global_let : LET CODELIT SEMI'
    try:
        exec(fixPythonIndentation(t[2]))
    except:
        error_bt(t.lineno(1), 'error in global let block "%s".' % t[2])
    t[0] = '' # contributes nothing to the output C++ file

# A bitfield definition looks like:
# 'def [signed] bitfield <ID> [<first>:<last>]'
# This generates a preprocessor macro in the output file.
def p_def_bitfield_0(t):
    'def_bitfield : DEF opt_signed BITFIELD ID LESS INTLIT COLON INTLIT GREATER SEMI'
    expr = 'bits(machInst, %2d, %2d)' % (t[6], t[8])
    if (t[2] == 'signed'):
        expr = 'sext<%d>(%s)' % (t[6] - t[8] + 1, expr)
    t[0] = '#undef %s\n#define %s\t%s\n' % (t[4], t[4], expr)

# alternate form for single bit: 'def [signed] bitfield <ID> [<bit>]'
def p_def_bitfield_1(t):
    'def_bitfield : DEF opt_signed BITFIELD ID LESS INTLIT GREATER SEMI'
    expr = 'bits(machInst, %2d, %2d)' % (t[6], t[6])
    if (t[2] == 'signed'):
        expr = 'sext<%d>(%s)' % (1, expr)
    t[0] = '#undef %s\n#define %s\t%s\n' % (t[4], t[4], expr)

def p_opt_signed_0(t):
    'opt_signed : SIGNED'
    t[0] = t[1]

def p_opt_signed_1(t):
    'opt_signed : empty'
    t[0] = ''

# Global map variable to hold templates
templateMap = {}

def p_def_template(t):
    'def_template : DEF TEMPLATE ID CODELIT SEMI'
    templateMap[t[3]] = t[4]
    t[0] = ''

# An instruction format definition looks like
# "def format <fmt>(<params>) {{...}};"
def p_def_format(t):
    'def_format : DEF FORMAT ID LPAREN param_list RPAREN CODELIT SEMI'
    (id, params, code) = (t[3], t[5], t[7])
    defFormat(id, params, code, t.lineno(1))
    # insert a comment into the output to note that the def was processed
    t[0] = '''
//
// parser: format %s defined
//
''' % id

# The formal parameter list for an instruction format is a possibly
# empty list of comma-separated parameters.
def p_param_list_0(t):
    'param_list : empty'
    t[0] = [ ]

def p_param_list_1(t):
    'param_list : param'
    t[0] = [t[1]]

def p_param_list_2(t):
    'param_list : param_list COMMA param'
    t[0] = t[1]
    t[0].append(t[3])

# Each formal parameter is either an identifier or an identifier
# preceded by an asterisk.  As in Python, the latter (if present) gets
# a tuple containing all the excess positional arguments, allowing
# varargs functions.
def p_param_0(t):
    'param : ID'
    t[0] = t[1]

def p_param_1(t):
    'param : ASTERISK ID'
    # just concatenate them: '*ID'
    t[0] = t[1] + t[2]

# End of format definition-related rules.
##############

#
# A decode block looks like:
#	decode <field1> [, <field2>]* [default <inst>] { ... }
#
def p_decode_block(t):
    'decode_block : DECODE ID opt_default LBRACE decode_stmt_list RBRACE'
    default_defaults = defaultStack.pop()
    (decls, code, has_default) = t[5]
    # use the "default defaults" only if there was no explicit
    # default statement in decode_stmt_list
    if not has_default:
        (default_decls, default_code) = default_defaults
        decls += default_decls
        code += default_code
    t[0] = (decls,  '''
switch (%s) {
%s
}
''' % (t[2], indent(code)))

# The opt_default statement serves only to push the "default defaults"
# onto defaultStack.  This value will be used by nested decode blocks,
# and used and popped off when the current decode_block is processed
# (in p_decode_block() above).
def p_opt_default_0(t):
    'opt_default : empty'
    # no default specified: reuse the one currently at the top of the stack
    defaultStack.push(defaultStack.top())
    # no meaningful value returned
    t[0] = None

def p_opt_default_1(t):
    'opt_default : DEFAULT inst'
    # push the new default
    (decls, code) = t[2]
    defaultStack.push((decls, '\ndefault:\n%sbreak;' % code))
    # no meaningful value returned
    t[0] = None

def p_decode_stmt_list_0(t):
    'decode_stmt_list : decode_stmt'
    t[0] = t[1]

def p_decode_stmt_list_1(t):
    'decode_stmt_list : decode_stmt decode_stmt_list'
    (decls1, code1, has_default1) = t[1]
    (decls2, code2, has_default2) = t[2]
    if (has_default1 and has_default2):
        error(t.lineno(1), 'Two default cases in decode block')
    t[0] = (decls1 + '\n' + decls2, code1 + '\n' + code2,
            has_default1 or has_default2)

#
# Decode statement rules
#
# There are four types of statements allowed in a decode block:
# 1. Format blocks 'format <foo> { ... }'
# 2. Nested decode blocks
# 3. Instruction definitions.
# 4. C preprocessor directives.


# Preprocessor directives found in a decode statement list are passed
# through to the output, replicated to both the declaration and decode
# streams.  This works well for ifdefs, so we can ifdef out both the
# declarations and the decode cases generated by an instruction
# definition.  Handling them as part of the grammar makes it easy to
# keep them in the right place with respect to the code generated by
# the other statements.
def p_decode_stmt_cpp(t):
    'decode_stmt : CPPDIRECTIVE'
    t[0] = (t[1], t[1], 0)

# A format block 'format <foo> { ... }' sets the default instruction
# format used to handle instruction definitions inside the block.
# This format can be overridden by using an explicit format on the
# instruction definition or with a nested format block.
def p_decode_stmt_format(t):
    'decode_stmt : FORMAT push_format_id LBRACE decode_stmt_list RBRACE'
    # The format will be pushed on the stack when 'push_format_id' is
    # processed (see below).  Once the parser has recognized the full
    # production (though the right brace), we're done with the format,
    # so now we can pop it.
    formatStack.pop()
    t[0] = t[4]

# This rule exists so we can set the current format (& push the stack)
# when we recognize the format name part of the format block.
def p_push_format_id(t):
    'push_format_id : ID'
    try:
        formatStack.push(formatMap[t[1]])
        t[0] = ('', '// format %s' % t[1])
    except KeyError:
        error(t.lineno(1), 'instruction format "%s" not defined.' % t[1])

# Nested decode block: if the value of the current field matches the
# specified constant, do a nested decode on some other field.
def p_decode_stmt_decode(t):
    'decode_stmt : case_label COLON decode_block'
    (label, is_default) = t[1]
    (decls, code) = t[3]
    # just wrap the decoding code from the block as a case in the
    # outer switch statement.
    t[0] = (decls, '\n%s:\n%s' % (label, indent(code)), is_default)

# Instruction definition (finally!).
def p_decode_stmt_inst(t):
    'decode_stmt : case_label COLON inst SEMI'
    (label, is_default) = t[1]
    (decls, code) = t[3]
    t[0] = (decls, '\n%s:%sbreak;' % (label, indent(code)), is_default)

# The case label is either a list of one or more constants or 'default'
def p_case_label_0(t):
    'case_label : intlit_list'
    t[0] = (': '.join(map(lambda a: 'case %#x' % a, t[1])), 0)

def p_case_label_1(t):
    'case_label : DEFAULT'
    t[0] = ('default', 1)

#
# The constant list for a decode case label must be non-empty, but may have
# one or more comma-separated integer literals in it.
#
def p_intlit_list_0(t):
    'intlit_list : INTLIT'
    t[0] = [t[1]]

def p_intlit_list_1(t):
    'intlit_list : intlit_list COMMA INTLIT'
    t[0] = t[1]
    t[0].append(t[3])

# Define an instruction using the current instruction format (specified
# by an enclosing format block).
# "<mnemonic>(<args>)"
def p_inst_0(t):
    'inst : ID LPAREN arg_list RPAREN'
    # Pass the ID and arg list to the current format class to deal with.
    currentFormat = formatStack.top()
    (decls, code) = currentFormat.defineInst(t[1], t[3], t.lineno(1))
    args = ','.join(map(str, t[3]))
    args = re.sub('(?m)^', '//', args)
    args = re.sub('^//', '', args)
    comment = '// %s::%s(%s)\n' % (currentFormat.id, t[1], args)
    t[0] = (comment + decls, comment + code)

# Define an instruction using an explicitly specified format:
# "<fmt>::<mnemonic>(<args>)"
def p_inst_1(t):
    'inst : ID DBLCOLON ID LPAREN arg_list RPAREN'
    try:
        format = formatMap[t[1]]
    except KeyError:
        error(t.lineno(1), 'instruction format "%s" not defined.' % t[1])
    (decls, code) = format.defineInst(t[3], t[5], t.lineno(1))
    comment = '// %s::%s(%s)\n' % (t[1], t[3], t[5])
    t[0] = (comment + decls, comment + code)

def p_arg_list_0(t):
    'arg_list : empty'
    t[0] = [ ]

def p_arg_list_1(t):
    'arg_list : arg'
    t[0] = [t[1]]

def p_arg_list_2(t):
    'arg_list : arg_list COMMA arg'
    t[0] = t[1]
    t[0].append(t[3])

def p_arg(t):
    '''arg : ID
           | INTLIT
           | STRLIT
           | CODELIT'''
    t[0] = t[1]

#
# Empty production... use in other rules for readability.
#
def p_empty(t):
    'empty :'
    pass

# Parse error handler.  Note that the argument here is the offending
# *token*, not a grammar symbol (hence the need to use t.value)
def p_error(t):
    if t:
        error(t.lineno, "syntax error at '%s'" % t.value)
    else:
        error_bt(0, "unknown syntax error")

# END OF GRAMMAR RULES
#
# Now build the parser.
yacc.yacc()

################
# Format object.
#
# A format object encapsulates an instruction format.  It must provide
# a defineInst() method that generates the code for an instruction
# definition.

class Format:
    def __init__(self, id, params, code):
        # constructor: just save away arguments
        self.id = id
        self.params = params
        # strip blank lines from code (ones at the end are troublesome)
        code = re.sub(r'(?m)^\s*$', '', code);
        if code == '':
            code = '    pass\n'
        param_list = string.join(params, ", ")
        f = 'def defInst(name, Name, ' + param_list + '):\n' + code
        exec(f)
        self.func = defInst

    def defineInst(self, name, args, lineno):
        # automatically provide a capitalized version of mnemonic
        Name = string.capitalize(name)
        try:
            retval = self.func(name, Name, *args)
        except:
            error_bt(lineno, 'error defining "%s".' % name)
        return retval

# Special null format to catch an implicit-format instruction
# definition outside of any format block.
class NoFormat:
    def __init__(self):
        self.defaultInst = ''

    def defineInst(self, name, args, lineno):
        error(lineno,
              'instruction definition "%s" with no active format!' % name)

# This dictionary maps format name strings to Format objects.
formatMap = {}

# Define a new format
def defFormat(id, params, code, lineno):
    # make sure we haven't already defined this one
    if formatMap.get(id, None) != None:
        error(lineno, 'format %s redefined.' % id)
    # create new object and store in global map
    formatMap[id] = Format(id, params, code)


##############
# Stack: a simple stack object.  Used for both formats (formatStack)
# and default cases (defaultStack).

class Stack:
    def __init__(self, initItem):
        self.stack = [ initItem ]

    def push(self, item):
        self.stack.append(item);

    def pop(self):
        return self.stack.pop()

    def top(self):
        return self.stack[-1]

# The global format stack.
formatStack = Stack(NoFormat())

# The global default case stack.
defaultStack = Stack( None )

###################
# Utility functions

#
# Indent every line in string 's' by two spaces
# (except preprocessor directives).
# Used to make nested code blocks look pretty.
#
def indent(s):
    return re.sub(r'(?m)^(?!\#)', '  ', s)

#
# Munge a somewhat arbitrarily formatted piece of Python code
# (e.g. from a format 'let' block) into something whose indentation
# will get by the Python parser.
#
# The two keys here are that Python will give a syntax error if
# there's any whitespace at the beginning of the first line, and that
# all lines at the same lexical nesting level must have identical
# indentation.  Unfortunately the way code literals work, an entire
# let block tends to have some initial indentation.  Rather than
# trying to figure out what that is and strip it off, we prepend 'if
# 1:' to make the let code the nested block inside the if (and have
# the parser automatically deal with the indentation for us).
#
# We don't want to do this if (1) the code block is empty or (2) the
# first line of the block doesn't have any whitespace at the front.

def fixPythonIndentation(s):
    # get rid of blank lines first
    s = re.sub(r'(?m)^\s*\n', '', s);
    if (s != '' and re.match(r'[ \t]', s[0])):
        s = 'if 1:\n' + s
    return s

# Error handler.  Just call exit.  Output formatted to work under
# Emacs compile-mode.
def error(lineno, string):
    sys.exit("%s:%d: %s" % (isa_desc_filename, lineno, string))

# Like error(), but include a Python stack backtrace (for processing
# Python exceptions).
def error_bt(lineno, string):
    print >> sys.stderr, "%s:%d: %s" % (isa_desc_filename, lineno, string)
    raise


#####################################################################
#
#                      Bitfield Operator Support
#
#####################################################################

bitOp1ArgRE = re.compile(r'<\s*(\w+)\s*:\s*>')

bitOpWordRE = re.compile(r'(?<![\w\.])([\w\.]+)<\s*(\w+)\s*:\s*(\w+)\s*>')
bitOpExprRE = re.compile(r'\)<\s*(\w+)\s*:\s*(\w+)\s*>')

def substBitOps(code):
    # first convert single-bit selectors to two-index form
    # i.e., <n> --> <n:n>
    code = bitOp1ArgRE.sub(r'<\1:\1>', code)
    # simple case: selector applied to ID (name)
    # i.e., foo<a:b> --> bits(foo, a, b)
    code = bitOpWordRE.sub(r'bits(\1, \2, \3)', code)
    # if selector is applied to expression (ending in ')'),
    # we need to search backward for matching '('
    match = bitOpExprRE.search(code)
    while match:
        exprEnd = match.start()
        here = exprEnd - 1
        nestLevel = 1
        while nestLevel > 0:
            if code[here] == '(':
                nestLevel -= 1
            elif code[here] == ')':
                nestLevel += 1
            here -= 1
            if here < 0:
                sys.exit("Didn't find '('!")
        exprStart = here+1
        newExpr = r'bits(%s, %s, %s)' % (code[exprStart:exprEnd+1],
                                         match.group(1), match.group(2))
        code = code[:exprStart] + newExpr + code[match.end():]
        match = bitOpExprRE.search(code)
    return code


#####################################################################
#
#                             Code Parser
#
# The remaining code is the support for automatically extracting
# instruction characteristics from pseudocode.
#
#####################################################################

# Force the argument to be a list
def makeList(list_or_item):
    if not list_or_item:
        return []
    elif type(list_or_item) == ListType:
        return list_or_item
    else:
        return [ list_or_item ]

# generate operandSizeMap based on provided operandTypeMap:
# basically generate equiv. C++ type and make is_signed flag
def buildOperandSizeMap():
    global operandSizeMap
    operandSizeMap = {}
    for ext in operandTypeMap.keys():
        (desc, size) = operandTypeMap[ext]
        if desc == 'signed int':
            type = 'int%d_t' % size
            is_signed = 1
        elif desc == 'unsigned int':
            type = 'uint%d_t' % size
            is_signed = 0
        elif desc == 'float':
            is_signed = 1	# shouldn't really matter
            if size == 32:
                type = 'float'
            elif size == 64:
                type = 'double'
        if type == '':
            error(0, 'Unrecognized type description "%s" in operandTypeMap')
        operandSizeMap[ext] = (size, type, is_signed)

#
# Base class for operand traits.  An instance of this class (or actually
# a class derived from this one) encapsulates the traits of a particular
# operand type (e.g., "32-bit integer register").
#
class OperandTraits:
    def __init__(self, dflt_ext, reg_spec, flags, sort_pri):
        # Force construction of operandSizeMap from operandTypeMap
        # if it hasn't happened yet
        if not globals().has_key('operandSizeMap'):
            buildOperandSizeMap()
        self.dflt_ext = dflt_ext
        (self.dflt_size, self.dflt_type, self.dflt_is_signed) = \
                         operandSizeMap[dflt_ext]
        self.reg_spec = reg_spec
        # Canonical flag structure is a triple of lists, where each list
        # indicates the set of flags implied by this operand always, when
        # used as a source, and when used as a dest, respectively.
        # For simplicity this can be initialized using a variety of fairly
        # obvious shortcuts; we convert these to canonical form here.
        if not flags:
            # no flags specified (e.g., 'None')
            self.flags = ( [], [], [] )
        elif type(flags) == StringType:
            # a single flag: assumed to be unconditional
            self.flags = ( [ flags ], [], [] )
        elif type(flags) == ListType:
            # a list of flags: also assumed to be unconditional
            self.flags = ( flags, [], [] )
        elif type(flags) == TupleType:
            # it's a tuple: it should be a triple,
            # but each item could be a single string or a list
            (uncond_flags, src_flags, dest_flags) = flags
            self.flags = (makeList(uncond_flags),
                          makeList(src_flags), makeList(dest_flags))
        self.sort_pri = sort_pri

    def isMem(self):
        return 0

    def isReg(self):
        return 0

    def isFloatReg(self):
        return 0

    def isIntReg(self):
        return 0

    def isControlReg(self):
        return 0

    def getFlags(self, op_desc):
        # note the empty slice '[:]' gives us a copy of self.flags[0]
        # instead of a reference to it
        my_flags = self.flags[0][:]
        if op_desc.is_src:
            my_flags += self.flags[1]
        if op_desc.is_dest:
            my_flags += self.flags[2]
        return my_flags

    def makeDecl(self, op_desc):
        (size, type, is_signed) = operandSizeMap[op_desc.eff_ext]
        # Note that initializations in the declarations are solely
        # to avoid 'uninitialized variable' errors from the compiler.
        return type + ' ' + op_desc.munged_name + ' = 0;\n';

class IntRegOperandTraits(OperandTraits):
    def isReg(self):
        return 1

    def isIntReg(self):
        return 1

    def makeConstructor(self, op_desc):
        c = ''
        if op_desc.is_src:
            c += '\n\t_srcRegIdx[%d] = %s;' % \
                 (op_desc.src_reg_idx, self.reg_spec)
        if op_desc.is_dest:
            c += '\n\t_destRegIdx[%d] = %s;' % \
                 (op_desc.dest_reg_idx, self.reg_spec)
        return c

    def makeRead(self, op_desc, cpu_model):
        (size, type, is_signed) = operandSizeMap[op_desc.eff_ext]
        if (type == 'float' or type == 'double'):
            error(0, 'Attempt to read integer register as FP')
        if (size == self.dflt_size):
            return '%s = xc->readIntReg(_srcRegIdx[%d]);\n' % \
                   (op_desc.munged_name, op_desc.src_reg_idx)
        else:
            return '%s = bits(xc->readIntReg(_srcRegIdx[%d]), %d, 0);\n' % \
                   (op_desc.munged_name, op_desc.src_reg_idx, size-1)

    def makeWrite(self, op_desc, cpu_model):
        (size, type, is_signed) = operandSizeMap[op_desc.eff_ext]
        if (type == 'float' or type == 'double'):
            error(0, 'Attempt to write integer register as FP')
        if (size != self.dflt_size and is_signed):
            final_val = 'sext<%d>(%s)' % (size, op_desc.munged_name)
        else:
            final_val = op_desc.munged_name
        wb = '''
        {
            %s final_val = %s;
            xc->setIntReg(_destRegIdx[%d], final_val);\n
            if (traceData) { traceData->setData(final_val); }
        }''' % (self.dflt_type, final_val, op_desc.dest_reg_idx)
        return wb

class FloatRegOperandTraits(OperandTraits):
    def isReg(self):
        return 1

    def isFloatReg(self):
        return 1

    def makeConstructor(self, op_desc):
        c = ''
        if op_desc.is_src:
            c += '\n\t_srcRegIdx[%d] = %s + FP_Base_DepTag;' % \
                 (op_desc.src_reg_idx, self.reg_spec)
        if op_desc.is_dest:
            c += '\n\t_destRegIdx[%d] = %s + FP_Base_DepTag;' % \
                 (op_desc.dest_reg_idx, self.reg_spec)
        return c

    def makeRead(self, op_desc, cpu_model):
        (size, type, is_signed) = operandSizeMap[op_desc.eff_ext]
        bit_select = 0
        if (type == 'float'):
            func = 'readFloatRegSingle'
        elif (type == 'double'):
            func = 'readFloatRegDouble'
        else:
            func = 'readFloatRegInt'
            if (size != self.dflt_size):
                bit_select = 1
        base = 'xc->%s(_srcRegIdx[%d] - FP_Base_DepTag)' % \
               (func, op_desc.src_reg_idx)
        if bit_select:
            return '%s = bits(%s, %d, 0);\n' % \
                   (op_desc.munged_name, base, size-1)
        else:
            return '%s = %s;\n' % (op_desc.munged_name, base)

    def makeWrite(self, op_desc, cpu_model):
        (size, type, is_signed) = operandSizeMap[op_desc.eff_ext]
        final_val = op_desc.munged_name
        if (type == 'float'):
            func = 'setFloatRegSingle'
        elif (type == 'double'):
            func = 'setFloatRegDouble'
        else:
            func = 'setFloatRegInt'
            type = 'uint%d_t' % self.dflt_size
            if (size != self.dflt_size and is_signed):
                final_val = 'sext<%d>(%s)' % (size, op_desc.munged_name)
        wb = '''
        {
            %s final_val = %s;
            xc->%s(_destRegIdx[%d] - FP_Base_DepTag, final_val);\n
            if (traceData) { traceData->setData(final_val); }
        }''' % (type, final_val, func, op_desc.dest_reg_idx)
        return wb

class ControlRegOperandTraits(OperandTraits):
    def isReg(self):
        return 1

    def isControlReg(self):
        return 1

    def makeConstructor(self, op_desc):
        c = ''
        if op_desc.is_src:
            c += '\n\t_srcRegIdx[%d] = %s_DepTag;' % \
                 (op_desc.src_reg_idx, self.reg_spec)
        if op_desc.is_dest:
            c += '\n\t_destRegIdx[%d] = %s_DepTag;' % \
                 (op_desc.dest_reg_idx, self.reg_spec)
        return c

    def makeRead(self, op_desc, cpu_model):
        (size, type, is_signed) = operandSizeMap[op_desc.eff_ext]
        bit_select = 0
        if (type == 'float' or type == 'double'):
            error(0, 'Attempt to read control register as FP')
        base = 'xc->read%s()' % self.reg_spec
        if size == self.dflt_size:
            return '%s = %s;\n' % (op_desc.munged_name, base)
        else:
            return '%s = bits(%s, %d, 0);\n' % \
                   (op_desc.munged_name, base, size-1)

    def makeWrite(self, op_desc, cpu_model):
        (size, type, is_signed) = operandSizeMap[op_desc.eff_ext]
        if (type == 'float' or type == 'double'):
            error(0, 'Attempt to write control register as FP')
        wb = 'xc->set%s(%s);\n' % (self.reg_spec, op_desc.munged_name)
        wb += 'if (traceData) { traceData->setData(%s); }' % \
              op_desc.munged_name
        return wb

class MemOperandTraits(OperandTraits):
    def isMem(self):
        return 1

    def makeConstructor(self, op_desc):
        return ''

    def makeDecl(self, op_desc):
        (size, type, is_signed) = operandSizeMap[op_desc.eff_ext]
        # Note that initializations in the declarations are solely
        # to avoid 'uninitialized variable' errors from the compiler.
        # Declare memory data variable.
        c = '%s %s = 0;\n' % (type, op_desc.munged_name)
        # Declare var to hold memory access flags.
        c += 'unsigned %s_flags = memAccessFlags;\n' % op_desc.base_name
        # If this operand is a dest (i.e., it's a store operation),
        # then we need to declare a variable for the write result code
        # as well.
        if op_desc.is_dest:
            c += 'uint64_t %s_write_result = 0;\n' % op_desc.base_name
        return c

    def makeRead(self, op_desc, cpu_model):
        (size, type, is_signed) = operandSizeMap[op_desc.eff_ext]
        eff_type = 'uint%d_t' % size
        return 'fault = memAccessObj->read(EA, (%s&)%s, %s_flags);\n' \
               % (eff_type, op_desc.munged_name, op_desc.base_name)

    def makeWrite(self, op_desc, cpu_model):
        (size, type, is_signed) = operandSizeMap[op_desc.eff_ext]
        eff_type = 'uint%d_t' % size
        return 'fault = memAccessObj->write((%s&)%s, EA, %s_flags,' \
               ' &%s_write_result);\n' \
               % (eff_type, op_desc.munged_name, op_desc.base_name,
                  op_desc.base_name)

class NPCOperandTraits(OperandTraits):
    def makeConstructor(self, op_desc):
        return ''

    def makeRead(self, op_desc, cpu_model):
        return '%s = xc->readPC() + 4;\n' % op_desc.munged_name

    def makeWrite(self, op_desc, cpu_model):
        return 'xc->setNextPC(%s);\n' % op_desc.munged_name


#
# Define operand variables that get derived from the basic declaration
# of ISA-specific operands in operandTraitsMap.  This function must be
# called by the ISA description file explicitly after defining
# operandTraitsMap (in a 'let' block).
#
def defineDerivedOperandVars():
    global operands
    operands = operandTraitsMap.keys()

    operandsREString = (r'''
    (?<![\w\.])	     # neg. lookbehind assertion: prevent partial matches
    ((%s)(?:\.(\w+))?)   # match: operand with optional '.' then suffix
    (?![\w\.])	     # neg. lookahead assertion: prevent partial matches
    '''
                        % string.join(operands, '|'))

    global operandsRE
    operandsRE = re.compile(operandsREString, re.MULTILINE|re.VERBOSE)

    # Same as operandsREString, but extension is mandatory, and only two
    # groups are returned (base and ext, not full name as above).
    # Used for subtituting '_' for '.' to make C++ identifiers.
    operandsWithExtREString = (r'(?<![\w\.])(%s)\.(\w+)(?![\w\.])'
                               % string.join(operands, '|'))

    global operandsWithExtRE
    operandsWithExtRE = re.compile(operandsWithExtREString, re.MULTILINE)


#
# Operand descriptor class.  An instance of this class represents
# a specific operand for a code block.
#
class OperandDescriptor:
    def __init__(self, full_name, base_name, ext, is_src, is_dest):
        self.full_name = full_name
        self.base_name = base_name
        self.ext = ext
        self.is_src = is_src
        self.is_dest = is_dest
        self.traits = operandTraitsMap[base_name]
        # The 'effective extension' (eff_ext) is either the actual
        # extension, if one was explicitly provided, or the default.
        # The 'munged name' replaces the '.' between the base and
        # extension (if any) with a '_' to make a legal C++ variable name.
        if ext:
            self.eff_ext = ext
            self.munged_name = base_name + '_' + ext
        else:
            self.eff_ext = self.traits.dflt_ext
            self.munged_name = base_name

    # Finalize additional fields (primarily code fields).  This step
    # is done separately since some of these fields may depend on the
    # register index enumeration that hasn't been performed yet at the
    # time of __init__().
    def finalize(self):
        self.flags = self.traits.getFlags(self)
        self.constructor = self.traits.makeConstructor(self)
        self.exec_decl = self.traits.makeDecl(self)

        if self.is_src:
            self.simple_rd = self.traits.makeRead(self, 'simple')
            self.dtld_rd = self.traits.makeRead(self, 'dtld')
        else:
            self.simple_rd = ''
            self.dtld_rd = ''

        if self.is_dest:
            self.simple_wb = self.traits.makeWrite(self, 'simple')
            self.dtld_wb = self.traits.makeWrite(self, 'dtld')
        else:
            self.simple_wb = ''
            self.dtld_wb = ''

class OperandDescriptorList:
    def __init__(self):
        self.items = []
        self.bases = {}

    def __len__(self):
        return len(self.items)

    def __getitem__(self, index):
        return self.items[index]

    def append(self, op_desc):
        self.items.append(op_desc)
        self.bases[op_desc.base_name] = op_desc

    def find_base(self, base_name):
        # like self.bases[base_name], but returns None if not found
        # (rather than raising exception)
        return self.bases.get(base_name)

    # internal helper function for concat[Some]Attr{Strings|Lists}
    def __internalConcatAttrs(self, attr_name, filter, result):
        for op_desc in self.items:
            if filter(op_desc):
                result += getattr(op_desc, attr_name)
        return result

    # return a single string that is the concatenation of the (string)
    # values of the specified attribute for all operands
    def concatAttrStrings(self, attr_name):
        return self.__internalConcatAttrs(attr_name, lambda x: 1, '')

    # like concatAttrStrings, but only include the values for the operands
    # for which the provided filter function returns true
    def concatSomeAttrStrings(self, filter, attr_name):
        return self.__internalConcatAttrs(attr_name, filter, '')

    # return a single list that is the concatenation of the (list)
    # values of the specified attribute for all operands
    def concatAttrLists(self, attr_name):
        return self.__internalConcatAttrs(attr_name, lambda x: 1, [])

    # like concatAttrLists, but only include the values for the operands
    # for which the provided filter function returns true
    def concatSomeAttrLists(self, filter, attr_name):
        return self.__internalConcatAttrs(attr_name, filter, [])

    def sort(self):
        self.items.sort(lambda a, b: a.traits.sort_pri - b.traits.sort_pri)

# Regular expression object to match C++ comments
# (used in findOperands())
commentRE = re.compile(r'//.*\n')

# Regular expression object to match assignment statements
# (used in findOperands())
assignRE = re.compile(r'\s*=(?!=)', re.MULTILINE)

#
# Find all the operands in the given code block.  Returns an operand
# descriptor list (instance of class OperandDescriptorList).
#
def findOperands(code):
    operands = OperandDescriptorList()
    # delete comments so we don't accidentally match on reg specifiers inside
    code = commentRE.sub('', code)
    # search for operands
    next_pos = 0
    while 1:
        match = operandsRE.search(code, next_pos)
        if not match:
            # no more matches: we're done
            break
        op = match.groups()
        # regexp groups are operand full name, base, and extension
        (op_full, op_base, op_ext) = op
        # if the token following the operand is an assignment, this is
        # a destination (LHS), else it's a source (RHS)
        is_dest = (assignRE.match(code, match.end()) != None)
        is_src = not is_dest
        # see if we've already seen this one
        op_desc = operands.find_base(op_base)
        if op_desc:
            if op_desc.ext != op_ext:
                error(0, 'Inconsistent extensions for operand %s' % op_base)
            op_desc.is_src = op_desc.is_src or is_src
            op_desc.is_dest = op_desc.is_dest or is_dest
        else:
            # new operand: create new descriptor
            op_desc = OperandDescriptor(op_full, op_base, op_ext,
                                        is_src, is_dest)
            operands.append(op_desc)
        # start next search after end of current match
        next_pos = match.end()
    operands.sort()
    # enumerate source & dest register operands... used in building
    # constructor later
    srcRegs = 0
    destRegs = 0
    operands.numFPDestRegs = 0
    operands.numIntDestRegs = 0
    for op_desc in operands:
        if op_desc.traits.isReg():
            if op_desc.is_src:
                op_desc.src_reg_idx = srcRegs
                srcRegs += 1
            if op_desc.is_dest:
                op_desc.dest_reg_idx = destRegs
                destRegs += 1
                if op_desc.traits.isFloatReg():
                    operands.numFPDestRegs += 1
                elif op_desc.traits.isIntReg():
                    operands.numIntDestRegs += 1
    operands.numSrcRegs = srcRegs
    operands.numDestRegs = destRegs
    # now make a final pass to finalize op_desc fields that may depend
    # on the register enumeration
    for op_desc in operands:
        op_desc.finalize()
    return operands

# Munge operand names in code string to make legal C++ variable names.
# (Will match munged_name attribute of OperandDescriptor object.)
def substMungedOpNames(code):
    return operandsWithExtRE.sub(r'\1_\2', code)

def joinLists(t):
    return map(string.join, t)

def makeFlagConstructor(flag_list):
    if len(flag_list) == 0:
        return ''
    # filter out repeated flags
    flag_list.sort()
    i = 1
    while i < len(flag_list):
        if flag_list[i] == flag_list[i-1]:
            del flag_list[i]
        else:
            i += 1
    pre = '\n\tflags['
    post = '] = true;'
    code = pre + string.join(flag_list, post + pre) + post
    return code

class CodeBlock:
    def __init__(self, code):
        self.orig_code = code
        self.operands = findOperands(code)
        self.code = substMungedOpNames(substBitOps(code))
        self.constructor = self.operands.concatAttrStrings('constructor')
        self.constructor += \
                 '\n\t_numSrcRegs = %d;' % self.operands.numSrcRegs
        self.constructor += \
                 '\n\t_numDestRegs = %d;' % self.operands.numDestRegs
        self.constructor += \
                 '\n\t_numFPDestRegs = %d;' % self.operands.numFPDestRegs
        self.constructor += \
                 '\n\t_numIntDestRegs = %d;' % self.operands.numIntDestRegs

        self.exec_decl = self.operands.concatAttrStrings('exec_decl')

        is_mem = lambda op: op.traits.isMem()
        not_mem = lambda op: not op.traits.isMem()

        self.simple_rd = self.operands.concatAttrStrings('simple_rd')
        self.simple_wb = self.operands.concatAttrStrings('simple_wb')
        self.simple_mem_rd = \
                 self.operands.concatSomeAttrStrings(is_mem, 'simple_rd')
        self.simple_mem_wb = \
                 self.operands.concatSomeAttrStrings(is_mem, 'simple_wb')
        self.simple_nonmem_rd = \
                 self.operands.concatSomeAttrStrings(not_mem, 'simple_rd')
        self.simple_nonmem_wb = \
                 self.operands.concatSomeAttrStrings(not_mem, 'simple_wb')

        self.dtld_rd = self.operands.concatAttrStrings('dtld_rd')
        self.dtld_wb = self.operands.concatAttrStrings('dtld_wb')
        self.dtld_mem_rd = \
                 self.operands.concatSomeAttrStrings(is_mem, 'dtld_rd')
        self.dtld_mem_wb = \
                 self.operands.concatSomeAttrStrings(is_mem, 'dtld_wb')
        self.dtld_nonmem_rd = \
                 self.operands.concatSomeAttrStrings(not_mem, 'dtld_rd')
        self.dtld_nonmem_wb = \
                 self.operands.concatSomeAttrStrings(not_mem, 'dtld_wb')

        self.flags = self.operands.concatAttrLists('flags')

        # Make a basic guess on the operand class (function unit type).
        # These are good enough for most cases, and will be overridden
        # later otherwise.
        if 'IsStore' in self.flags:
            self.op_class = 'WrPort'
        elif 'IsLoad' in self.flags or 'IsPrefetch' in self.flags:
            self.op_class = 'RdPort'
        elif 'IsFloating' in self.flags:
            self.op_class = 'FloatADD'
        else:
            self.op_class = 'IntALU'

# Assume all instruction flags are of the form 'IsFoo'
instFlagRE = re.compile(r'Is.*')

# OpClass constants are just a little more complicated
opClassRE = re.compile(r'Int.*|Float.*|.*Port|No_OpClass')

class InstObjParams:
    def __init__(self, mnem, class_name, base_class = '',
                 code_block = None, opt_args = []):
        self.mnemonic = mnem
        self.class_name = class_name
        self.base_class = base_class
        if code_block:
            for code_attr in code_block.__dict__.keys():
                setattr(self, code_attr, getattr(code_block, code_attr))
        else:
            self.constructor = ''
            self.flags = []
        # Optional arguments are assumed to be either StaticInst flags
        # or an OpClass value.  To avoid having to import a complete
        # list of these values to match against, we do it ad-hoc
        # with regexps.
        for oa in opt_args:
            if instFlagRE.match(oa):
                self.flags.append(oa)
            elif opClassRE.match(oa):
                self.op_class = oa
            else:
                error(0, 'InstObjParams: optional arg "%s" not recognized '
                      'as StaticInst::Flag or OpClass.' % oa)

        # add flag initialization to contructor here to include
        # any flags added via opt_args
        self.constructor += makeFlagConstructor(self.flags)

        # if 'IsFloating' is set, add call to the FP enable check
        # function (which should be provided by isa_desc via a declare)
        if 'IsFloating' in self.flags:
            self.fp_enable_check = 'fault = checkFpEnableFault(xc);'
        else:
            self.fp_enable_check = ''

    def subst(self, *args):
        result = []
        for t in args:
            if not templateMap.has_key(t):
                error(0, 'InstObjParams::subst: undefined template "%s"' % t)
            try:
                result.append(templateMap[t] % self.__dict__)
            except KeyError, key:
                error(0, 'InstObjParams::subst: no definition for "%s"' % key)
        if len(args) == 1:
            result = result[0]
        return result

#
# All set... read in and parse the ISA description.
#
yacc.parse(isa_desc)
