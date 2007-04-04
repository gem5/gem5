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
# Authors: Steve Reinhardt
#          Korey Sewell

import os
import sys
import re
import string
import traceback
# get type names
from types import *

# Prepend the directory where the PLY lex & yacc modules are found
# to the search path.  Assumes we're compiling in a subdirectory
# of 'build' in the current tree.
sys.path[0:0] = [os.environ['M5_PLY']]

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
    'BITFIELD', 'DECODE', 'DECODER', 'DEFAULT', 'DEF', 'EXEC', 'FORMAT',
    'HEADER', 'LET', 'NAMESPACE', 'OPERAND_TYPES', 'OPERANDS',
    'OUTPUT', 'SIGNED', 'TEMPLATE'
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

    # ( ) [ ] { } < > , ; . : :: *
    'LPAREN', 'RPAREN',
    'LBRACKET', 'RBRACKET',
    'LBRACE', 'RBRACE',
    'LESS', 'GREATER', 'EQUALS',
    'COMMA', 'SEMI', 'DOT', 'COLON', 'DBLCOLON',
    'ASTERISK',

    # C preprocessor directives
    'CPPDIRECTIVE'

# The following are matched but never returned. commented out to
# suppress PLY warning
    # newfile directive
#    'NEWFILE',

    # endfile directive
#    'ENDFILE'
)

# Regular expressions for token matching
t_LPAREN           = r'\('
t_RPAREN           = r'\)'
t_LBRACKET         = r'\['
t_RBRACKET         = r'\]'
t_LBRACE           = r'\{'
t_RBRACE           = r'\}'
t_LESS             = r'\<'
t_GREATER          = r'\>'
t_EQUALS           = r'='
t_COMMA            = r','
t_SEMI             = r';'
t_DOT              = r'\.'
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
    r'^\#[^\#].*\n'
    t.lineno += t.value.count('\n')
    return t

def t_NEWFILE(t):
    r'^\#\#newfile\s+"[\w/.-]*"'
    fileNameStack.push((t.value[11:-1], t.lineno))
    t.lineno = 0

def t_ENDFILE(t):
    r'^\#\#endfile'
    (old_filename, t.lineno) = fileNameStack.pop()

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

# The LHS of the first grammar rule is used as the start symbol
# (in this case, 'specification').  Note that this rule enforces
# that there will be exactly one namespace declaration, with 0 or more
# global defs/decls before and after it.  The defs & decls before
# the namespace decl will be outside the namespace; those after
# will be inside.  The decoder function is always inside the namespace.
def p_specification(t):
    'specification : opt_defs_and_outputs name_decl opt_defs_and_outputs decode_block'
    global_code = t[1]
    isa_name = t[2]
    namespace = isa_name + "Inst"
    # wrap the decode block as a function definition
    t[4].wrap_decode_block('''
StaticInstPtr
%(isa_name)s::decodeInst(%(isa_name)s::ExtMachInst machInst)
{
    using namespace %(namespace)s;
''' % vars(), '}')
    # both the latter output blocks and the decode block are in the namespace
    namespace_code = t[3] + t[4]
    # pass it all back to the caller of yacc.parse()
    t[0] = (isa_name, namespace, global_code, namespace_code)

# ISA name declaration looks like "namespace <foo>;"
def p_name_decl(t):
    'name_decl : NAMESPACE ID SEMI'
    t[0] = t[2]

# 'opt_defs_and_outputs' is a possibly empty sequence of
# def and/or output statements.
def p_opt_defs_and_outputs_0(t):
    'opt_defs_and_outputs : empty'
    t[0] = GenCode()

def p_opt_defs_and_outputs_1(t):
    'opt_defs_and_outputs : defs_and_outputs'
    t[0] = t[1]

def p_defs_and_outputs_0(t):
    'defs_and_outputs : def_or_output'
    t[0] = t[1]

def p_defs_and_outputs_1(t):
    'defs_and_outputs : defs_and_outputs def_or_output'
    t[0] = t[1] + t[2]

# The list of possible definition/output statements.
def p_def_or_output(t):
    '''def_or_output : def_format
                     | def_bitfield
                     | def_bitfield_struct
                     | def_template
                     | def_operand_types
                     | def_operands
                     | output_header
                     | output_decoder
                     | output_exec
                     | global_let'''
    t[0] = t[1]

# Output blocks 'output <foo> {{...}}' (C++ code blocks) are copied
# directly to the appropriate output section.


# Protect any non-dict-substitution '%'s in a format string
# (i.e. those not followed by '(')
def protect_non_subst_percents(s):
    return re.sub(r'%(?!\()', '%%', s)

# Massage output block by substituting in template definitions and bit
# operators.  We handle '%'s embedded in the string that don't
# indicate template substitutions (or CPU-specific symbols, which get
# handled in GenCode) by doubling them first so that the format
# operation will reduce them back to single '%'s.
def process_output(s):
    s = protect_non_subst_percents(s)
    # protects cpu-specific symbols too
    s = protect_cpu_symbols(s)
    return substBitOps(s % templateMap)

def p_output_header(t):
    'output_header : OUTPUT HEADER CODELIT SEMI'
    t[0] = GenCode(header_output = process_output(t[3]))

def p_output_decoder(t):
    'output_decoder : OUTPUT DECODER CODELIT SEMI'
    t[0] = GenCode(decoder_output = process_output(t[3]))

def p_output_exec(t):
    'output_exec : OUTPUT EXEC CODELIT SEMI'
    t[0] = GenCode(exec_output = process_output(t[3]))

# global let blocks 'let {{...}}' (Python code blocks) are executed
# directly when seen.  Note that these execute in a special variable
# context 'exportContext' to prevent the code from polluting this
# script's namespace.
def p_global_let(t):
    'global_let : LET CODELIT SEMI'
    updateExportContext()
    exportContext["header_output"] = ''
    exportContext["decoder_output"] = ''
    exportContext["exec_output"] = ''
    exportContext["decode_block"] = ''
    try:
        exec fixPythonIndentation(t[2]) in exportContext
    except Exception, exc:
        error(t.lineno(1),
              'error: %s in global let block "%s".' % (exc, t[2]))
    t[0] = GenCode(header_output = exportContext["header_output"],
                   decoder_output = exportContext["decoder_output"],
                   exec_output = exportContext["exec_output"],
                   decode_block = exportContext["decode_block"])

# Define the mapping from operand type extensions to C++ types and bit
# widths (stored in operandTypeMap).
def p_def_operand_types(t):
    'def_operand_types : DEF OPERAND_TYPES CODELIT SEMI'
    try:
        userDict = eval('{' + t[3] + '}')
    except Exception, exc:
        error(t.lineno(1),
              'error: %s in def operand_types block "%s".' % (exc, t[3]))
    buildOperandTypeMap(userDict, t.lineno(1))
    t[0] = GenCode() # contributes nothing to the output C++ file

# Define the mapping from operand names to operand classes and other
# traits.  Stored in operandNameMap.
def p_def_operands(t):
    'def_operands : DEF OPERANDS CODELIT SEMI'
    if not globals().has_key('operandTypeMap'):
        error(t.lineno(1),
              'error: operand types must be defined before operands')
    try:
        userDict = eval('{' + t[3] + '}')
    except Exception, exc:
        error(t.lineno(1),
              'error: %s in def operands block "%s".' % (exc, t[3]))
    buildOperandNameMap(userDict, t.lineno(1))
    t[0] = GenCode() # contributes nothing to the output C++ file

# A bitfield definition looks like:
# 'def [signed] bitfield <ID> [<first>:<last>]'
# This generates a preprocessor macro in the output file.
def p_def_bitfield_0(t):
    'def_bitfield : DEF opt_signed BITFIELD ID LESS INTLIT COLON INTLIT GREATER SEMI'
    expr = 'bits(machInst, %2d, %2d)' % (t[6], t[8])
    if (t[2] == 'signed'):
        expr = 'sext<%d>(%s)' % (t[6] - t[8] + 1, expr)
    hash_define = '#undef %s\n#define %s\t%s\n' % (t[4], t[4], expr)
    t[0] = GenCode(header_output = hash_define)

# alternate form for single bit: 'def [signed] bitfield <ID> [<bit>]'
def p_def_bitfield_1(t):
    'def_bitfield : DEF opt_signed BITFIELD ID LESS INTLIT GREATER SEMI'
    expr = 'bits(machInst, %2d, %2d)' % (t[6], t[6])
    if (t[2] == 'signed'):
        expr = 'sext<%d>(%s)' % (1, expr)
    hash_define = '#undef %s\n#define %s\t%s\n' % (t[4], t[4], expr)
    t[0] = GenCode(header_output = hash_define)

# alternate form for structure member: 'def bitfield <ID> <ID>'
def p_def_bitfield_struct(t):
    'def_bitfield_struct : DEF opt_signed BITFIELD ID id_with_dot SEMI'
    if (t[2] != ''):
        error(t.lineno(1), 'error: structure bitfields are always unsigned.')
    expr = 'machInst.%s' % t[5]
    hash_define = '#undef %s\n#define %s\t%s\n' % (t[4], t[4], expr)
    t[0] = GenCode(header_output = hash_define)

def p_id_with_dot_0(t):
    'id_with_dot : ID'
    t[0] = t[1]

def p_id_with_dot_1(t):
    'id_with_dot : ID DOT id_with_dot'
    t[0] = t[1] + t[2] + t[3]

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
    templateMap[t[3]] = Template(t[4])
    t[0] = GenCode()

# An instruction format definition looks like
# "def format <fmt>(<params>) {{...}};"
def p_def_format(t):
    'def_format : DEF FORMAT ID LPAREN param_list RPAREN CODELIT SEMI'
    (id, params, code) = (t[3], t[5], t[7])
    defFormat(id, params, code, t.lineno(1))
    t[0] = GenCode()

# The formal parameter list for an instruction format is a possibly
# empty list of comma-separated parameters.  Positional (standard,
# non-keyword) parameters must come first, followed by keyword
# parameters, followed by a '*foo' parameter that gets excess
# positional arguments (as in Python).  Each of these three parameter
# categories is optional.
#
# Note that we do not support the '**foo' parameter for collecting
# otherwise undefined keyword args.  Otherwise the parameter list is
# (I believe) identical to what is supported in Python.
#
# The param list generates a tuple, where the first element is a list of
# the positional params and the second element is a dict containing the
# keyword params.
def p_param_list_0(t):
    'param_list : positional_param_list COMMA nonpositional_param_list'
    t[0] = t[1] + t[3]

def p_param_list_1(t):
    '''param_list : positional_param_list
                  | nonpositional_param_list'''
    t[0] = t[1]

def p_positional_param_list_0(t):
    'positional_param_list : empty'
    t[0] = []

def p_positional_param_list_1(t):
    'positional_param_list : ID'
    t[0] = [t[1]]

def p_positional_param_list_2(t):
    'positional_param_list : positional_param_list COMMA ID'
    t[0] = t[1] + [t[3]]

def p_nonpositional_param_list_0(t):
    'nonpositional_param_list : keyword_param_list COMMA excess_args_param'
    t[0] = t[1] + t[3]

def p_nonpositional_param_list_1(t):
    '''nonpositional_param_list : keyword_param_list
                                | excess_args_param'''
    t[0] = t[1]

def p_keyword_param_list_0(t):
    'keyword_param_list : keyword_param'
    t[0] = [t[1]]

def p_keyword_param_list_1(t):
    'keyword_param_list : keyword_param_list COMMA keyword_param'
    t[0] = t[1] + [t[3]]

def p_keyword_param(t):
    'keyword_param : ID EQUALS expr'
    t[0] = t[1] + ' = ' + t[3].__repr__()

def p_excess_args_param(t):
    'excess_args_param : ASTERISK ID'
    # Just concatenate them: '*ID'.  Wrap in list to be consistent
    # with positional_param_list and keyword_param_list.
    t[0] = [t[1] + t[2]]

# End of format definition-related rules.
##############

#
# A decode block looks like:
#	decode <field1> [, <field2>]* [default <inst>] { ... }
#
def p_decode_block(t):
    'decode_block : DECODE ID opt_default LBRACE decode_stmt_list RBRACE'
    default_defaults = defaultStack.pop()
    codeObj = t[5]
    # use the "default defaults" only if there was no explicit
    # default statement in decode_stmt_list
    if not codeObj.has_decode_default:
        codeObj += default_defaults
    codeObj.wrap_decode_block('switch (%s) {\n' % t[2], '}\n')
    t[0] = codeObj

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
    codeObj = t[2]
    codeObj.wrap_decode_block('\ndefault:\n', 'break;\n')
    defaultStack.push(codeObj)
    # no meaningful value returned
    t[0] = None

def p_decode_stmt_list_0(t):
    'decode_stmt_list : decode_stmt'
    t[0] = t[1]

def p_decode_stmt_list_1(t):
    'decode_stmt_list : decode_stmt decode_stmt_list'
    if (t[1].has_decode_default and t[2].has_decode_default):
        error(t.lineno(1), 'Two default cases in decode block')
    t[0] = t[1] + t[2]

#
# Decode statement rules
#
# There are four types of statements allowed in a decode block:
# 1. Format blocks 'format <foo> { ... }'
# 2. Nested decode blocks
# 3. Instruction definitions.
# 4. C preprocessor directives.


# Preprocessor directives found in a decode statement list are passed
# through to the output, replicated to all of the output code
# streams.  This works well for ifdefs, so we can ifdef out both the
# declarations and the decode cases generated by an instruction
# definition.  Handling them as part of the grammar makes it easy to
# keep them in the right place with respect to the code generated by
# the other statements.
def p_decode_stmt_cpp(t):
    'decode_stmt : CPPDIRECTIVE'
    t[0] = GenCode(t[1], t[1], t[1], t[1])

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
    label = t[1]
    codeObj = t[3]
    # just wrap the decoding code from the block as a case in the
    # outer switch statement.
    codeObj.wrap_decode_block('\n%s:\n' % label)
    codeObj.has_decode_default = (label == 'default')
    t[0] = codeObj

# Instruction definition (finally!).
def p_decode_stmt_inst(t):
    'decode_stmt : case_label COLON inst SEMI'
    label = t[1]
    codeObj = t[3]
    codeObj.wrap_decode_block('\n%s:' % label, 'break;\n')
    codeObj.has_decode_default = (label == 'default')
    t[0] = codeObj

# The case label is either a list of one or more constants or 'default'
def p_case_label_0(t):
    'case_label : intlit_list'
    t[0] = ': '.join(map(lambda a: 'case %#x' % a, t[1]))

def p_case_label_1(t):
    'case_label : DEFAULT'
    t[0] = 'default'

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
    codeObj = currentFormat.defineInst(t[1], t[3], t.lineno(1))
    args = ','.join(map(str, t[3]))
    args = re.sub('(?m)^', '//', args)
    args = re.sub('^//', '', args)
    comment = '\n// %s::%s(%s)\n' % (currentFormat.id, t[1], args)
    codeObj.prepend_all(comment)
    t[0] = codeObj

# Define an instruction using an explicitly specified format:
# "<fmt>::<mnemonic>(<args>)"
def p_inst_1(t):
    'inst : ID DBLCOLON ID LPAREN arg_list RPAREN'
    try:
        format = formatMap[t[1]]
    except KeyError:
        error(t.lineno(1), 'instruction format "%s" not defined.' % t[1])
    codeObj = format.defineInst(t[3], t[5], t.lineno(1))
    comment = '\n// %s::%s(%s)\n' % (t[1], t[3], t[5])
    codeObj.prepend_all(comment)
    t[0] = codeObj

# The arg list generates a tuple, where the first element is a list of
# the positional args and the second element is a dict containing the
# keyword args.
def p_arg_list_0(t):
    'arg_list : positional_arg_list COMMA keyword_arg_list'
    t[0] = ( t[1], t[3] )

def p_arg_list_1(t):
    'arg_list : positional_arg_list'
    t[0] = ( t[1], {} )

def p_arg_list_2(t):
    'arg_list : keyword_arg_list'
    t[0] = ( [], t[1] )

def p_positional_arg_list_0(t):
    'positional_arg_list : empty'
    t[0] = []

def p_positional_arg_list_1(t):
    'positional_arg_list : expr'
    t[0] = [t[1]]

def p_positional_arg_list_2(t):
    'positional_arg_list : positional_arg_list COMMA expr'
    t[0] = t[1] + [t[3]]

def p_keyword_arg_list_0(t):
    'keyword_arg_list : keyword_arg'
    t[0] = t[1]

def p_keyword_arg_list_1(t):
    'keyword_arg_list : keyword_arg_list COMMA keyword_arg'
    t[0] = t[1]
    t[0].update(t[3])

def p_keyword_arg(t):
    'keyword_arg : ID EQUALS expr'
    t[0] = { t[1] : t[3] }

#
# Basic expressions.  These constitute the argument values of
# "function calls" (i.e. instruction definitions in the decode block)
# and default values for formal parameters of format functions.
#
# Right now, these are either strings, integers, or (recursively)
# lists of exprs (using Python square-bracket list syntax).  Note that
# bare identifiers are trated as string constants here (since there
# isn't really a variable namespace to refer to).
#
def p_expr_0(t):
    '''expr : ID
            | INTLIT
            | STRLIT
            | CODELIT'''
    t[0] = t[1]

def p_expr_1(t):
    '''expr : LBRACKET list_expr RBRACKET'''
    t[0] = t[2]

def p_list_expr_0(t):
    'list_expr : expr'
    t[0] = [t[1]]

def p_list_expr_1(t):
    'list_expr : list_expr COMMA expr'
    t[0] = t[1] + [t[3]]

def p_list_expr_2(t):
    'list_expr : empty'
    t[0] = []

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
        error(0, "unknown syntax error", True)

# END OF GRAMMAR RULES
#
# Now build the parser.
yacc.yacc()


#####################################################################
#
#                           Support Classes
#
#####################################################################

# Expand template with CPU-specific references into a dictionary with
# an entry for each CPU model name.  The entry key is the model name
# and the corresponding value is the template with the CPU-specific
# refs substituted for that model.
def expand_cpu_symbols_to_dict(template):
    # Protect '%'s that don't go with CPU-specific terms
    t = re.sub(r'%(?!\(CPU_)', '%%', template)
    result = {}
    for cpu in cpu_models:
        result[cpu.name] = t % cpu.strings
    return result

# *If* the template has CPU-specific references, return a single
# string containing a copy of the template for each CPU model with the
# corresponding values substituted in.  If the template has no
# CPU-specific references, it is returned unmodified.
def expand_cpu_symbols_to_string(template):
    if template.find('%(CPU_') != -1:
        return reduce(lambda x,y: x+y,
                      expand_cpu_symbols_to_dict(template).values())
    else:
        return template

# Protect CPU-specific references by doubling the corresponding '%'s
# (in preparation for substituting a different set of references into
# the template).
def protect_cpu_symbols(template):
    return re.sub(r'%(?=\(CPU_)', '%%', template)

###############
# GenCode class
#
# The GenCode class encapsulates generated code destined for various
# output files.  The header_output and decoder_output attributes are
# strings containing code destined for decoder.hh and decoder.cc
# respectively.  The decode_block attribute contains code to be
# incorporated in the decode function itself (that will also end up in
# decoder.cc).  The exec_output attribute is a dictionary with a key
# for each CPU model name; the value associated with a particular key
# is the string of code for that CPU model's exec.cc file.  The
# has_decode_default attribute is used in the decode block to allow
# explicit default clauses to override default default clauses.

class GenCode:
    # Constructor.  At this point we substitute out all CPU-specific
    # symbols.  For the exec output, these go into the per-model
    # dictionary.  For all other output types they get collapsed into
    # a single string.
    def __init__(self,
                 header_output = '', decoder_output = '', exec_output = '',
                 decode_block = '', has_decode_default = False):
        self.header_output = expand_cpu_symbols_to_string(header_output)
        self.decoder_output = expand_cpu_symbols_to_string(decoder_output)
        if isinstance(exec_output, dict):
            self.exec_output = exec_output
        elif isinstance(exec_output, str):
            # If the exec_output arg is a single string, we replicate
            # it for each of the CPU models, substituting and
            # %(CPU_foo)s params appropriately.
            self.exec_output = expand_cpu_symbols_to_dict(exec_output)
        self.decode_block = expand_cpu_symbols_to_string(decode_block)
        self.has_decode_default = has_decode_default

    # Override '+' operator: generate a new GenCode object that
    # concatenates all the individual strings in the operands.
    def __add__(self, other):
        exec_output = {}
        for cpu in cpu_models:
            n = cpu.name
            exec_output[n] = self.exec_output[n] + other.exec_output[n]
        return GenCode(self.header_output + other.header_output,
                       self.decoder_output + other.decoder_output,
                       exec_output,
                       self.decode_block + other.decode_block,
                       self.has_decode_default or other.has_decode_default)

    # Prepend a string (typically a comment) to all the strings.
    def prepend_all(self, pre):
        self.header_output = pre + self.header_output
        self.decoder_output  = pre + self.decoder_output
        self.decode_block = pre + self.decode_block
        for cpu in cpu_models:
            self.exec_output[cpu.name] = pre + self.exec_output[cpu.name]

    # Wrap the decode block in a pair of strings (e.g., 'case foo:'
    # and 'break;').  Used to build the big nested switch statement.
    def wrap_decode_block(self, pre, post = ''):
        self.decode_block = pre + indent(self.decode_block) + post

################
# Format object.
#
# A format object encapsulates an instruction format.  It must provide
# a defineInst() method that generates the code for an instruction
# definition.

exportContextSymbols = ('InstObjParams', 'makeList', 're', 'string')

exportContext = {}

def updateExportContext():
    exportContext.update(exportDict(*exportContextSymbols))
    exportContext.update(templateMap)

def exportDict(*symNames):
    return dict([(s, eval(s)) for s in symNames])


class Format:
    def __init__(self, id, params, code):
        # constructor: just save away arguments
        self.id = id
        self.params = params
        label = 'def format ' + id
        self.user_code = compile(fixPythonIndentation(code), label, 'exec')
        param_list = string.join(params, ", ")
        f = '''def defInst(_code, _context, %s):
                my_locals = vars().copy()
                exec _code in _context, my_locals
                return my_locals\n''' % param_list
        c = compile(f, label + ' wrapper', 'exec')
        exec c
        self.func = defInst

    def defineInst(self, name, args, lineno):
        context = {}
        updateExportContext()
        context.update(exportContext)
        if len(name):
            Name = name[0].upper()
            if len(name) > 1:
                Name += name[1:]
        context.update({ 'name': name, 'Name': Name })
        try:
            vars = self.func(self.user_code, context, *args[0], **args[1])
        except Exception, exc:
            error(lineno, 'error defining "%s": %s.' % (name, exc))
        for k in vars.keys():
            if k not in ('header_output', 'decoder_output',
                         'exec_output', 'decode_block'):
                del vars[k]
        return GenCode(**vars)

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
# and default cases (defaultStack).  Simply wraps a list to give more
# stack-like syntax and enable initialization with an argument list
# (as opposed to an argument that's a list).

class Stack(list):
    def __init__(self, *items):
        list.__init__(self, items)

    def push(self, item):
        self.append(item);

    def top(self):
        return self[-1]

# The global format stack.
formatStack = Stack(NoFormat())

# The global default case stack.
defaultStack = Stack( None )

# Global stack that tracks current file and line number.
# Each element is a tuple (filename, lineno) that records the
# *current* filename and the line number in the *previous* file where
# it was included.
fileNameStack = Stack()

###################
# Utility functions

#
# Indent every line in string 's' by two spaces
# (except preprocessor directives).
# Used to make nested code blocks look pretty.
#
def indent(s):
    return re.sub(r'(?m)^(?!#)', '  ', s)

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
# Emacs compile-mode.  Optional 'print_traceback' arg, if set to True,
# prints a Python stack backtrace too (can be handy when trying to
# debug the parser itself).
def error(lineno, string, print_traceback = False):
    spaces = ""
    for (filename, line) in fileNameStack[0:-1]:
        print spaces + "In file included from " + filename + ":"
        spaces += "  "
    # Print a Python stack backtrace if requested.
    if (print_traceback):
        traceback.print_exc()
    if lineno != 0:
        line_str = "%d:" % lineno
    else:
        line_str = ""
    sys.exit(spaces + "%s:%s %s" % (fileNameStack[-1][0], line_str, string))


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


####################
# Template objects.
#
# Template objects are format strings that allow substitution from
# the attribute spaces of other objects (e.g. InstObjParams instances).

labelRE = re.compile(r'[^%]%\(([^\)]+)\)[sd]')

class Template:
    def __init__(self, t):
        self.template = t

    def subst(self, d):
        myDict = None

        # Protect non-Python-dict substitutions (e.g. if there's a printf
        # in the templated C++ code)
        template = protect_non_subst_percents(self.template)
        # CPU-model-specific substitutions are handled later (in GenCode).
        template = protect_cpu_symbols(template)

        # Build a dict ('myDict') to use for the template substitution.
        # Start with the template namespace.  Make a copy since we're
        # going to modify it.
        myDict = templateMap.copy()

        if isinstance(d, InstObjParams):
            # If we're dealing with an InstObjParams object, we need
            # to be a little more sophisticated.  The instruction-wide
            # parameters are already formed, but the parameters which
            # are only function wide still need to be generated.
            compositeCode = ''

            myDict.update(d.__dict__)
            # The "operands" and "snippets" attributes of the InstObjParams
            # objects are for internal use and not substitution.
            del myDict['operands']
            del myDict['snippets']

            snippetLabels = [l for l in labelRE.findall(template)
                             if d.snippets.has_key(l)]

            snippets = dict([(s, mungeSnippet(d.snippets[s]))
                             for s in snippetLabels])

            myDict.update(snippets)

            compositeCode = ' '.join(map(str, snippets.values()))

            # Add in template itself in case it references any
            # operands explicitly (like Mem)
            compositeCode += ' ' + template

            operands = SubOperandList(compositeCode, d.operands)

            myDict['op_decl'] = operands.concatAttrStrings('op_decl')

            is_src = lambda op: op.is_src
            is_dest = lambda op: op.is_dest

            myDict['op_src_decl'] = \
                      operands.concatSomeAttrStrings(is_src, 'op_src_decl')
            myDict['op_dest_decl'] = \
                      operands.concatSomeAttrStrings(is_dest, 'op_dest_decl')

            myDict['op_rd'] = operands.concatAttrStrings('op_rd')
            myDict['op_wb'] = operands.concatAttrStrings('op_wb')

            if d.operands.memOperand:
                myDict['mem_acc_size'] = d.operands.memOperand.mem_acc_size
                myDict['mem_acc_type'] = d.operands.memOperand.mem_acc_type

        elif isinstance(d, dict):
            # if the argument is a dictionary, we just use it.
            myDict.update(d)
        elif hasattr(d, '__dict__'):
            # if the argument is an object, we use its attribute map.
            myDict.update(d.__dict__)
        else:
            raise TypeError, "Template.subst() arg must be or have dictionary"
        return template % myDict

    # Convert to string.  This handles the case when a template with a
    # CPU-specific term gets interpolated into another template or into
    # an output block.
    def __str__(self):
        return expand_cpu_symbols_to_string(self.template)

#####################################################################
#
#                             Code Parser
#
# The remaining code is the support for automatically extracting
# instruction characteristics from pseudocode.
#
#####################################################################

# Force the argument to be a list.  Useful for flags, where a caller
# can specify a singleton flag or a list of flags.  Also usful for
# converting tuples to lists so they can be modified.
def makeList(arg):
    if isinstance(arg, list):
        return arg
    elif isinstance(arg, tuple):
        return list(arg)
    elif not arg:
        return []
    else:
        return [ arg ]

# Generate operandTypeMap from the user's 'def operand_types'
# statement.
def buildOperandTypeMap(userDict, lineno):
    global operandTypeMap
    operandTypeMap = {}
    for (ext, (desc, size)) in userDict.iteritems():
        if desc == 'signed int':
            ctype = 'int%d_t' % size
            is_signed = 1
        elif desc == 'unsigned int':
            ctype = 'uint%d_t' % size
            is_signed = 0
        elif desc == 'float':
            is_signed = 1	# shouldn't really matter
            if size == 32:
                ctype = 'float'
            elif size == 64:
                ctype = 'double'
        elif desc == 'twin64 int':
            is_signed = 0
            ctype = 'Twin64_t'
        elif desc == 'twin32 int':
            is_signed = 0
            ctype = 'Twin32_t'
        if ctype == '':
            error(lineno, 'Unrecognized type description "%s" in userDict')
        operandTypeMap[ext] = (size, ctype, is_signed)

#
#
#
# Base class for operand descriptors.  An instance of this class (or
# actually a class derived from this one) represents a specific
# operand for a code block (e.g, "Rc.sq" as a dest). Intermediate
# derived classes encapsulates the traits of a particular operand type
# (e.g., "32-bit integer register").
#
class Operand(object):
    def __init__(self, full_name, ext, is_src, is_dest):
        self.full_name = full_name
        self.ext = ext
        self.is_src = is_src
        self.is_dest = is_dest
        # The 'effective extension' (eff_ext) is either the actual
        # extension, if one was explicitly provided, or the default.
        if ext:
            self.eff_ext = ext
        else:
            self.eff_ext = self.dflt_ext

        (self.size, self.ctype, self.is_signed) = operandTypeMap[self.eff_ext]

        # note that mem_acc_size is undefined for non-mem operands...
        # template must be careful not to use it if it doesn't apply.
        if self.isMem():
            self.mem_acc_size = self.makeAccSize()
            if self.ctype in ['Twin32_t', 'Twin64_t']:
                self.mem_acc_type = 'Twin'
            else:
                self.mem_acc_type = 'uint'

    # Finalize additional fields (primarily code fields).  This step
    # is done separately since some of these fields may depend on the
    # register index enumeration that hasn't been performed yet at the
    # time of __init__().
    def finalize(self):
        self.flags = self.getFlags()
        self.constructor = self.makeConstructor()
        self.op_decl = self.makeDecl()

        if self.is_src:
            self.op_rd = self.makeRead()
            self.op_src_decl = self.makeDecl()
        else:
            self.op_rd = ''
            self.op_src_decl = ''

        if self.is_dest:
            self.op_wb = self.makeWrite()
            self.op_dest_decl = self.makeDecl()
        else:
            self.op_wb = ''
            self.op_dest_decl = ''

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

    def getFlags(self):
        # note the empty slice '[:]' gives us a copy of self.flags[0]
        # instead of a reference to it
        my_flags = self.flags[0][:]
        if self.is_src:
            my_flags += self.flags[1]
        if self.is_dest:
            my_flags += self.flags[2]
        return my_flags

    def makeDecl(self):
        # Note that initializations in the declarations are solely
        # to avoid 'uninitialized variable' errors from the compiler.
        return self.ctype + ' ' + self.base_name + ' = 0;\n';

class IntRegOperand(Operand):
    def isReg(self):
        return 1

    def isIntReg(self):
        return 1

    def makeConstructor(self):
        c = ''
        if self.is_src:
            c += '\n\t_srcRegIdx[%d] = %s;' % \
                 (self.src_reg_idx, self.reg_spec)
        if self.is_dest:
            c += '\n\t_destRegIdx[%d] = %s;' % \
                 (self.dest_reg_idx, self.reg_spec)
        return c

    def makeRead(self):
        if (self.ctype == 'float' or self.ctype == 'double'):
            error(0, 'Attempt to read integer register as FP')
        if (self.size == self.dflt_size):
            return '%s = xc->readIntRegOperand(this, %d);\n' % \
                   (self.base_name, self.src_reg_idx)
        elif (self.size > self.dflt_size):
            int_reg_val = 'xc->readIntRegOperand(this, %d)' % \
                          (self.src_reg_idx)
            if (self.is_signed):
                int_reg_val = 'sext<%d>(%s)' % (self.dflt_size, int_reg_val)
            return '%s = %s;\n' % (self.base_name, int_reg_val)
        else:
            return '%s = bits(xc->readIntRegOperand(this, %d), %d, 0);\n' % \
                   (self.base_name, self.src_reg_idx, self.size-1)

    def makeWrite(self):
        if (self.ctype == 'float' or self.ctype == 'double'):
            error(0, 'Attempt to write integer register as FP')
        if (self.size != self.dflt_size and self.is_signed):
            final_val = 'sext<%d>(%s)' % (self.size, self.base_name)
        else:
            final_val = self.base_name
        wb = '''
        {
            %s final_val = %s;
            xc->setIntRegOperand(this, %d, final_val);\n
            if (traceData) { traceData->setData(final_val); }
        }''' % (self.dflt_ctype, final_val, self.dest_reg_idx)
        return wb

class FloatRegOperand(Operand):
    def isReg(self):
        return 1

    def isFloatReg(self):
        return 1

    def makeConstructor(self):
        c = ''
        if self.is_src:
            c += '\n\t_srcRegIdx[%d] = %s + FP_Base_DepTag;' % \
                 (self.src_reg_idx, self.reg_spec)
        if self.is_dest:
            c += '\n\t_destRegIdx[%d] = %s + FP_Base_DepTag;' % \
                 (self.dest_reg_idx, self.reg_spec)
        return c

    def makeRead(self):
        bit_select = 0
        width = 0;
        if (self.ctype == 'float'):
            func = 'readFloatRegOperand'
            width = 32;
        elif (self.ctype == 'double'):
            func = 'readFloatRegOperand'
            width = 64;
        else:
            func = 'readFloatRegOperandBits'
            if (self.ctype == 'uint32_t'):
                width = 32;
            elif (self.ctype == 'uint64_t'):
                width = 64;
            if (self.size != self.dflt_size):
                bit_select = 1
        if width:
            base = 'xc->%s(this, %d, %d)' % \
                   (func, self.src_reg_idx, width)
        else:
            base = 'xc->%s(this, %d)' % \
                   (func, self.src_reg_idx)
        if bit_select:
            return '%s = bits(%s, %d, 0);\n' % \
                   (self.base_name, base, self.size-1)
        else:
            return '%s = %s;\n' % (self.base_name, base)

    def makeWrite(self):
        final_val = self.base_name
        final_ctype = self.ctype
        widthSpecifier = ''
        width = 0
        if (self.ctype == 'float'):
            width = 32
            func = 'setFloatRegOperand'
        elif (self.ctype == 'double'):
            width = 64
            func = 'setFloatRegOperand'
        elif (self.ctype == 'uint32_t'):
            func = 'setFloatRegOperandBits'
            width = 32
        elif (self.ctype == 'uint64_t'):
            func = 'setFloatRegOperandBits'
            width = 64
        else:
            func = 'setFloatRegOperandBits'
            final_ctype = 'uint%d_t' % self.dflt_size
            if (self.size != self.dflt_size and self.is_signed):
                final_val = 'sext<%d>(%s)' % (self.size, self.base_name)
        if width:
            widthSpecifier = ', %d' % width
        wb = '''
        {
            %s final_val = %s;
            xc->%s(this, %d, final_val%s);\n
            if (traceData) { traceData->setData(final_val); }
        }''' % (final_ctype, final_val, func, self.dest_reg_idx,
                widthSpecifier)
        return wb

class ControlRegOperand(Operand):
    def isReg(self):
        return 1

    def isControlReg(self):
        return 1

    def makeConstructor(self):
        c = ''
        if self.is_src:
            c += '\n\t_srcRegIdx[%d] = %s + Ctrl_Base_DepTag;' % \
                 (self.src_reg_idx, self.reg_spec)
        if self.is_dest:
            c += '\n\t_destRegIdx[%d] = %s + Ctrl_Base_DepTag;' % \
                 (self.dest_reg_idx, self.reg_spec)
        return c

    def makeRead(self):
        bit_select = 0
        if (self.ctype == 'float' or self.ctype == 'double'):
            error(0, 'Attempt to read control register as FP')
        base = 'xc->readMiscRegOperand(this, %s)' % self.src_reg_idx
        if self.size == self.dflt_size:
            return '%s = %s;\n' % (self.base_name, base)
        else:
            return '%s = bits(%s, %d, 0);\n' % \
                   (self.base_name, base, self.size-1)

    def makeWrite(self):
        if (self.ctype == 'float' or self.ctype == 'double'):
            error(0, 'Attempt to write control register as FP')
        wb = 'xc->setMiscRegOperand(this, %s, %s);\n' % \
             (self.dest_reg_idx, self.base_name)
        wb += 'if (traceData) { traceData->setData(%s); }' % \
              self.base_name
        return wb

class MemOperand(Operand):
    def isMem(self):
        return 1

    def makeConstructor(self):
        return ''

    def makeDecl(self):
        # Note that initializations in the declarations are solely
        # to avoid 'uninitialized variable' errors from the compiler.
        # Declare memory data variable.
        if self.ctype in ['Twin32_t','Twin64_t']:
            return "%s %s; %s.a = 0; %s.b = 0;\n" % (self.ctype, self.base_name,
                    self.base_name, self.base_name)
        c = '%s %s = 0;\n' % (self.ctype, self.base_name)
        return c

    def makeRead(self):
        return ''

    def makeWrite(self):
        return ''

    # Return the memory access size *in bits*, suitable for
    # forming a type via "uint%d_t".  Divide by 8 if you want bytes.
    def makeAccSize(self):
        return self.size


class NPCOperand(Operand):
    def makeConstructor(self):
        return ''

    def makeRead(self):
        return '%s = xc->readNextPC();\n' % self.base_name

    def makeWrite(self):
        return 'xc->setNextPC(%s);\n' % self.base_name

class NNPCOperand(Operand):
    def makeConstructor(self):
        return ''

    def makeRead(self):
        return '%s = xc->readNextNPC();\n' % self.base_name

    def makeWrite(self):
        return 'xc->setNextNPC(%s);\n' % self.base_name

def buildOperandNameMap(userDict, lineno):
    global operandNameMap
    operandNameMap = {}
    for (op_name, val) in userDict.iteritems():
        (base_cls_name, dflt_ext, reg_spec, flags, sort_pri) = val
        (dflt_size, dflt_ctype, dflt_is_signed) = operandTypeMap[dflt_ext]
        # Canonical flag structure is a triple of lists, where each list
        # indicates the set of flags implied by this operand always, when
        # used as a source, and when used as a dest, respectively.
        # For simplicity this can be initialized using a variety of fairly
        # obvious shortcuts; we convert these to canonical form here.
        if not flags:
            # no flags specified (e.g., 'None')
            flags = ( [], [], [] )
        elif isinstance(flags, str):
            # a single flag: assumed to be unconditional
            flags = ( [ flags ], [], [] )
        elif isinstance(flags, list):
            # a list of flags: also assumed to be unconditional
            flags = ( flags, [], [] )
        elif isinstance(flags, tuple):
            # it's a tuple: it should be a triple,
            # but each item could be a single string or a list
            (uncond_flags, src_flags, dest_flags) = flags
            flags = (makeList(uncond_flags),
                     makeList(src_flags), makeList(dest_flags))
        # Accumulate attributes of new operand class in tmp_dict
        tmp_dict = {}
        for attr in ('dflt_ext', 'reg_spec', 'flags', 'sort_pri',
                     'dflt_size', 'dflt_ctype', 'dflt_is_signed'):
            tmp_dict[attr] = eval(attr)
        tmp_dict['base_name'] = op_name
        # New class name will be e.g. "IntReg_Ra"
        cls_name = base_cls_name + '_' + op_name
        # Evaluate string arg to get class object.  Note that the
        # actual base class for "IntReg" is "IntRegOperand", i.e. we
        # have to append "Operand".
        try:
            base_cls = eval(base_cls_name + 'Operand')
        except NameError:
            error(lineno,
                  'error: unknown operand base class "%s"' % base_cls_name)
        # The following statement creates a new class called
        # <cls_name> as a subclass of <base_cls> with the attributes
        # in tmp_dict, just as if we evaluated a class declaration.
        operandNameMap[op_name] = type(cls_name, (base_cls,), tmp_dict)

    # Define operand variables.
    operands = userDict.keys()

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


class OperandList:

    # Find all the operands in the given code block.  Returns an operand
    # descriptor list (instance of class OperandList).
    def __init__(self, code):
        self.items = []
        self.bases = {}
        # delete comments so we don't match on reg specifiers inside
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
            op_desc = self.find_base(op_base)
            if op_desc:
                if op_desc.ext != op_ext:
                    error(0, 'Inconsistent extensions for operand %s' % \
                          op_base)
                op_desc.is_src = op_desc.is_src or is_src
                op_desc.is_dest = op_desc.is_dest or is_dest
            else:
                # new operand: create new descriptor
                op_desc = operandNameMap[op_base](op_full, op_ext,
                                                  is_src, is_dest)
                self.append(op_desc)
            # start next search after end of current match
            next_pos = match.end()
        self.sort()
        # enumerate source & dest register operands... used in building
        # constructor later
        self.numSrcRegs = 0
        self.numDestRegs = 0
        self.numFPDestRegs = 0
        self.numIntDestRegs = 0
        self.memOperand = None
        for op_desc in self.items:
            if op_desc.isReg():
                if op_desc.is_src:
                    op_desc.src_reg_idx = self.numSrcRegs
                    self.numSrcRegs += 1
                if op_desc.is_dest:
                    op_desc.dest_reg_idx = self.numDestRegs
                    self.numDestRegs += 1
                    if op_desc.isFloatReg():
                        self.numFPDestRegs += 1
                    elif op_desc.isIntReg():
                        self.numIntDestRegs += 1
            elif op_desc.isMem():
                if self.memOperand:
                    error(0, "Code block has more than one memory operand.")
                self.memOperand = op_desc
        # now make a final pass to finalize op_desc fields that may depend
        # on the register enumeration
        for op_desc in self.items:
            op_desc.finalize()

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
        self.items.sort(lambda a, b: a.sort_pri - b.sort_pri)

class SubOperandList(OperandList):

    # Find all the operands in the given code block.  Returns an operand
    # descriptor list (instance of class OperandList).
    def __init__(self, code, master_list):
        self.items = []
        self.bases = {}
        # delete comments so we don't match on reg specifiers inside
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
            # find this op in the master list
            op_desc = master_list.find_base(op_base)
            if not op_desc:
                error(0, 'Found operand %s which is not in the master list!' \
                        ' This is an internal error' % \
                          op_base)
            else:
                # See if we've already found this operand
                op_desc = self.find_base(op_base)
                if not op_desc:
                    # if not, add a reference to it to this sub list
                    self.append(master_list.bases[op_base])

            # start next search after end of current match
            next_pos = match.end()
        self.sort()
        self.memOperand = None
        for op_desc in self.items:
            if op_desc.isMem():
                if self.memOperand:
                    error(0, "Code block has more than one memory operand.")
                self.memOperand = op_desc

# Regular expression object to match C++ comments
# (used in findOperands())
commentRE = re.compile(r'//.*\n')

# Regular expression object to match assignment statements
# (used in findOperands())
assignRE = re.compile(r'\s*=(?!=)', re.MULTILINE)

# Munge operand names in code string to make legal C++ variable names.
# This means getting rid of the type extension if any.
# (Will match base_name attribute of Operand object.)
def substMungedOpNames(code):
    return operandsWithExtRE.sub(r'\1', code)

# Fix up code snippets for final substitution in templates.
def mungeSnippet(s):
    if isinstance(s, str):
        return substMungedOpNames(substBitOps(s))
    else:
        return s

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

# Assume all instruction flags are of the form 'IsFoo'
instFlagRE = re.compile(r'Is.*')

# OpClass constants end in 'Op' except No_OpClass
opClassRE = re.compile(r'.*Op|No_OpClass')

class InstObjParams:
    def __init__(self, mnem, class_name, base_class = '',
                 snippets = {}, opt_args = []):
        self.mnemonic = mnem
        self.class_name = class_name
        self.base_class = base_class
        if not isinstance(snippets, dict):
            snippets = {'code' : snippets}
        compositeCode = ' '.join(map(str, snippets.values()))
        self.snippets = snippets

        self.operands = OperandList(compositeCode)
        self.constructor = self.operands.concatAttrStrings('constructor')
        self.constructor += \
                 '\n\t_numSrcRegs = %d;' % self.operands.numSrcRegs
        self.constructor += \
                 '\n\t_numDestRegs = %d;' % self.operands.numDestRegs
        self.constructor += \
                 '\n\t_numFPDestRegs = %d;' % self.operands.numFPDestRegs
        self.constructor += \
                 '\n\t_numIntDestRegs = %d;' % self.operands.numIntDestRegs
        self.flags = self.operands.concatAttrLists('flags')

        # Make a basic guess on the operand class (function unit type).
        # These are good enough for most cases, and can be overridden
        # later otherwise.
        if 'IsStore' in self.flags:
            self.op_class = 'MemWriteOp'
        elif 'IsLoad' in self.flags or 'IsPrefetch' in self.flags:
            self.op_class = 'MemReadOp'
        elif 'IsFloating' in self.flags:
            self.op_class = 'FloatAddOp'
        else:
            self.op_class = 'IntAluOp'

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

#######################
#
# Output file template
#

file_template = '''
/*
 * DO NOT EDIT THIS FILE!!!
 *
 * It was automatically generated from the ISA description in %(filename)s
 */

%(includes)s

%(global_output)s

namespace %(namespace)s {

%(namespace_output)s

} // namespace %(namespace)s

%(decode_function)s
'''


# Update the output file only if the new contents are different from
# the current contents.  Minimizes the files that need to be rebuilt
# after minor changes.
def update_if_needed(file, contents):
    update = False
    if os.access(file, os.R_OK):
        f = open(file, 'r')
        old_contents = f.read()
        f.close()
        if contents != old_contents:
            print 'Updating', file
            os.remove(file) # in case it's write-protected
            update = True
        else:
            print 'File', file, 'is unchanged'
    else:
        print 'Generating', file
        update = True
    if update:
        f = open(file, 'w')
        f.write(contents)
        f.close()

# This regular expression matches '##include' directives
includeRE = re.compile(r'^\s*##include\s+"(?P<filename>[\w/.-]*)".*$',
                       re.MULTILINE)

# Function to replace a matched '##include' directive with the
# contents of the specified file (with nested ##includes replaced
# recursively).  'matchobj' is an re match object (from a match of
# includeRE) and 'dirname' is the directory relative to which the file
# path should be resolved.
def replace_include(matchobj, dirname):
    fname = matchobj.group('filename')
    full_fname = os.path.normpath(os.path.join(dirname, fname))
    contents = '##newfile "%s"\n%s\n##endfile\n' % \
               (full_fname, read_and_flatten(full_fname))
    return contents

# Read a file and recursively flatten nested '##include' files.
def read_and_flatten(filename):
    current_dir = os.path.dirname(filename)
    try:
        contents = open(filename).read()
    except IOError:
        error(0, 'Error including file "%s"' % filename)
    fileNameStack.push((filename, 0))
    # Find any includes and include them
    contents = includeRE.sub(lambda m: replace_include(m, current_dir),
                             contents)
    fileNameStack.pop()
    return contents

#
# Read in and parse the ISA description.
#
def parse_isa_desc(isa_desc_file, output_dir):
    # Read file and (recursively) all included files into a string.
    # PLY requires that the input be in a single string so we have to
    # do this up front.
    isa_desc = read_and_flatten(isa_desc_file)

    # Initialize filename stack with outer file.
    fileNameStack.push((isa_desc_file, 0))

    # Parse it.
    (isa_name, namespace, global_code, namespace_code) = yacc.parse(isa_desc)

    # grab the last three path components of isa_desc_file to put in
    # the output
    filename = '/'.join(isa_desc_file.split('/')[-3:])

    # generate decoder.hh
    includes = '#include "base/bitfield.hh" // for bitfield support'
    global_output = global_code.header_output
    namespace_output = namespace_code.header_output
    decode_function = ''
    update_if_needed(output_dir + '/decoder.hh', file_template % vars())

    # generate decoder.cc
    includes = '#include "decoder.hh"'
    global_output = global_code.decoder_output
    namespace_output = namespace_code.decoder_output
    # namespace_output += namespace_code.decode_block
    decode_function = namespace_code.decode_block
    update_if_needed(output_dir + '/decoder.cc', file_template % vars())

    # generate per-cpu exec files
    for cpu in cpu_models:
        includes = '#include "decoder.hh"\n'
        includes += cpu.includes
        global_output = global_code.exec_output[cpu.name]
        namespace_output = namespace_code.exec_output[cpu.name]
        decode_function = ''
        update_if_needed(output_dir + '/' + cpu.filename,
                          file_template % vars())

# global list of CpuModel objects (see cpu_models.py)
cpu_models = []

# Called as script: get args from command line.
# Args are: <path to cpu_models.py> <isa desc file> <output dir> <cpu models>
if __name__ == '__main__':
    execfile(sys.argv[1])  # read in CpuModel definitions
    cpu_models = [CpuModel.dict[cpu] for cpu in sys.argv[4:]]
    parse_isa_desc(sys.argv[2], sys.argv[3])
