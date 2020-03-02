# Copyright (c) 2014, 2016, 2018-2019 ARM Limited
# All rights reserved
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 2003-2005 The Regents of The University of Michigan
# Copyright (c) 2013,2015 Advanced Micro Devices, Inc.
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

from __future__ import with_statement, print_function
import os
import sys
import re
import inspect, traceback
# get type names
from types import *

from m5.util.grammar import Grammar

debug=False

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

class ISAParserError(Exception):
    """Exception class for parser errors"""
    def __init__(self, first, second=None):
        if second is None:
            self.lineno = 0
            self.string = first
        else:
            self.lineno = first
            self.string = second

    def __str__(self):
        return self.string

def error(*args):
    raise ISAParserError(*args)

####################
# Template objects.
#
# Template objects are format strings that allow substitution from
# the attribute spaces of other objects (e.g. InstObjParams instances).

labelRE = re.compile(r'(?<!%)%\(([^\)]+)\)[sd]')

class Template(object):
    def __init__(self, parser, t):
        self.parser = parser
        self.template = t

    def subst(self, d):
        myDict = None

        # Protect non-Python-dict substitutions (e.g. if there's a printf
        # in the templated C++ code)
        template = self.parser.protectNonSubstPercents(self.template)

        # Build a dict ('myDict') to use for the template substitution.
        # Start with the template namespace.  Make a copy since we're
        # going to modify it.
        myDict = self.parser.templateMap.copy()

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
                             if l in d.snippets]

            snippets = dict([(s, self.parser.mungeSnippet(d.snippets[s]))
                             for s in snippetLabels])

            myDict.update(snippets)

            compositeCode = ' '.join(list(map(str, snippets.values())))

            # Add in template itself in case it references any
            # operands explicitly (like Mem)
            compositeCode += ' ' + template

            operands = SubOperandList(self.parser, compositeCode, d.operands)

            myDict['op_decl'] = operands.concatAttrStrings('op_decl')
            if operands.readPC or operands.setPC:
                myDict['op_decl'] += 'TheISA::PCState __parserAutoPCState;\n'

            # In case there are predicated register reads and write, declare
            # the variables for register indicies. It is being assumed that
            # all the operands in the OperandList are also in the
            # SubOperandList and in the same order. Otherwise, it is
            # expected that predication would not be used for the operands.
            if operands.predRead:
                myDict['op_decl'] += 'uint8_t _sourceIndex = 0;\n'
            if operands.predWrite:
                myDict['op_decl'] += 'uint8_t M5_VAR_USED _destIndex = 0;\n'

            is_src = lambda op: op.is_src
            is_dest = lambda op: op.is_dest

            myDict['op_src_decl'] = \
                      operands.concatSomeAttrStrings(is_src, 'op_src_decl')
            myDict['op_dest_decl'] = \
                      operands.concatSomeAttrStrings(is_dest, 'op_dest_decl')
            if operands.readPC:
                myDict['op_src_decl'] += \
                    'TheISA::PCState __parserAutoPCState;\n'
            if operands.setPC:
                myDict['op_dest_decl'] += \
                    'TheISA::PCState __parserAutoPCState;\n'

            myDict['op_rd'] = operands.concatAttrStrings('op_rd')
            if operands.readPC:
                myDict['op_rd'] = '__parserAutoPCState = xc->pcState();\n' + \
                                  myDict['op_rd']

            # Compose the op_wb string. If we're going to write back the
            # PC state because we changed some of its elements, we'll need to
            # do that as early as possible. That allows later uncoordinated
            # modifications to the PC to layer appropriately.
            reordered = list(operands.items)
            reordered.reverse()
            op_wb_str = ''
            pcWbStr = 'xc->pcState(__parserAutoPCState);\n'
            for op_desc in reordered:
                if op_desc.isPCPart() and op_desc.is_dest:
                    op_wb_str = op_desc.op_wb + pcWbStr + op_wb_str
                    pcWbStr = ''
                else:
                    op_wb_str = op_desc.op_wb + op_wb_str
            myDict['op_wb'] = op_wb_str

        elif isinstance(d, dict):
            # if the argument is a dictionary, we just use it.
            myDict.update(d)
        elif hasattr(d, '__dict__'):
            # if the argument is an object, we use its attribute map.
            myDict.update(d.__dict__)
        else:
            raise TypeError("Template.subst() arg must be or have dictionary")
        return template % myDict

    # Convert to string.
    def __str__(self):
        return self.template

################
# Format object.
#
# A format object encapsulates an instruction format.  It must provide
# a defineInst() method that generates the code for an instruction
# definition.

class Format(object):
    def __init__(self, id, params, code):
        self.id = id
        self.params = params
        label = 'def format ' + id
        self.user_code = compile(fixPythonIndentation(code), label, 'exec')
        param_list = ", ".join(params)
        f = '''def defInst(_code, _context, %s):
                my_locals = vars().copy()
                exec(_code, _context, my_locals)
                return my_locals\n''' % param_list
        c = compile(f, label + ' wrapper', 'exec')
        exec(c, globals())
        self.func = defInst

    def defineInst(self, parser, name, args, lineno):
        parser.updateExportContext()
        context = parser.exportContext.copy()
        if len(name):
            Name = name[0].upper()
            if len(name) > 1:
                Name += name[1:]
        context.update({ 'name' : name, 'Name' : Name })
        try:
            vars = self.func(self.user_code, context, *args[0], **args[1])
        except Exception as exc:
            if debug:
                raise
            error(lineno, 'error defining "%s": %s.' % (name, exc))
        for k in list(vars.keys()):
            if k not in ('header_output', 'decoder_output',
                         'exec_output', 'decode_block'):
                del vars[k]
        return GenCode(parser, **vars)

# Special null format to catch an implicit-format instruction
# definition outside of any format block.
class NoFormat(object):
    def __init__(self):
        self.defaultInst = ''

    def defineInst(self, parser, name, args, lineno):
        error(lineno,
              'instruction definition "%s" with no active format!' % name)

###############
# GenCode class
#
# The GenCode class encapsulates generated code destined for various
# output files.  The header_output and decoder_output attributes are
# strings containing code destined for decoder.hh and decoder.cc
# respectively.  The decode_block attribute contains code to be
# incorporated in the decode function itself (that will also end up in
# decoder.cc).  The exec_output attribute  is the string of code for the
# exec.cc file.  The has_decode_default attribute is used in the decode block
# to allow explicit default clauses to override default default clauses.

class GenCode(object):
    # Constructor.
    def __init__(self, parser,
                 header_output = '', decoder_output = '', exec_output = '',
                 decode_block = '', has_decode_default = False):
        self.parser = parser
        self.header_output = header_output
        self.decoder_output = decoder_output
        self.exec_output = exec_output
        self.decode_block = decode_block
        self.has_decode_default = has_decode_default

    # Write these code chunks out to the filesystem.  They will be properly
    # interwoven by the write_top_level_files().
    def emit(self):
        if self.header_output:
            self.parser.get_file('header').write(self.header_output)
        if self.decoder_output:
            self.parser.get_file('decoder').write(self.decoder_output)
        if self.exec_output:
            self.parser.get_file('exec').write(self.exec_output)
        if self.decode_block:
            self.parser.get_file('decode_block').write(self.decode_block)

    # Override '+' operator: generate a new GenCode object that
    # concatenates all the individual strings in the operands.
    def __add__(self, other):
        return GenCode(self.parser,
                       self.header_output + other.header_output,
                       self.decoder_output + other.decoder_output,
                       self.exec_output + other.exec_output,
                       self.decode_block + other.decode_block,
                       self.has_decode_default or other.has_decode_default)

    # Prepend a string (typically a comment) to all the strings.
    def prepend_all(self, pre):
        self.header_output = pre + self.header_output
        self.decoder_output  = pre + self.decoder_output
        self.decode_block = pre + self.decode_block
        self.exec_output  = pre + self.exec_output

    # Wrap the decode block in a pair of strings (e.g., 'case foo:'
    # and 'break;').  Used to build the big nested switch statement.
    def wrap_decode_block(self, pre, post = ''):
        self.decode_block = pre + indent(self.decode_block) + post

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

class Operand(object):
    '''Base class for operand descriptors.  An instance of this class
    (or actually a class derived from this one) represents a specific
    operand for a code block (e.g, "Rc.sq" as a dest). Intermediate
    derived classes encapsulates the traits of a particular operand
    type (e.g., "32-bit integer register").'''

    def buildReadCode(self, func = None):
        subst_dict = {"name": self.base_name,
                      "func": func,
                      "reg_idx": self.reg_spec,
                      "ctype": self.ctype}
        if hasattr(self, 'src_reg_idx'):
            subst_dict['op_idx'] = self.src_reg_idx
        code = self.read_code % subst_dict
        return '%s = %s;\n' % (self.base_name, code)

    def buildWriteCode(self, func = None):
        subst_dict = {"name": self.base_name,
                      "func": func,
                      "reg_idx": self.reg_spec,
                      "ctype": self.ctype,
                      "final_val": self.base_name}
        if hasattr(self, 'dest_reg_idx'):
            subst_dict['op_idx'] = self.dest_reg_idx
        code = self.write_code % subst_dict
        return '''
        {
            %s final_val = %s;
            %s;
            if (traceData) { traceData->setData(final_val); }
        }''' % (self.dflt_ctype, self.base_name, code)

    def __init__(self, parser, full_name, ext, is_src, is_dest):
        self.full_name = full_name
        self.ext = ext
        self.is_src = is_src
        self.is_dest = is_dest
        # The 'effective extension' (eff_ext) is either the actual
        # extension, if one was explicitly provided, or the default.
        if ext:
            self.eff_ext = ext
        elif hasattr(self, 'dflt_ext'):
            self.eff_ext = self.dflt_ext

        if hasattr(self, 'eff_ext'):
            self.ctype = parser.operandTypeMap[self.eff_ext]

    # Finalize additional fields (primarily code fields).  This step
    # is done separately since some of these fields may depend on the
    # register index enumeration that hasn't been performed yet at the
    # time of __init__(). The register index enumeration is affected
    # by predicated register reads/writes. Hence, we forward the flags
    # that indicate whether or not predication is in use.
    def finalize(self, predRead, predWrite):
        self.flags = self.getFlags()
        self.constructor = self.makeConstructor(predRead, predWrite)
        self.op_decl = self.makeDecl()

        if self.is_src:
            self.op_rd = self.makeRead(predRead)
            self.op_src_decl = self.makeDecl()
        else:
            self.op_rd = ''
            self.op_src_decl = ''

        if self.is_dest:
            self.op_wb = self.makeWrite(predWrite)
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

    def isCCReg(self):
        return 0

    def isControlReg(self):
        return 0

    def isVecReg(self):
        return 0

    def isVecElem(self):
        return 0

    def isVecPredReg(self):
        return 0

    def isPCState(self):
        return 0

    def isPCPart(self):
        return self.isPCState() and self.reg_spec

    def hasReadPred(self):
        return self.read_predicate != None

    def hasWritePred(self):
        return self.write_predicate != None

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


src_reg_constructor = '\n\t_srcRegIdx[_numSrcRegs++] = RegId(%s, %s);'
dst_reg_constructor = '\n\t_destRegIdx[_numDestRegs++] = RegId(%s, %s);'


class IntRegOperand(Operand):
    reg_class = 'IntRegClass'

    def isReg(self):
        return 1

    def isIntReg(self):
        return 1

    def makeConstructor(self, predRead, predWrite):
        c_src = ''
        c_dest = ''

        if self.is_src:
            c_src = src_reg_constructor % (self.reg_class, self.reg_spec)
            if self.hasReadPred():
                c_src = '\n\tif (%s) {%s\n\t}' % \
                        (self.read_predicate, c_src)

        if self.is_dest:
            c_dest = dst_reg_constructor % (self.reg_class, self.reg_spec)
            c_dest += '\n\t_numIntDestRegs++;'
            if self.hasWritePred():
                c_dest = '\n\tif (%s) {%s\n\t}' % \
                         (self.write_predicate, c_dest)

        return c_src + c_dest

    def makeRead(self, predRead):
        if (self.ctype == 'float' or self.ctype == 'double'):
            error('Attempt to read integer register as FP')
        if self.read_code != None:
            return self.buildReadCode('readIntRegOperand')

        int_reg_val = ''
        if predRead:
            int_reg_val = 'xc->readIntRegOperand(this, _sourceIndex++)'
            if self.hasReadPred():
                int_reg_val = '(%s) ? %s : 0' % \
                              (self.read_predicate, int_reg_val)
        else:
            int_reg_val = 'xc->readIntRegOperand(this, %d)' % self.src_reg_idx

        return '%s = %s;\n' % (self.base_name, int_reg_val)

    def makeWrite(self, predWrite):
        if (self.ctype == 'float' or self.ctype == 'double'):
            error('Attempt to write integer register as FP')
        if self.write_code != None:
            return self.buildWriteCode('setIntRegOperand')

        if predWrite:
            wp = 'true'
            if self.hasWritePred():
                wp = self.write_predicate

            wcond = 'if (%s)' % (wp)
            windex = '_destIndex++'
        else:
            wcond = ''
            windex = '%d' % self.dest_reg_idx

        wb = '''
        %s
        {
            %s final_val = %s;
            xc->setIntRegOperand(this, %s, final_val);\n
            if (traceData) { traceData->setData(final_val); }
        }''' % (wcond, self.ctype, self.base_name, windex)

        return wb

class FloatRegOperand(Operand):
    reg_class = 'FloatRegClass'

    def isReg(self):
        return 1

    def isFloatReg(self):
        return 1

    def makeConstructor(self, predRead, predWrite):
        c_src = ''
        c_dest = ''

        if self.is_src:
            c_src = src_reg_constructor % (self.reg_class, self.reg_spec)

        if self.is_dest:
            c_dest = dst_reg_constructor % (self.reg_class, self.reg_spec)
            c_dest += '\n\t_numFPDestRegs++;'

        return c_src + c_dest

    def makeRead(self, predRead):
        if self.read_code != None:
            return self.buildReadCode('readFloatRegOperandBits')

        if predRead:
            rindex = '_sourceIndex++'
        else:
            rindex = '%d' % self.src_reg_idx

        code = 'xc->readFloatRegOperandBits(this, %s)' % rindex
        if self.ctype == 'float':
            code = 'bitsToFloat32(%s)' % code
        elif self.ctype == 'double':
            code = 'bitsToFloat64(%s)' % code
        return '%s = %s;\n' % (self.base_name, code)

    def makeWrite(self, predWrite):
        if self.write_code != None:
            return self.buildWriteCode('setFloatRegOperandBits')

        if predWrite:
            wp = '_destIndex++'
        else:
            wp = '%d' % self.dest_reg_idx

        val = 'final_val'
        if self.ctype == 'float':
            val = 'floatToBits32(%s)' % val
        elif self.ctype == 'double':
            val = 'floatToBits64(%s)' % val

        wp = 'xc->setFloatRegOperandBits(this, %s, %s);' % (wp, val)

        wb = '''
        {
            %s final_val = %s;
            %s\n
            if (traceData) { traceData->setData(final_val); }
        }''' % (self.ctype, self.base_name, wp)
        return wb

class VecRegOperand(Operand):
    reg_class = 'VecRegClass'

    def __init__(self, parser, full_name, ext, is_src, is_dest):
        Operand.__init__(self, parser, full_name, ext, is_src, is_dest)
        self.elemExt = None
        self.parser = parser

    def isReg(self):
        return 1

    def isVecReg(self):
        return 1

    def makeDeclElem(self, elem_op):
        (elem_name, elem_ext) = elem_op
        (elem_spec, dflt_elem_ext, zeroing) = self.elems[elem_name]
        if elem_ext:
            ext = elem_ext
        else:
            ext = dflt_elem_ext
        ctype = self.parser.operandTypeMap[ext]
        return '\n\t%s %s = 0;' % (ctype, elem_name)

    def makeDecl(self):
        if not self.is_dest and self.is_src:
            c_decl = '\t/* Vars for %s*/' % (self.base_name)
            if hasattr(self, 'active_elems'):
                if self.active_elems:
                    for elem in self.active_elems:
                        c_decl += self.makeDeclElem(elem)
            return c_decl + '\t/* End vars for %s */\n' % (self.base_name)
        else:
            return ''

    def makeConstructor(self, predRead, predWrite):
        c_src = ''
        c_dest = ''

        numAccessNeeded = 1

        if self.is_src:
            c_src = src_reg_constructor % (self.reg_class, self.reg_spec)

        if self.is_dest:
            c_dest = dst_reg_constructor % (self.reg_class, self.reg_spec)
            c_dest += '\n\t_numVecDestRegs++;'

        return c_src + c_dest

    # Read destination register to write
    def makeReadWElem(self, elem_op):
        (elem_name, elem_ext) = elem_op
        (elem_spec, dflt_elem_ext, zeroing) = self.elems[elem_name]
        if elem_ext:
            ext = elem_ext
        else:
            ext = dflt_elem_ext
        ctype = self.parser.operandTypeMap[ext]
        c_read = '\t\t%s& %s = %s[%s];\n' % \
                  (ctype, elem_name, self.base_name, elem_spec)
        return c_read

    def makeReadW(self, predWrite):
        func = 'getWritableVecRegOperand'
        if self.read_code != None:
            return self.buildReadCode(func)

        if predWrite:
            rindex = '_destIndex++'
        else:
            rindex = '%d' % self.dest_reg_idx

        c_readw = '\t\t%s& tmp_d%s = xc->%s(this, %s);\n'\
                % ('TheISA::VecRegContainer', rindex, func, rindex)
        if self.elemExt:
            c_readw += '\t\tauto %s = tmp_d%s.as<%s>();\n' % (self.base_name,
                        rindex, self.parser.operandTypeMap[self.elemExt])
        if self.ext:
            c_readw += '\t\tauto %s = tmp_d%s.as<%s>();\n' % (self.base_name,
                        rindex, self.parser.operandTypeMap[self.ext])
        if hasattr(self, 'active_elems'):
            if self.active_elems:
                for elem in self.active_elems:
                    c_readw += self.makeReadWElem(elem)
        return c_readw

    # Normal source operand read
    def makeReadElem(self, elem_op, name):
        (elem_name, elem_ext) = elem_op
        (elem_spec, dflt_elem_ext, zeroing) = self.elems[elem_name]

        if elem_ext:
            ext = elem_ext
        else:
            ext = dflt_elem_ext
        ctype = self.parser.operandTypeMap[ext]
        c_read = '\t\t%s = %s[%s];\n' % \
                  (elem_name, name, elem_spec)
        return c_read

    def makeRead(self, predRead):
        func = 'readVecRegOperand'
        if self.read_code != None:
            return self.buildReadCode(func)

        if predRead:
            rindex = '_sourceIndex++'
        else:
            rindex = '%d' % self.src_reg_idx

        name = self.base_name
        if self.is_dest and self.is_src:
            name += '_merger'

        c_read =  '\t\t%s& tmp_s%s = xc->%s(this, %s);\n' \
                % ('const TheISA::VecRegContainer', rindex, func, rindex)
        # If the parser has detected that elements are being access, create
        # the appropriate view
        if self.elemExt:
            c_read += '\t\tauto %s = tmp_s%s.as<%s>();\n' % \
                 (name, rindex, self.parser.operandTypeMap[self.elemExt])
        if self.ext:
            c_read += '\t\tauto %s = tmp_s%s.as<%s>();\n' % \
                 (name, rindex, self.parser.operandTypeMap[self.ext])
        if hasattr(self, 'active_elems'):
            if self.active_elems:
                for elem in self.active_elems:
                    c_read += self.makeReadElem(elem, name)
        return c_read

    def makeWrite(self, predWrite):
        func = 'setVecRegOperand'
        if self.write_code != None:
            return self.buildWriteCode(func)

        wb = '''
        if (traceData) {
            traceData->setData(tmp_d%d);
        }
        ''' % self.dest_reg_idx
        return wb

    def finalize(self, predRead, predWrite):
        super(VecRegOperand, self).finalize(predRead, predWrite)
        if self.is_dest:
            self.op_rd = self.makeReadW(predWrite) + self.op_rd

class VecElemOperand(Operand):
    reg_class = 'VecElemClass'

    def isReg(self):
        return 1

    def isVecElem(self):
        return 1

    def makeDecl(self):
        if self.is_dest and not self.is_src:
            return '\n\t%s %s;' % (self.ctype, self.base_name)
        else:
            return ''

    def makeConstructor(self, predRead, predWrite):
        c_src = ''
        c_dest = ''

        numAccessNeeded = 1

        if self.is_src:
            c_src = ('\n\t_srcRegIdx[_numSrcRegs++] = RegId(%s, %s, %s);' %
                    (self.reg_class, self.reg_spec, self.elem_spec))

        if self.is_dest:
            c_dest = ('\n\t_destRegIdx[_numDestRegs++] = RegId(%s, %s, %s);' %
                    (self.reg_class, self.reg_spec, self.elem_spec))
            c_dest += '\n\t_numVecElemDestRegs++;'
        return c_src + c_dest

    def makeRead(self, predRead):
        c_read = 'xc->readVecElemOperand(this, %d)' % self.src_reg_idx

        if self.ctype == 'float':
            c_read = 'bitsToFloat32(%s)' % c_read
        elif self.ctype == 'double':
            c_read = 'bitsToFloat64(%s)' % c_read

        return '\n\t%s %s = %s;\n' % (self.ctype, self.base_name, c_read)

    def makeWrite(self, predWrite):
        if self.ctype == 'float':
            c_write = 'floatToBits32(%s)' % self.base_name
        elif self.ctype == 'double':
            c_write = 'floatToBits64(%s)' % self.base_name
        else:
            c_write = self.base_name

        c_write = ('\n\txc->setVecElemOperand(this, %d, %s);' %
                  (self.dest_reg_idx, c_write))

        return c_write

class VecPredRegOperand(Operand):
    reg_class = 'VecPredRegClass'

    def __init__(self, parser, full_name, ext, is_src, is_dest):
        Operand.__init__(self, parser, full_name, ext, is_src, is_dest)
        self.parser = parser

    def isReg(self):
        return 1

    def isVecPredReg(self):
        return 1

    def makeDecl(self):
        return ''

    def makeConstructor(self, predRead, predWrite):
        c_src = ''
        c_dest = ''

        if self.is_src:
            c_src = src_reg_constructor % (self.reg_class, self.reg_spec)

        if self.is_dest:
            c_dest = dst_reg_constructor % (self.reg_class, self.reg_spec)
            c_dest += '\n\t_numVecPredDestRegs++;'

        return c_src + c_dest

    def makeRead(self, predRead):
        func = 'readVecPredRegOperand'
        if self.read_code != None:
            return self.buildReadCode(func)

        if predRead:
            rindex = '_sourceIndex++'
        else:
            rindex = '%d' % self.src_reg_idx

        c_read =  '\t\t%s& tmp_s%s = xc->%s(this, %s);\n' % (
                'const TheISA::VecPredRegContainer', rindex, func, rindex)
        if self.ext:
            c_read += '\t\tauto %s = tmp_s%s.as<%s>();\n' % (
                    self.base_name, rindex,
                    self.parser.operandTypeMap[self.ext])
        return c_read

    def makeReadW(self, predWrite):
        func = 'getWritableVecPredRegOperand'
        if self.read_code != None:
            return self.buildReadCode(func)

        if predWrite:
            rindex = '_destIndex++'
        else:
            rindex = '%d' % self.dest_reg_idx

        c_readw = '\t\t%s& tmp_d%s = xc->%s(this, %s);\n' % (
                'TheISA::VecPredRegContainer', rindex, func, rindex)
        if self.ext:
            c_readw += '\t\tauto %s = tmp_d%s.as<%s>();\n' % (
                    self.base_name, rindex,
                    self.parser.operandTypeMap[self.ext])
        return c_readw

    def makeWrite(self, predWrite):
        func = 'setVecPredRegOperand'
        if self.write_code != None:
            return self.buildWriteCode(func)

        wb = '''
        if (traceData) {
            traceData->setData(tmp_d%d);
        }
        ''' % self.dest_reg_idx
        return wb

    def finalize(self, predRead, predWrite):
        super(VecPredRegOperand, self).finalize(predRead, predWrite)
        if self.is_dest:
            self.op_rd = self.makeReadW(predWrite) + self.op_rd

class CCRegOperand(Operand):
    reg_class = 'CCRegClass'

    def isReg(self):
        return 1

    def isCCReg(self):
        return 1

    def makeConstructor(self, predRead, predWrite):
        c_src = ''
        c_dest = ''

        if self.is_src:
            c_src = src_reg_constructor % (self.reg_class, self.reg_spec)
            if self.hasReadPred():
                c_src = '\n\tif (%s) {%s\n\t}' % \
                        (self.read_predicate, c_src)

        if self.is_dest:
            c_dest = dst_reg_constructor % (self.reg_class, self.reg_spec)
            c_dest += '\n\t_numCCDestRegs++;'
            if self.hasWritePred():
                c_dest = '\n\tif (%s) {%s\n\t}' % \
                         (self.write_predicate, c_dest)

        return c_src + c_dest

    def makeRead(self, predRead):
        if (self.ctype == 'float' or self.ctype == 'double'):
            error('Attempt to read condition-code register as FP')
        if self.read_code != None:
            return self.buildReadCode('readCCRegOperand')

        int_reg_val = ''
        if predRead:
            int_reg_val = 'xc->readCCRegOperand(this, _sourceIndex++)'
            if self.hasReadPred():
                int_reg_val = '(%s) ? %s : 0' % \
                              (self.read_predicate, int_reg_val)
        else:
            int_reg_val = 'xc->readCCRegOperand(this, %d)' % self.src_reg_idx

        return '%s = %s;\n' % (self.base_name, int_reg_val)

    def makeWrite(self, predWrite):
        if (self.ctype == 'float' or self.ctype == 'double'):
            error('Attempt to write condition-code register as FP')
        if self.write_code != None:
            return self.buildWriteCode('setCCRegOperand')

        if predWrite:
            wp = 'true'
            if self.hasWritePred():
                wp = self.write_predicate

            wcond = 'if (%s)' % (wp)
            windex = '_destIndex++'
        else:
            wcond = ''
            windex = '%d' % self.dest_reg_idx

        wb = '''
        %s
        {
            %s final_val = %s;
            xc->setCCRegOperand(this, %s, final_val);\n
            if (traceData) { traceData->setData(final_val); }
        }''' % (wcond, self.ctype, self.base_name, windex)

        return wb

class ControlRegOperand(Operand):
    reg_class = 'MiscRegClass'

    def isReg(self):
        return 1

    def isControlReg(self):
        return 1

    def makeConstructor(self, predRead, predWrite):
        c_src = ''
        c_dest = ''

        if self.is_src:
            c_src = src_reg_constructor % (self.reg_class, self.reg_spec)

        if self.is_dest:
            c_dest = dst_reg_constructor % (self.reg_class, self.reg_spec)

        return c_src + c_dest

    def makeRead(self, predRead):
        bit_select = 0
        if (self.ctype == 'float' or self.ctype == 'double'):
            error('Attempt to read control register as FP')
        if self.read_code != None:
            return self.buildReadCode('readMiscRegOperand')

        if predRead:
            rindex = '_sourceIndex++'
        else:
            rindex = '%d' % self.src_reg_idx

        return '%s = xc->readMiscRegOperand(this, %s);\n' % \
            (self.base_name, rindex)

    def makeWrite(self, predWrite):
        if (self.ctype == 'float' or self.ctype == 'double'):
            error('Attempt to write control register as FP')
        if self.write_code != None:
            return self.buildWriteCode('setMiscRegOperand')

        if predWrite:
            windex = '_destIndex++'
        else:
            windex = '%d' % self.dest_reg_idx

        wb = 'xc->setMiscRegOperand(this, %s, %s);\n' % \
             (windex, self.base_name)
        wb += 'if (traceData) { traceData->setData(%s); }' % \
              self.base_name

        return wb

class MemOperand(Operand):
    def isMem(self):
        return 1

    def makeConstructor(self, predRead, predWrite):
        return ''

    def makeDecl(self):
        # Declare memory data variable.
        return '%s %s;\n' % (self.ctype, self.base_name)

    def makeRead(self, predRead):
        if self.read_code != None:
            return self.buildReadCode()
        return ''

    def makeWrite(self, predWrite):
        if self.write_code != None:
            return self.buildWriteCode()
        return ''

class PCStateOperand(Operand):
    def makeConstructor(self, predRead, predWrite):
        return ''

    def makeRead(self, predRead):
        if self.reg_spec:
            # A component of the PC state.
            return '%s = __parserAutoPCState.%s();\n' % \
                (self.base_name, self.reg_spec)
        else:
            # The whole PC state itself.
            return '%s = xc->pcState();\n' % self.base_name

    def makeWrite(self, predWrite):
        if self.reg_spec:
            # A component of the PC state.
            return '__parserAutoPCState.%s(%s);\n' % \
                (self.reg_spec, self.base_name)
        else:
            # The whole PC state itself.
            return 'xc->pcState(%s);\n' % self.base_name

    def makeDecl(self):
        ctype = 'TheISA::PCState'
        if self.isPCPart():
            ctype = self.ctype
        # Note that initializations in the declarations are solely
        # to avoid 'uninitialized variable' errors from the compiler.
        return '%s %s = 0;\n' % (ctype, self.base_name)

    def isPCState(self):
        return 1

class OperandList(object):
    '''Find all the operands in the given code block.  Returns an operand
    descriptor list (instance of class OperandList).'''
    def __init__(self, parser, code):
        self.items = []
        self.bases = {}
        # delete strings and comments so we don't match on operands inside
        for regEx in (stringRE, commentRE):
            code = regEx.sub('', code)
        # search for operands
        next_pos = 0
        while 1:
            match = parser.operandsRE.search(code, next_pos)
            if not match:
                # no more matches: we're done
                break
            op = match.groups()
            # regexp groups are operand full name, base, and extension
            (op_full, op_base, op_ext) = op
            # If is a elem operand, define or update the corresponding
            # vector operand
            isElem = False
            if op_base in parser.elemToVector:
                isElem = True
                elem_op = (op_base, op_ext)
                op_base = parser.elemToVector[op_base]
                op_ext = '' # use the default one
            # if the token following the operand is an assignment, this is
            # a destination (LHS), else it's a source (RHS)
            is_dest = (assignRE.match(code, match.end()) != None)
            is_src = not is_dest

            # see if we've already seen this one
            op_desc = self.find_base(op_base)
            if op_desc:
                if op_ext and op_ext != '' and op_desc.ext != op_ext:
                    error ('Inconsistent extensions for operand %s: %s - %s' \
                            % (op_base, op_desc.ext, op_ext))
                op_desc.is_src = op_desc.is_src or is_src
                op_desc.is_dest = op_desc.is_dest or is_dest
                if isElem:
                    (elem_base, elem_ext) = elem_op
                    found = False
                    for ae in op_desc.active_elems:
                        (ae_base, ae_ext) = ae
                        if ae_base == elem_base:
                            if ae_ext != elem_ext:
                                error('Inconsistent extensions for elem'
                                      ' operand %s' % elem_base)
                            else:
                                found = True
                    if not found:
                        op_desc.active_elems.append(elem_op)
            else:
                # new operand: create new descriptor
                op_desc = parser.operandNameMap[op_base](parser,
                    op_full, op_ext, is_src, is_dest)
                # if operand is a vector elem, add the corresponding vector
                # operand if not already done
                if isElem:
                    op_desc.elemExt = elem_op[1]
                    op_desc.active_elems = [elem_op]
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
        self.numVecDestRegs = 0
        self.numVecPredDestRegs = 0
        self.numCCDestRegs = 0
        self.numMiscDestRegs = 0
        self.memOperand = None

        # Flags to keep track if one or more operands are to be read/written
        # conditionally.
        self.predRead = False
        self.predWrite = False

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
                    elif op_desc.isVecReg():
                        self.numVecDestRegs += 1
                    elif op_desc.isVecPredReg():
                        self.numVecPredDestRegs += 1
                    elif op_desc.isCCReg():
                        self.numCCDestRegs += 1
                    elif op_desc.isControlReg():
                        self.numMiscDestRegs += 1
            elif op_desc.isMem():
                if self.memOperand:
                    error("Code block has more than one memory operand.")
                self.memOperand = op_desc

            # Check if this operand has read/write predication. If true, then
            # the microop will dynamically index source/dest registers.
            self.predRead = self.predRead or op_desc.hasReadPred()
            self.predWrite = self.predWrite or op_desc.hasWritePred()

        if parser.maxInstSrcRegs < self.numSrcRegs:
            parser.maxInstSrcRegs = self.numSrcRegs
        if parser.maxInstDestRegs < self.numDestRegs:
            parser.maxInstDestRegs = self.numDestRegs
        if parser.maxMiscDestRegs < self.numMiscDestRegs:
            parser.maxMiscDestRegs = self.numMiscDestRegs

        # now make a final pass to finalize op_desc fields that may depend
        # on the register enumeration
        for op_desc in self.items:
            op_desc.finalize(self.predRead, self.predWrite)

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
        self.items.sort(key=lambda a: a.sort_pri)

class SubOperandList(OperandList):
    '''Find all the operands in the given code block.  Returns an operand
    descriptor list (instance of class OperandList).'''
    def __init__(self, parser, code, master_list):
        self.items = []
        self.bases = {}
        # delete strings and comments so we don't match on operands inside
        for regEx in (stringRE, commentRE):
            code = regEx.sub('', code)
        # search for operands
        next_pos = 0
        while 1:
            match = parser.operandsRE.search(code, next_pos)
            if not match:
                # no more matches: we're done
                break
            op = match.groups()
            # regexp groups are operand full name, base, and extension
            (op_full, op_base, op_ext) = op
            # If is a elem operand, define or update the corresponding
            # vector operand
            if op_base in parser.elemToVector:
                elem_op = op_base
                op_base = parser.elemToVector[elem_op]
            # find this op in the master list
            op_desc = master_list.find_base(op_base)
            if not op_desc:
                error('Found operand %s which is not in the master list!'
                      % op_base)
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
        # Whether the whole PC needs to be read so parts of it can be accessed
        self.readPC = False
        # Whether the whole PC needs to be written after parts of it were
        # changed
        self.setPC = False
        # Whether this instruction manipulates the whole PC or parts of it.
        # Mixing the two is a bad idea and flagged as an error.
        self.pcPart = None

        # Flags to keep track if one or more operands are to be read/written
        # conditionally.
        self.predRead = False
        self.predWrite = False

        for op_desc in self.items:
            if op_desc.isPCPart():
                self.readPC = True
                if op_desc.is_dest:
                    self.setPC = True

            if op_desc.isPCState():
                if self.pcPart is not None:
                    if self.pcPart and not op_desc.isPCPart() or \
                            not self.pcPart and op_desc.isPCPart():
                        error("Mixed whole and partial PC state operands.")
                self.pcPart = op_desc.isPCPart()

            if op_desc.isMem():
                if self.memOperand:
                    error("Code block has more than one memory operand.")
                self.memOperand = op_desc

            # Check if this operand has read/write predication. If true, then
            # the microop will dynamically index source/dest registers.
            self.predRead = self.predRead or op_desc.hasReadPred()
            self.predWrite = self.predWrite or op_desc.hasWritePred()

# Regular expression object to match C++ strings
stringRE = re.compile(r'"([^"\\]|\\.)*"')

# Regular expression object to match C++ comments
# (used in findOperands())
commentRE = re.compile(r'(^)?[^\S\n]*/(?:\*(.*?)\*/[^\S\n]*|/[^\n]*)($)?',
        re.DOTALL | re.MULTILINE)

# Regular expression object to match assignment statements (used in
# findOperands()).  If the code immediately following the first
# appearance of the operand matches this regex, then the operand
# appears to be on the LHS of an assignment, and is thus a
# destination.  basically we're looking for an '=' that's not '=='.
# The heinous tangle before that handles the case where the operand
# has an array subscript.
assignRE = re.compile(r'(\[[^\]]+\])?\s*=(?!=)', re.MULTILINE)

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
    code = pre + (post + pre).join(flag_list) + post
    return code

# Assume all instruction flags are of the form 'IsFoo'
instFlagRE = re.compile(r'Is.*')

# OpClass constants end in 'Op' except No_OpClass
opClassRE = re.compile(r'.*Op|No_OpClass')

class InstObjParams(object):
    def __init__(self, parser, mnem, class_name, base_class = '',
                 snippets = {}, opt_args = []):
        self.mnemonic = mnem
        self.class_name = class_name
        self.base_class = base_class
        if not isinstance(snippets, dict):
            snippets = {'code' : snippets}
        compositeCode = ' '.join(list(map(str, snippets.values())))
        self.snippets = snippets

        self.operands = OperandList(parser, compositeCode)

        # The header of the constructor declares the variables to be used
        # in the body of the constructor.
        header = ''
        header += '\n\t_numSrcRegs = 0;'
        header += '\n\t_numDestRegs = 0;'
        header += '\n\t_numFPDestRegs = 0;'
        header += '\n\t_numVecDestRegs = 0;'
        header += '\n\t_numVecElemDestRegs = 0;'
        header += '\n\t_numVecPredDestRegs = 0;'
        header += '\n\t_numIntDestRegs = 0;'
        header += '\n\t_numCCDestRegs = 0;'

        self.constructor = header + \
                           self.operands.concatAttrStrings('constructor')

        self.flags = self.operands.concatAttrLists('flags')

        self.op_class = None

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
                error('InstObjParams: optional arg "%s" not recognized '
                      'as StaticInst::Flag or OpClass.' % oa)

        # Make a basic guess on the operand class if not set.
        # These are good enough for most cases.
        if not self.op_class:
            if 'IsStore' in self.flags:
                # The order matters here: 'IsFloating' and 'IsInteger' are
                # usually set in FP instructions because of the base
                # register
                if 'IsFloating' in self.flags:
                    self.op_class = 'FloatMemWriteOp'
                else:
                    self.op_class = 'MemWriteOp'
            elif 'IsLoad' in self.flags or 'IsPrefetch' in self.flags:
                # The order matters here: 'IsFloating' and 'IsInteger' are
                # usually set in FP instructions because of the base
                # register
                if 'IsFloating' in self.flags:
                    self.op_class = 'FloatMemReadOp'
                else:
                    self.op_class = 'MemReadOp'
            elif 'IsFloating' in self.flags:
                self.op_class = 'FloatAddOp'
            elif 'IsVector' in self.flags:
                self.op_class = 'SimdAddOp'
            else:
                self.op_class = 'IntAluOp'

        # add flag initialization to contructor here to include
        # any flags added via opt_args
        self.constructor += makeFlagConstructor(self.flags)

        # if 'IsFloating' is set, add call to the FP enable check
        # function (which should be provided by isa_desc via a declare)
        # if 'IsVector' is set, add call to the Vector enable check
        # function (which should be provided by isa_desc via a declare)
        if 'IsFloating' in self.flags:
            self.fp_enable_check = 'fault = checkFpEnableFault(xc);'
        elif 'IsVector' in self.flags:
            self.fp_enable_check = 'fault = checkVecEnableFault(xc);'
        else:
            self.fp_enable_check = ''

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

# Format a file include stack backtrace as a string
def backtrace(filename_stack):
    fmt = "In file included from %s:"
    return "\n".join([fmt % f for f in filename_stack])


#######################
#
# LineTracker: track filenames along with line numbers in PLY lineno fields
#     PLY explicitly doesn't do anything with 'lineno' except propagate
#     it.  This class lets us tie filenames with the line numbers with a
#     minimum of disruption to existing increment code.
#

class LineTracker(object):
    def __init__(self, filename, lineno=1):
        self.filename = filename
        self.lineno = lineno

    # Overload '+=' for increments.  We need to create a new object on
    # each update else every token ends up referencing the same
    # constantly incrementing instance.
    def __iadd__(self, incr):
        return LineTracker(self.filename, self.lineno + incr)

    def __str__(self):
        return "%s:%d" % (self.filename, self.lineno)

    # In case there are places where someone really expects a number
    def __int__(self):
        return self.lineno


#######################
#
# ISA Parser
#   parses ISA DSL and emits C++ headers and source
#

class ISAParser(Grammar):
    def __init__(self, output_dir):
        super(ISAParser, self).__init__()
        self.output_dir = output_dir

        self.filename = None # for output file watermarking/scaremongering

        # variable to hold templates
        self.templateMap = {}

        # This dictionary maps format name strings to Format objects.
        self.formatMap = {}

        # Track open files and, if applicable, how many chunks it has been
        # split into so far.
        self.files = {}
        self.splits = {}

        # isa_name / namespace identifier from namespace declaration.
        # before the namespace declaration, None.
        self.isa_name = None
        self.namespace = None

        # The format stack.
        self.formatStack = Stack(NoFormat())

        # The default case stack.
        self.defaultStack = Stack(None)

        # Stack that tracks current file and line number.  Each
        # element is a tuple (filename, lineno) that records the
        # *current* filename and the line number in the *previous*
        # file where it was included.
        self.fileNameStack = Stack()

        symbols = ('makeList', 're')
        self.exportContext = dict([(s, eval(s)) for s in symbols])

        self.maxInstSrcRegs = 0
        self.maxInstDestRegs = 0
        self.maxMiscDestRegs = 0

    def __getitem__(self, i):    # Allow object (self) to be
        return getattr(self, i)  # passed to %-substitutions

    # Change the file suffix of a base filename:
    #   (e.g.) decoder.cc -> decoder-g.cc.inc for 'global' outputs
    def suffixize(self, s, sec):
        extn = re.compile('(\.[^\.]+)$') # isolate extension
        if self.namespace:
            return extn.sub(r'-ns\1.inc', s) # insert some text on either side
        else:
            return extn.sub(r'-g\1.inc', s)

    # Get the file object for emitting code into the specified section
    # (header, decoder, exec, decode_block).
    def get_file(self, section):
        if section == 'decode_block':
            filename = 'decode-method.cc.inc'
        else:
            if section == 'header':
                file = 'decoder.hh'
            else:
                file = '%s.cc' % section
            filename = self.suffixize(file, section)
        try:
            return self.files[filename]
        except KeyError: pass

        f = self.open(filename)
        self.files[filename] = f

        # The splittable files are the ones with many independent
        # per-instruction functions - the decoder's instruction constructors
        # and the instruction execution (execute()) methods. These both have
        # the suffix -ns.cc.inc, meaning they are within the namespace part
        # of the ISA, contain object-emitting C++ source, and are included
        # into other top-level files. These are the files that need special
        # #define's to allow parts of them to be compiled separately. Rather
        # than splitting the emissions into separate files, the monolithic
        # output of the ISA parser is maintained, but the value (or lack
        # thereof) of the __SPLIT definition during C preprocessing will
        # select the different chunks. If no 'split' directives are used,
        # the cpp emissions have no effect.
        if re.search('-ns.cc.inc$', filename):
            print('#if !defined(__SPLIT) || (__SPLIT == 1)', file=f)
            self.splits[f] = 1
        # ensure requisite #include's
        elif filename == 'decoder-g.hh.inc':
            print('#include "base/bitfield.hh"', file=f)

        return f

    # Weave together the parts of the different output sections by
    # #include'ing them into some very short top-level .cc/.hh files.
    # These small files make it much clearer how this tool works, since
    # you directly see the chunks emitted as files that are #include'd.
    def write_top_level_files(self):
        # decoder header - everything depends on this
        file = 'decoder.hh'
        with self.open(file) as f:
            f.write('#ifndef __ARCH_%(isa)s_GENERATED_DECODER_HH__\n'
                    '#define __ARCH_%(isa)s_GENERATED_DECODER_HH__\n\n' %
                    {'isa': self.isa_name.upper()})
            fn = 'decoder-g.hh.inc'
            assert(fn in self.files)
            f.write('#include "%s"\n' % fn)

            fn = 'decoder-ns.hh.inc'
            assert(fn in self.files)
            f.write('namespace %s {\n#include "%s"\n}\n'
                    % (self.namespace, fn))
            f.write('\n#endif  // __ARCH_%s_GENERATED_DECODER_HH__\n' %
                    self.isa_name.upper())

        # decoder method - cannot be split
        file = 'decoder.cc'
        with self.open(file) as f:
            fn = 'base/compiler.hh'
            f.write('#include "%s"\n' % fn)

            fn = 'decoder-g.cc.inc'
            assert(fn in self.files)
            f.write('#include "%s"\n' % fn)

            fn = 'decoder.hh'
            f.write('#include "%s"\n' % fn)

            fn = 'decode-method.cc.inc'
            # is guaranteed to have been written for parse to complete
            f.write('#include "%s"\n' % fn)

        extn = re.compile('(\.[^\.]+)$')

        # instruction constructors
        splits = self.splits[self.get_file('decoder')]
        file_ = 'inst-constrs.cc'
        for i in range(1, splits+1):
            if splits > 1:
                file = extn.sub(r'-%d\1' % i, file_)
            else:
                file = file_
            with self.open(file) as f:
                fn = 'decoder-g.cc.inc'
                assert(fn in self.files)
                f.write('#include "%s"\n' % fn)

                fn = 'decoder.hh'
                f.write('#include "%s"\n' % fn)

                fn = 'decoder-ns.cc.inc'
                assert(fn in self.files)
                print('namespace %s {' % self.namespace, file=f)
                if splits > 1:
                    print('#define __SPLIT %u' % i, file=f)
                print('#include "%s"' % fn, file=f)
                print('}', file=f)

        # instruction execution
        splits = self.splits[self.get_file('exec')]
        for i in range(1, splits+1):
            file = 'generic_cpu_exec.cc'
            if splits > 1:
                file = extn.sub(r'_%d\1' % i, file)
            with self.open(file) as f:
                fn = 'exec-g.cc.inc'
                assert(fn in self.files)
                f.write('#include "%s"\n' % fn)
                f.write('#include "cpu/exec_context.hh"\n')
                f.write('#include "decoder.hh"\n')

                fn = 'exec-ns.cc.inc'
                assert(fn in self.files)
                print('namespace %s {' % self.namespace, file=f)
                if splits > 1:
                    print('#define __SPLIT %u' % i, file=f)
                print('#include "%s"' % fn, file=f)
                print('}', file=f)

        # max_inst_regs.hh
        self.update('max_inst_regs.hh',
                    '''namespace %(namespace)s {
    const int MaxInstSrcRegs = %(maxInstSrcRegs)d;
    const int MaxInstDestRegs = %(maxInstDestRegs)d;
    const int MaxMiscDestRegs = %(maxMiscDestRegs)d;\n}\n''' % self)

    scaremonger_template ='''// DO NOT EDIT
// This file was automatically generated from an ISA description:
//   %(filename)s

''';

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
        'OUTPUT', 'SIGNED', 'SPLIT', 'TEMPLATE'
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
    t_ASTERISK         = r'\*'

    # Identifiers and reserved words
    reserved_map = { }
    for r in reserved:
        reserved_map[r.lower()] = r

    def t_ID(self, t):
        r'[A-Za-z_]\w*'
        t.type = self.reserved_map.get(t.value, 'ID')
        return t

    # Integer literal
    def t_INTLIT(self, t):
        r'-?(0x[\da-fA-F]+)|\d+'
        try:
            t.value = int(t.value,0)
        except ValueError:
            error(t.lexer.lineno, 'Integer value "%s" too large' % t.value)
            t.value = 0
        return t

    # String literal.  Note that these use only single quotes, and
    # can span multiple lines.
    def t_STRLIT(self, t):
        r"(?m)'([^'])+'"
        # strip off quotes
        t.value = t.value[1:-1]
        t.lexer.lineno += t.value.count('\n')
        return t


    # "Code literal"... like a string literal, but delimiters are
    # '{{' and '}}' so they get formatted nicely under emacs c-mode
    def t_CODELIT(self, t):
        r"(?m)\{\{([^\}]|}(?!\}))+\}\}"
        # strip off {{ & }}
        t.value = t.value[2:-2]
        t.lexer.lineno += t.value.count('\n')
        return t

    def t_CPPDIRECTIVE(self, t):
        r'^\#[^\#].*\n'
        t.lexer.lineno += t.value.count('\n')
        return t

    def t_NEWFILE(self, t):
        r'^\#\#newfile\s+"[^"]*"\n'
        self.fileNameStack.push(t.lexer.lineno)
        t.lexer.lineno = LineTracker(t.value[11:-2])

    def t_ENDFILE(self, t):
        r'^\#\#endfile\n'
        t.lexer.lineno = self.fileNameStack.pop()

    #
    # The functions t_NEWLINE, t_ignore, and t_error are
    # special for the lex module.
    #

    # Newlines
    def t_NEWLINE(self, t):
        r'\n+'
        t.lexer.lineno += t.value.count('\n')

    # Comments
    def t_comment(self, t):
        r'//.*'

    # Completely ignored characters
    t_ignore = ' \t\x0c'

    # Error handler
    def t_error(self, t):
        error(t.lexer.lineno, "illegal character '%s'" % t.value[0])
        t.skip(1)

    #####################################################################
    #
    #                                Parser
    #
    # Every function whose name starts with 'p_' defines a grammar
    # rule.  The rule is encoded in the function's doc string, while
    # the function body provides the action taken when the rule is
    # matched.  The argument to each function is a list of the values
    # of the rule's symbols: t[0] for the LHS, and t[1..n] for the
    # symbols on the RHS.  For tokens, the value is copied from the
    # t.value attribute provided by the lexer.  For non-terminals, the
    # value is assigned by the producing rule; i.e., the job of the
    # grammar rule function is to set the value for the non-terminal
    # on the LHS (by assigning to t[0]).
    #####################################################################

    # The LHS of the first grammar rule is used as the start symbol
    # (in this case, 'specification').  Note that this rule enforces
    # that there will be exactly one namespace declaration, with 0 or
    # more global defs/decls before and after it.  The defs & decls
    # before the namespace decl will be outside the namespace; those
    # after will be inside.  The decoder function is always inside the
    # namespace.
    def p_specification(self, t):
        'specification : opt_defs_and_outputs top_level_decode_block'

        for f in self.splits.keys():
            f.write('\n#endif\n')

        for f in self.files.values(): # close ALL the files;
            f.close() # not doing so can cause compilation to fail

        self.write_top_level_files()

        t[0] = True

    # 'opt_defs_and_outputs' is a possibly empty sequence of def and/or
    # output statements. Its productions do the hard work of eventually
    # instantiating a GenCode, which are generally emitted (written to disk)
    # as soon as possible, except for the decode_block, which has to be
    # accumulated into one large function of nested switch/case blocks.
    def p_opt_defs_and_outputs_0(self, t):
        'opt_defs_and_outputs : empty'

    def p_opt_defs_and_outputs_1(self, t):
        'opt_defs_and_outputs : defs_and_outputs'

    def p_defs_and_outputs_0(self, t):
        'defs_and_outputs : def_or_output'

    def p_defs_and_outputs_1(self, t):
        'defs_and_outputs : defs_and_outputs def_or_output'

    # The list of possible definition/output statements.
    # They are all processed as they are seen.
    def p_def_or_output(self, t):
        '''def_or_output : name_decl
                         | def_format
                         | def_bitfield
                         | def_bitfield_struct
                         | def_template
                         | def_operand_types
                         | def_operands
                         | output
                         | global_let
                         | split'''

    # Utility function used by both invocations of splitting - explicit
    # 'split' keyword and split() function inside "let {{ }};" blocks.
    def split(self, sec, write=False):
        assert(sec != 'header' and "header cannot be split")

        f = self.get_file(sec)
        self.splits[f] += 1
        s = '\n#endif\n#if __SPLIT == %u\n' % self.splits[f]
        if write:
            f.write(s)
        else:
            return s

    # split output file to reduce compilation time
    def p_split(self, t):
        'split : SPLIT output_type SEMI'
        assert(self.isa_name and "'split' not allowed before namespace decl")

        self.split(t[2], True)

    def p_output_type(self, t):
        '''output_type : DECODER
                       | HEADER
                       | EXEC'''
        t[0] = t[1]

    # ISA name declaration looks like "namespace <foo>;"
    def p_name_decl(self, t):
        'name_decl : NAMESPACE ID SEMI'
        assert(self.isa_name == None and "Only 1 namespace decl permitted")
        self.isa_name = t[2]
        self.namespace = t[2] + 'Inst'

    # Output blocks 'output <foo> {{...}}' (C++ code blocks) are copied
    # directly to the appropriate output section.

    # Massage output block by substituting in template definitions and
    # bit operators.  We handle '%'s embedded in the string that don't
    # indicate template substitutions by doubling them first so that the
    # format operation will reduce them back to single '%'s.
    def process_output(self, s):
        s = self.protectNonSubstPercents(s)
        return substBitOps(s % self.templateMap)

    def p_output(self, t):
        'output : OUTPUT output_type CODELIT SEMI'
        kwargs = { t[2]+'_output' : self.process_output(t[3]) }
        GenCode(self, **kwargs).emit()

    def make_split(self):
        def _split(sec):
            return self.split(sec)
        return _split

    # global let blocks 'let {{...}}' (Python code blocks) are
    # executed directly when seen.  Note that these execute in a
    # special variable context 'exportContext' to prevent the code
    # from polluting this script's namespace.
    def p_global_let(self, t):
        'global_let : LET CODELIT SEMI'
        self.updateExportContext()
        self.exportContext["header_output"] = ''
        self.exportContext["decoder_output"] = ''
        self.exportContext["exec_output"] = ''
        self.exportContext["decode_block"] = ''
        self.exportContext["split"] = self.make_split()
        split_setup = '''
def wrap(func):
    def split(sec):
        globals()[sec + '_output'] += func(sec)
    return split
split = wrap(split)
del wrap
'''
        # This tricky setup (immediately above) allows us to just write
        # (e.g.) "split('exec')" in the Python code and the split #ifdef's
        # will automatically be added to the exec_output variable. The inner
        # Python execution environment doesn't know about the split points,
        # so we carefully inject and wrap a closure that can retrieve the
        # next split's #define from the parser and add it to the current
        # emission-in-progress.
        try:
            exec(split_setup+fixPythonIndentation(t[2]), self.exportContext)
        except Exception as exc:
            traceback.print_exc(file=sys.stdout)
            if debug:
                raise
            error(t.lineno(1), 'In global let block: %s' % exc)
        GenCode(self,
                header_output=self.exportContext["header_output"],
                decoder_output=self.exportContext["decoder_output"],
                exec_output=self.exportContext["exec_output"],
                decode_block=self.exportContext["decode_block"]).emit()

    # Define the mapping from operand type extensions to C++ types and
    # bit widths (stored in operandTypeMap).
    def p_def_operand_types(self, t):
        'def_operand_types : DEF OPERAND_TYPES CODELIT SEMI'
        try:
            self.operandTypeMap = eval('{' + t[3] + '}')
        except Exception as exc:
            if debug:
                raise
            error(t.lineno(1),
                  'In def operand_types: %s' % exc)

    # Define the mapping from operand names to operand classes and
    # other traits.  Stored in operandNameMap.
    def p_def_operands(self, t):
        'def_operands : DEF OPERANDS CODELIT SEMI'
        if not hasattr(self, 'operandTypeMap'):
            error(t.lineno(1),
                  'error: operand types must be defined before operands')
        try:
            user_dict = eval('{' + t[3] + '}', self.exportContext)
        except Exception as exc:
            if debug:
                raise
            error(t.lineno(1), 'In def operands: %s' % exc)
        self.buildOperandNameMap(user_dict, t.lexer.lineno)

    # A bitfield definition looks like:
    # 'def [signed] bitfield <ID> [<first>:<last>]'
    # This generates a preprocessor macro in the output file.
    def p_def_bitfield_0(self, t):
        'def_bitfield : DEF opt_signed BITFIELD ID LESS INTLIT COLON INTLIT GREATER SEMI'
        expr = 'bits(machInst, %2d, %2d)' % (t[6], t[8])
        if (t[2] == 'signed'):
            expr = 'sext<%d>(%s)' % (t[6] - t[8] + 1, expr)
        hash_define = '#undef %s\n#define %s\t%s\n' % (t[4], t[4], expr)
        GenCode(self, header_output=hash_define).emit()

    # alternate form for single bit: 'def [signed] bitfield <ID> [<bit>]'
    def p_def_bitfield_1(self, t):
        'def_bitfield : DEF opt_signed BITFIELD ID LESS INTLIT GREATER SEMI'
        expr = 'bits(machInst, %2d, %2d)' % (t[6], t[6])
        if (t[2] == 'signed'):
            expr = 'sext<%d>(%s)' % (1, expr)
        hash_define = '#undef %s\n#define %s\t%s\n' % (t[4], t[4], expr)
        GenCode(self, header_output=hash_define).emit()

    # alternate form for structure member: 'def bitfield <ID> <ID>'
    def p_def_bitfield_struct(self, t):
        'def_bitfield_struct : DEF opt_signed BITFIELD ID id_with_dot SEMI'
        if (t[2] != ''):
            error(t.lineno(1),
                  'error: structure bitfields are always unsigned.')
        expr = 'machInst.%s' % t[5]
        hash_define = '#undef %s\n#define %s\t%s\n' % (t[4], t[4], expr)
        GenCode(self, header_output=hash_define).emit()

    def p_id_with_dot_0(self, t):
        'id_with_dot : ID'
        t[0] = t[1]

    def p_id_with_dot_1(self, t):
        'id_with_dot : ID DOT id_with_dot'
        t[0] = t[1] + t[2] + t[3]

    def p_opt_signed_0(self, t):
        'opt_signed : SIGNED'
        t[0] = t[1]

    def p_opt_signed_1(self, t):
        'opt_signed : empty'
        t[0] = ''

    def p_def_template(self, t):
        'def_template : DEF TEMPLATE ID CODELIT SEMI'
        if t[3] in self.templateMap:
            print("warning: template %s already defined" % t[3])
        self.templateMap[t[3]] = Template(self, t[4])

    # An instruction format definition looks like
    # "def format <fmt>(<params>) {{...}};"
    def p_def_format(self, t):
        'def_format : DEF FORMAT ID LPAREN param_list RPAREN CODELIT SEMI'
        (id, params, code) = (t[3], t[5], t[7])
        self.defFormat(id, params, code, t.lexer.lineno)

    # The formal parameter list for an instruction format is a
    # possibly empty list of comma-separated parameters.  Positional
    # (standard, non-keyword) parameters must come first, followed by
    # keyword parameters, followed by a '*foo' parameter that gets
    # excess positional arguments (as in Python).  Each of these three
    # parameter categories is optional.
    #
    # Note that we do not support the '**foo' parameter for collecting
    # otherwise undefined keyword args.  Otherwise the parameter list
    # is (I believe) identical to what is supported in Python.
    #
    # The param list generates a tuple, where the first element is a
    # list of the positional params and the second element is a dict
    # containing the keyword params.
    def p_param_list_0(self, t):
        'param_list : positional_param_list COMMA nonpositional_param_list'
        t[0] = t[1] + t[3]

    def p_param_list_1(self, t):
        '''param_list : positional_param_list
                      | nonpositional_param_list'''
        t[0] = t[1]

    def p_positional_param_list_0(self, t):
        'positional_param_list : empty'
        t[0] = []

    def p_positional_param_list_1(self, t):
        'positional_param_list : ID'
        t[0] = [t[1]]

    def p_positional_param_list_2(self, t):
        'positional_param_list : positional_param_list COMMA ID'
        t[0] = t[1] + [t[3]]

    def p_nonpositional_param_list_0(self, t):
        'nonpositional_param_list : keyword_param_list COMMA excess_args_param'
        t[0] = t[1] + t[3]

    def p_nonpositional_param_list_1(self, t):
        '''nonpositional_param_list : keyword_param_list
                                    | excess_args_param'''
        t[0] = t[1]

    def p_keyword_param_list_0(self, t):
        'keyword_param_list : keyword_param'
        t[0] = [t[1]]

    def p_keyword_param_list_1(self, t):
        'keyword_param_list : keyword_param_list COMMA keyword_param'
        t[0] = t[1] + [t[3]]

    def p_keyword_param(self, t):
        'keyword_param : ID EQUALS expr'
        t[0] = t[1] + ' = ' + t[3].__repr__()

    def p_excess_args_param(self, t):
        'excess_args_param : ASTERISK ID'
        # Just concatenate them: '*ID'.  Wrap in list to be consistent
        # with positional_param_list and keyword_param_list.
        t[0] = [t[1] + t[2]]

    # End of format definition-related rules.
    ##############

    #
    # A decode block looks like:
    #       decode <field1> [, <field2>]* [default <inst>] { ... }
    #
    def p_top_level_decode_block(self, t):
        'top_level_decode_block : decode_block'
        codeObj = t[1]
        codeObj.wrap_decode_block('''
StaticInstPtr
%(isa_name)s::Decoder::decodeInst(%(isa_name)s::ExtMachInst machInst)
{
    using namespace %(namespace)s;
''' % self, '}')

        codeObj.emit()

    def p_decode_block(self, t):
        'decode_block : DECODE ID opt_default LBRACE decode_stmt_list RBRACE'
        default_defaults = self.defaultStack.pop()
        codeObj = t[5]
        # use the "default defaults" only if there was no explicit
        # default statement in decode_stmt_list
        if not codeObj.has_decode_default:
            codeObj += default_defaults
        codeObj.wrap_decode_block('switch (%s) {\n' % t[2], '}\n')
        t[0] = codeObj

    # The opt_default statement serves only to push the "default
    # defaults" onto defaultStack.  This value will be used by nested
    # decode blocks, and used and popped off when the current
    # decode_block is processed (in p_decode_block() above).
    def p_opt_default_0(self, t):
        'opt_default : empty'
        # no default specified: reuse the one currently at the top of
        # the stack
        self.defaultStack.push(self.defaultStack.top())
        # no meaningful value returned
        t[0] = None

    def p_opt_default_1(self, t):
        'opt_default : DEFAULT inst'
        # push the new default
        codeObj = t[2]
        codeObj.wrap_decode_block('\ndefault:\n', 'break;\n')
        self.defaultStack.push(codeObj)
        # no meaningful value returned
        t[0] = None

    def p_decode_stmt_list_0(self, t):
        'decode_stmt_list : decode_stmt'
        t[0] = t[1]

    def p_decode_stmt_list_1(self, t):
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


    # Preprocessor directives found in a decode statement list are
    # passed through to the output, replicated to all of the output
    # code streams.  This works well for ifdefs, so we can ifdef out
    # both the declarations and the decode cases generated by an
    # instruction definition.  Handling them as part of the grammar
    # makes it easy to keep them in the right place with respect to
    # the code generated by the other statements.
    def p_decode_stmt_cpp(self, t):
        'decode_stmt : CPPDIRECTIVE'
        t[0] = GenCode(self, t[1], t[1], t[1], t[1])

    # A format block 'format <foo> { ... }' sets the default
    # instruction format used to handle instruction definitions inside
    # the block.  This format can be overridden by using an explicit
    # format on the instruction definition or with a nested format
    # block.
    def p_decode_stmt_format(self, t):
        'decode_stmt : FORMAT push_format_id LBRACE decode_stmt_list RBRACE'
        # The format will be pushed on the stack when 'push_format_id'
        # is processed (see below).  Once the parser has recognized
        # the full production (though the right brace), we're done
        # with the format, so now we can pop it.
        self.formatStack.pop()
        t[0] = t[4]

    # This rule exists so we can set the current format (& push the
    # stack) when we recognize the format name part of the format
    # block.
    def p_push_format_id(self, t):
        'push_format_id : ID'
        try:
            self.formatStack.push(self.formatMap[t[1]])
            t[0] = ('', '// format %s' % t[1])
        except KeyError:
            error(t.lineno(1), 'instruction format "%s" not defined.' % t[1])

    # Nested decode block: if the value of the current field matches
    # the specified constant(s), do a nested decode on some other field.
    def p_decode_stmt_decode(self, t):
        'decode_stmt : case_list COLON decode_block'
        case_list = t[1]
        codeObj = t[3]
        # just wrap the decoding code from the block as a case in the
        # outer switch statement.
        codeObj.wrap_decode_block('\n%s\n' % ''.join(case_list),
                                  'M5_UNREACHABLE;\n')
        codeObj.has_decode_default = (case_list == ['default:'])
        t[0] = codeObj

    # Instruction definition (finally!).
    def p_decode_stmt_inst(self, t):
        'decode_stmt : case_list COLON inst SEMI'
        case_list = t[1]
        codeObj = t[3]
        codeObj.wrap_decode_block('\n%s' % ''.join(case_list), 'break;\n')
        codeObj.has_decode_default = (case_list == ['default:'])
        t[0] = codeObj

    # The constant list for a decode case label must be non-empty, and must
    # either be the keyword 'default', or made up of one or more
    # comma-separated integer literals or strings which evaluate to
    # constants when compiled as C++.
    def p_case_list_0(self, t):
        'case_list : DEFAULT'
        t[0] = ['default:']

    def prep_int_lit_case_label(self, lit):
        if lit >= 2**32:
            return 'case ULL(%#x): ' % lit
        else:
            return 'case %#x: ' % lit

    def prep_str_lit_case_label(self, lit):
        return 'case %s: ' % lit

    def p_case_list_1(self, t):
        'case_list : INTLIT'
        t[0] = [self.prep_int_lit_case_label(t[1])]

    def p_case_list_2(self, t):
        'case_list : STRLIT'
        t[0] = [self.prep_str_lit_case_label(t[1])]

    def p_case_list_3(self, t):
        'case_list : case_list COMMA INTLIT'
        t[0] = t[1]
        t[0].append(self.prep_int_lit_case_label(t[3]))

    def p_case_list_4(self, t):
        'case_list : case_list COMMA STRLIT'
        t[0] = t[1]
        t[0].append(self.prep_str_lit_case_label(t[3]))

    # Define an instruction using the current instruction format
    # (specified by an enclosing format block).
    # "<mnemonic>(<args>)"
    def p_inst_0(self, t):
        'inst : ID LPAREN arg_list RPAREN'
        # Pass the ID and arg list to the current format class to deal with.
        currentFormat = self.formatStack.top()
        codeObj = currentFormat.defineInst(self, t[1], t[3], t.lexer.lineno)
        args = ','.join(list(map(str, t[3])))
        args = re.sub('(?m)^', '//', args)
        args = re.sub('^//', '', args)
        comment = '\n// %s::%s(%s)\n' % (currentFormat.id, t[1], args)
        codeObj.prepend_all(comment)
        t[0] = codeObj

    # Define an instruction using an explicitly specified format:
    # "<fmt>::<mnemonic>(<args>)"
    def p_inst_1(self, t):
        'inst : ID DBLCOLON ID LPAREN arg_list RPAREN'
        try:
            format = self.formatMap[t[1]]
        except KeyError:
            error(t.lineno(1), 'instruction format "%s" not defined.' % t[1])

        codeObj = format.defineInst(self, t[3], t[5], t.lexer.lineno)
        comment = '\n// %s::%s(%s)\n' % (t[1], t[3], t[5])
        codeObj.prepend_all(comment)
        t[0] = codeObj

    # The arg list generates a tuple, where the first element is a
    # list of the positional args and the second element is a dict
    # containing the keyword args.
    def p_arg_list_0(self, t):
        'arg_list : positional_arg_list COMMA keyword_arg_list'
        t[0] = ( t[1], t[3] )

    def p_arg_list_1(self, t):
        'arg_list : positional_arg_list'
        t[0] = ( t[1], {} )

    def p_arg_list_2(self, t):
        'arg_list : keyword_arg_list'
        t[0] = ( [], t[1] )

    def p_positional_arg_list_0(self, t):
        'positional_arg_list : empty'
        t[0] = []

    def p_positional_arg_list_1(self, t):
        'positional_arg_list : expr'
        t[0] = [t[1]]

    def p_positional_arg_list_2(self, t):
        'positional_arg_list : positional_arg_list COMMA expr'
        t[0] = t[1] + [t[3]]

    def p_keyword_arg_list_0(self, t):
        'keyword_arg_list : keyword_arg'
        t[0] = t[1]

    def p_keyword_arg_list_1(self, t):
        'keyword_arg_list : keyword_arg_list COMMA keyword_arg'
        t[0] = t[1]
        t[0].update(t[3])

    def p_keyword_arg(self, t):
        'keyword_arg : ID EQUALS expr'
        t[0] = { t[1] : t[3] }

    #
    # Basic expressions.  These constitute the argument values of
    # "function calls" (i.e. instruction definitions in the decode
    # block) and default values for formal parameters of format
    # functions.
    #
    # Right now, these are either strings, integers, or (recursively)
    # lists of exprs (using Python square-bracket list syntax).  Note
    # that bare identifiers are trated as string constants here (since
    # there isn't really a variable namespace to refer to).
    #
    def p_expr_0(self, t):
        '''expr : ID
                | INTLIT
                | STRLIT
                | CODELIT'''
        t[0] = t[1]

    def p_expr_1(self, t):
        '''expr : LBRACKET list_expr RBRACKET'''
        t[0] = t[2]

    def p_list_expr_0(self, t):
        'list_expr : expr'
        t[0] = [t[1]]

    def p_list_expr_1(self, t):
        'list_expr : list_expr COMMA expr'
        t[0] = t[1] + [t[3]]

    def p_list_expr_2(self, t):
        'list_expr : empty'
        t[0] = []

    #
    # Empty production... use in other rules for readability.
    #
    def p_empty(self, t):
        'empty :'
        pass

    # Parse error handler.  Note that the argument here is the
    # offending *token*, not a grammar symbol (hence the need to use
    # t.value)
    def p_error(self, t):
        if t:
            error(t.lexer.lineno, "syntax error at '%s'" % t.value)
        else:
            error("unknown syntax error")

    # END OF GRAMMAR RULES

    def updateExportContext(self):

        # create a continuation that allows us to grab the current parser
        def wrapInstObjParams(*args):
            return InstObjParams(self, *args)
        self.exportContext['InstObjParams'] = wrapInstObjParams
        self.exportContext.update(self.templateMap)

    def defFormat(self, id, params, code, lineno):
        '''Define a new format'''

        # make sure we haven't already defined this one
        if id in self.formatMap:
            error(lineno, 'format %s redefined.' % id)

        # create new object and store in global map
        self.formatMap[id] = Format(id, params, code)

    def protectNonSubstPercents(self, s):
        '''Protect any non-dict-substitution '%'s in a format string
        (i.e. those not followed by '(')'''

        return re.sub(r'%(?!\()', '%%', s)

    def buildOperandNameMap(self, user_dict, lineno):
        operand_name = {}
        for op_name, val in user_dict.items():

            # Check if extra attributes have been specified.
            if len(val) > 9:
                error(lineno, 'error: too many attributes for operand "%s"' %
                      base_cls_name)

            # Pad val with None in case optional args are missing
            val += (None, None, None, None)
            base_cls_name, dflt_ext, reg_spec, flags, sort_pri, \
            read_code, write_code, read_predicate, write_predicate = val[:9]

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
            attrList = ['reg_spec', 'flags', 'sort_pri',
                        'read_code', 'write_code',
                        'read_predicate', 'write_predicate']
            if dflt_ext:
                dflt_ctype = self.operandTypeMap[dflt_ext]
                attrList.extend(['dflt_ctype', 'dflt_ext'])
            # reg_spec is either just a string or a dictionary
            # (for elems of vector)
            if isinstance(reg_spec, tuple):
                (reg_spec, elem_spec) = reg_spec
                if isinstance(elem_spec, str):
                    attrList.append('elem_spec')
                else:
                    assert(isinstance(elem_spec, dict))
                    elems = elem_spec
                    attrList.append('elems')
            for attr in attrList:
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
            operand_name[op_name] = type(cls_name, (base_cls,), tmp_dict)

        self.operandNameMap = operand_name

        # Define operand variables.
        operands = list(user_dict.keys())
        # Add the elems defined in the vector operands and
        # build a map elem -> vector (used in OperandList)
        elem_to_vec = {}
        for op in user_dict.keys():
            if hasattr(self.operandNameMap[op], 'elems'):
                for elem in self.operandNameMap[op].elems.keys():
                    operands.append(elem)
                    elem_to_vec[elem] = op
        self.elemToVector = elem_to_vec
        extensions = self.operandTypeMap.keys()

        operandsREString = r'''
        (?<!\w)      # neg. lookbehind assertion: prevent partial matches
        ((%s)(?:_(%s))?)   # match: operand with optional '_' then suffix
        (?!\w)       # neg. lookahead assertion: prevent partial matches
        ''' % ('|'.join(operands), '|'.join(extensions))

        self.operandsRE = re.compile(operandsREString, re.MULTILINE|re.VERBOSE)

        # Same as operandsREString, but extension is mandatory, and only two
        # groups are returned (base and ext, not full name as above).
        # Used for subtituting '_' for '.' to make C++ identifiers.
        operandsWithExtREString = r'(?<!\w)(%s)_(%s)(?!\w)' \
            % ('|'.join(operands), '|'.join(extensions))

        self.operandsWithExtRE = \
            re.compile(operandsWithExtREString, re.MULTILINE)

    def substMungedOpNames(self, code):
        '''Munge operand names in code string to make legal C++
        variable names.  This means getting rid of the type extension
        if any.  Will match base_name attribute of Operand object.)'''
        return self.operandsWithExtRE.sub(r'\1', code)

    def mungeSnippet(self, s):
        '''Fix up code snippets for final substitution in templates.'''
        if isinstance(s, str):
            return self.substMungedOpNames(substBitOps(s))
        else:
            return s

    def open(self, name, bare=False):
        '''Open the output file for writing and include scary warning.'''
        filename = os.path.join(self.output_dir, name)
        f = open(filename, 'w')
        if f:
            if not bare:
                f.write(ISAParser.scaremonger_template % self)
        return f

    def update(self, file, contents):
        '''Update the output file only.  Scons should handle the case when
        the new contents are unchanged using its built-in hash feature.'''
        f = self.open(file)
        f.write(contents)
        f.close()

    # This regular expression matches '##include' directives
    includeRE = re.compile(r'^\s*##include\s+"(?P<filename>[^"]*)".*$',
                           re.MULTILINE)

    def replace_include(self, matchobj, dirname):
        """Function to replace a matched '##include' directive with the
        contents of the specified file (with nested ##includes
        replaced recursively).  'matchobj' is an re match object
        (from a match of includeRE) and 'dirname' is the directory
        relative to which the file path should be resolved."""

        fname = matchobj.group('filename')
        full_fname = os.path.normpath(os.path.join(dirname, fname))
        contents = '##newfile "%s"\n%s\n##endfile\n' % \
                   (full_fname, self.read_and_flatten(full_fname))
        return contents

    def read_and_flatten(self, filename):
        """Read a file and recursively flatten nested '##include' files."""

        current_dir = os.path.dirname(filename)
        try:
            contents = open(filename).read()
        except IOError:
            error('Error including file "%s"' % filename)

        self.fileNameStack.push(LineTracker(filename))

        # Find any includes and include them
        def replace(matchobj):
            return self.replace_include(matchobj, current_dir)
        contents = self.includeRE.sub(replace, contents)

        self.fileNameStack.pop()
        return contents

    AlreadyGenerated = {}

    def _parse_isa_desc(self, isa_desc_file):
        '''Read in and parse the ISA description.'''

        # The build system can end up running the ISA parser twice: once to
        # finalize the build dependencies, and then to actually generate
        # the files it expects (in src/arch/$ARCH/generated). This code
        # doesn't do anything different either time, however; the SCons
        # invocations just expect different things. Since this code runs
        # within SCons, we can just remember that we've already run and
        # not perform a completely unnecessary run, since the ISA parser's
        # effect is idempotent.
        if isa_desc_file in ISAParser.AlreadyGenerated:
            return

        # grab the last three path components of isa_desc_file
        self.filename = '/'.join(isa_desc_file.split('/')[-3:])

        # Read file and (recursively) all included files into a string.
        # PLY requires that the input be in a single string so we have to
        # do this up front.
        isa_desc = self.read_and_flatten(isa_desc_file)

        # Initialize lineno tracker
        self.lex.lineno = LineTracker(isa_desc_file)

        # Parse.
        self.parse_string(isa_desc)

        ISAParser.AlreadyGenerated[isa_desc_file] = None

    def parse_isa_desc(self, *args, **kwargs):
        try:
            self._parse_isa_desc(*args, **kwargs)
        except ISAParserError as e:
            print(backtrace(self.fileNameStack))
            print("At %s:" % e.lineno)
            print(e)
            sys.exit(1)

# Called as script: get args from command line.
# Args are: <isa desc file> <output dir>
if __name__ == '__main__':
    ISAParser(sys.argv[2]).parse_isa_desc(sys.argv[1])
