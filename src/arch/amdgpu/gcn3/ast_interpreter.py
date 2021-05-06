# Copyright (c) 2015-2021 Advanced Micro Devices, Inc.
# All rights reserved.
#
# For use for simulation and test purposes only
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import copy
import os
import re
import sys

from ast_objects import *
from description_objects import *
from description_parser import DescriptionParser, ParseError
from hand_coded import *

from pprint import pprint, pformat

# generate code using the gem5 style
class CodeGen(object):
    def __init__(self, file):
        self.file = file
        self.line_max = 80
        self.semi_stack = ''
        self.tab = '    '
        self.indent = ''
        self.code = ''

    # write self.code out to file
    def generate(self):
        fd = open(self.file, 'w')
        fd.write(self.code)
        fd.close()

    # add an indent level
    def inc_indent(self):
        self.indent += self.tab

    # remove an indent level
    def dec_indent(self):
        self.indent = self.indent[0:-len(self.tab)]

    # add an optional semicolon after brace for matched cg_end()
    def push_semi(self, semi):
        if semi == '':
            semi = ' '
        self.semi_stack += semi

    # used by cg_end to decide if a semicolon should follow the brace
    def pop_semi(self):
        semi = self.semi_stack[-1:]
        self.semi_stack = self.semi_stack[0:-1]
        if semi == ' ':
            semi = ''
        return semi

    def smart_split(self, line):
        max = self.line_max - len(self.indent) - len('\n')
        if len(line) < max:
            self.code += '%s%s\n' % (self.indent, line)
        else:
            pad = ''
            while len(line) > max:
                high = max
                looking = True
                while looking and high > 8:
                    low = high - 8
                    for s in ['&&', '>=', '<<', '>>', ' *', ' +', ' =',
                              ' ?', ' :', ',']:
                        pos = line[low:high].find(s)
                        if pos > 0:
                            mid = low + pos + len(s)
                            self.code += '%s%s\n' % (self.indent, line[0:mid])
                            line = self.tab + line[mid:].strip()
                            looking = False
                            break
                    high -= 4
                if looking:
                    import pdb; pdb.set_trace()
            self.code += '%s%s\n' % (self.indent, line)

    # close a block with brace and optional semicolon
    # }[;]
    def cg_end(self, comment):
        self.dec_indent()
        self.code += self.indent + '}'
        semi = self.pop_semi()
        if semi:
            self.code += semi
        if comment:
            self.code += ' // %s\n' % comment

    # class <classname>[: public <base[0]> ]
    #                  [, public <base[1]> ]
    #                  [, . . . ]
    #                  [, public <base[n-1]> ]
    # {
    def cg_class(self, name, base):
        self.code += self.indent
        self.code += 'class '
        self.code += name
        b0 = base[0:1]
        if b0:
            self.code += ' : public %s' % base[0]
        if base[1:]:
            spaces = self.indent + (' ' * (len(name) + 7))
            for b in base[1:]:
                self.code += '\n%s, public %s' % (spaces, b)
        self.code += '\n%s{\n' % self.indent
        self.push_semi(';')
        self.inc_indent()

    #   <scope>:
    def cg_scope(self, scope):
        self.code += '%s%s\n' % (self.indent[0:-2], scope)

    # [<type>]
    # [<base>::]<name>([<args[0]>, <args[1]>, ... , <args[n-1]>) qual
    # {
    def cg_method(self, typ, base, name, args, ini, qual=None):
        if typ:
            self.code += '%s%s\n' % (self.indent, typ)
        if base:
            line = '%s%s::%s(' % (self.indent, base, name)
        else:
            line = '%s%s(' % (self.indent, name)
        if args:
            left = self.line_max - len(line)
            if (len(args[0]) + 2) < left:
                line += args[0]
            else:
                self.code += '%s\n' % line
                line = '%s      %s' % (self.indent, args[0])
            for a in args[1:]:
                left = self.line_max - len(line)
                if (len(a) + 2) < left:
                    line += ', %s' % a
                else:
                    self.code += '%s,\n' % line
                    line = '%s    %s' % (self.indent, a)
        if qual:
            self.code += '%s) %s\n' % (line, qual)
        else:
            self.code += '%s)\n' % line
        if ini:
            separator = '%s    : ' % self.indent
            for i in ini:
                self.code += '%s%s\n' % (separator, i)
                separator = '%s    , ' % self.indent

        self.code += '%s{\n' % self.indent
        self.push_semi('')
        self.inc_indent()

    # <typ> <base>::<name> [] = {
    #     &<base>::<entries[0]>,
    #     &<base>::<entries[1]>,
    #     . . .
    #     &<base>::<entries[n-1]>,
    # }
    def cg_table(self, typ, base, name, entries):
        self.code += '%s%s %s::%s[] = {\n' % (self.indent, typ, base, name)
        spaces = self.indent + self.tab
        for entry in entries:
            self.code += '%s&%s::%s,\n' % (spaces, base, entry)
        self.code += '%s};\n\n' % self.indent

    # union <name> {
    def cg_union(self, name):
        self.code += '%sunion %s {\n' % (self.indent, name)
        self.push_semi(';')
        self.inc_indent()

    # struct {
    #     unsigned int <field> : <size>;
    #     . . .
    # } <name>;
    #
    def cg_struct(self, name, fields):
        max = len('pad_00_31')
        for f in fields:
            sz = len(f[0])
            if sz > max:
                max = sz
        bit = 0
        self.code += '%sstruct %s {\n' % (self.indent, name)
        prefix = '%s%sunsigned int ' % (self.indent, self.tab)
        for f in fields:
            n = f[0]
            v = f[1]
            m = f[2]
            assert v >= bit, 'fields not in sorted order'
            if bit < v:
                sz = v - bit
                p = 'pad_' + str(bit)
                if sz > 1:
                    p +=  '_' + str(v - 1)
                pad = ' ' * ((max - len(p)))
                self.code += '%s%s%s : %d;\n' % (prefix, pad, p, sz)
                bit += sz
            sz = 1 + m - v
            pad = ' ' * (max - len(n))
            self.code += '%s%s%s : %d;\n' % (prefix, pad, n, sz)
            bit += sz
        self.code += '%s};\n' % self.indent

    # for (<init>; <test>; <fini>) {
    def cg_for(self, init, test, fini):
        line = '%sfor (%s;' % (self.indent, init)
        left = self.line_max - len(line)
        if (len(test) + 2) > left:
            self.code += line
            line = self.indent + self.tab
            left = self.line_max - len(line)
        line += ' %s;' % test
        if (len(fini) + 2) > left:
            self.code += line
            line = self.indent + self.tab
            left = self.line_max - len(line)
        line += ' %s) {\n' % fini
        self.code += line
        self.inc_indent()

    # if (<test>) {
    def cg_if(self, test):
        self.code += '%sif (%s) {\n' % (self.indent, test)
        self.push_semi('')
        self.inc_indent()

    # } else if (<test>) {
    def cg_else_if(self, test):
        self.dec_indent()
        self.code += '%s} else if (%s) {\n' % (self.indent, test)
        self.inc_indent()

    # } else {
    def cg_else(self):
        self.dec_indent()
        self.code += '%s} else {\n' % self.indent
        self.inc_indent()

    # <code>
    def cg_code(self, code):
        self.code += '%s%s\n' % (self.indent, code)

    # <multi-line code>
    def cg_block(self, lines):
        for line in lines:
            self.smart_split(line)

    # #include <file>
    def cg_include(self, file):
        if file[0] == '<':
            self.code += '#include %s\n' % file
        else:
            self.code += '#include "%s"\n' % file

    # namespace <namespace>
    # {
    def cg_namespace(self, namespace):
        self.code += '%snamespace %s\n' % (self.indent, namespace)
        self.code += '%s{\n' % (self.indent)
        self.inc_indent()

    # // <comment>
    def cg_comment(self, comment):
        self.code += '%s// %s\n' % (self.indent, comment)

    # gem5 fatal() call
    def cg_fatal(self, fatal_cause):
        self.code += '%sfatal("%s");\n' % (self.indent, fatal_cause)

    # blank line
    def cg_newline(self):
        self.code += '\n'

# CodeFrag is used to build up the code for one data reference
class CodeFrag(object):
    def __init__(self, k):
        self.key = k
        self.ctx = '' # context accessor fragment
        self.fld = '' # field reference fragment
        self.var = '' # variable name fragment
        self.typ = '' # variable type fragment
        self.exp = '' # general expression fragment
        self.vec = '' # vector index fragment
        self.scn = '' # section selector

    def setup(self, initList):
        assert type(initList) is list
        assert len(initList) == 6
        self.ctx = initList[0]
        self.fld = initList[1]
        self.var = initList[2]
        self.typ = initList[3]
        self.exp = initList[4]
        self.vec = initList[5]
        self.scn = ''

    def __repr__(self):
        text  = '"key":\t' + repr(self.key) + ',\n'
        text += '"ctx":\t' + repr(self.ctx) + ',\n'
        text += '"fld":\t' + repr(self.fld) + ',\n'
        text += '"var":\t' + repr(self.var) + ',\n'
        text += '"typ":\t' + repr(self.typ) + ',\n'
        text += '"exp":\t' + repr(self.exp) + ',\n'
        text += '"vec":\t' + repr(self.vec) + ',\n'
        text += '"scn":\t' + repr(self.scn) + ',\n'
        return text

# GenOne is used to build up the code for one method
class GenOne(object):
    def __init__(self, op_inst, cg, is_vec, methods, info):
        assert type(cg) is CodeGen
        self.op_inst = op_inst
        self.cg = cg
        self.decl = []
        self.vector = []
        self.scalar = []
        self.store = []
        self.modified = []
        self.tab = '    '
        self.indent = ''
        self.store_vars = []
        self.mem_vars = {}
        self.info = info
        self.mem_load = []
        if is_vec:
            self.math = self.vector
            self.decl = ['SregU64 exec;']
            #self.load = ['exec = readSpecialReg<SregU64>(gpuDynInst, '
            #             'REG_EXEC);']
            self.load = ['exec = '
                         'gpuDynInst->wavefront()->execMask().to_ullong();']
            self.load_vars = ['exec']
        else:
            self.math = self.scalar
            self.load = []
            self.load_vars = []
        self.methods = methods

    # change scalar v vector mode
    def set_vector(self, is_vec):
        if is_vec:
            self.math = self.vector
        else:
            self.math = self.scalar

    # add an indent level
    def inc_indent(self):
        self.indent += self.tab

    # remove an indent level
    def dec_indent(self):
        self.indent = self.indent[0:-len(self.tab)]

    # add to the scalar or vector code
    def add_math(self, stmt):
        while True:
            m = re.search('(vmem_\$([^$]+)\$)', stmt)
            if m is None:
                break;
            stmt = stmt.replace(m.group(1), self.mem_vars[m.group(2)])
        self.math.append(self.indent + stmt)

    # add to the scalar code
    def add_scalar(self, stmt):
        self.scalar.append(self.indent + stmt)

    def decl_type(self, var):
        assert type(var) is str
        for dcl in self.decl:
            if var in dcl:
                pos = dcl.find(' ')
                return dcl[0:pos]
        return 'SregU16'

    def add_decl(self, var, dcl):
        for d in self.decl:
            if var in d:
                return
        self.decl.append(dcl)

    def add_src_set(self, src_set):
        for s in src_set:
            assert type(s) is CodeFrag
            if s.scn == 'mem':
                if s.exp not in self.mem_vars.keys():
                    self.mem_vars[s.exp] = 'vmem_%d' % len(self.mem_vars)
                s.var = self.mem_vars[s.exp]
            if s.var in self.load_vars:
                continue
            self.load_vars.append(s.var)
            if s.vec != '':
                vreg = s.typ.replace('S', 'V')
                src_dec = '%s %s;' % (vreg, s.var)
            else:
                sreg = s.typ
                src_dec = '%s %s;' % (sreg, s.var)
            self.add_decl(s.var, src_dec)
            if s.var not in self.modified:
                shift_fld = False
                # fixup SBASE, which needs to be shifted left by 1 bit
                if s.fld == 'instData.SBASE':
                    shift_fld = True
                if s.scn == 'mem':
                    if s.ctx != 'MEM':
                        import pdb; pdb.set_trace()
                    if '+' in s.exp:
                        pos = s.exp.find('+') + 1
                        off = s.exp[pos:].strip()
                        string = 'calculateAddr<%s>(gpuDynInst, '\
                                 '%s, (%s).get(), 0);'
                        params = (self.decl_type(s.fld), s.fld, off)
                        typ1 = self.decl_type(s.var)
                        typ2 = self.decl_type(s.fld)
                        typ3 = self.decl_type(off)
                    elif ',' in s.exp:
                        args = s.exp.split(',')
                        if len(args) == 3:
                            a1 = args[1].strip()
                            a2 = args[2].strip()
                            string = 'calculateAddr<%s>(gpuDynInst, %s, '\
                                     '(%s).get(), (%s).get());'
                            params = (self.decl_type(s.fld), s.fld, a1, a2)
                            typ1 = self.decl_type(s.var)
                            typ2 = self.decl_type(s.fld)
                            typ3 = self.decl_type(a1)
                            typ4 = self.decl_type(a2)
                        else:
                            import pdb; pdb.set_trace()
                    else:
                        if 'SMEM' in self.op_inst:
                            string = 'calculateAddr<%s>(gpuDynInst, %s, '\
                                     'offset.get());'
                        elif 'FLAT' in self.op_inst:
                            string = 'calculateAddr<%s>(gpuDynInst, %s);'
                        else:
                            string = 'calculateAddr<%s>(gpuDynInst, %s, 0, 0);'
                        params = (self.decl_type(s.fld), s.fld)
                        typ1 = self.decl_type(s.var)
                        typ2 = self.decl_type(s.fld)
                    mem_read_str = 'initiateMemRead<%s>(gpuDynInst, %s);'
                    mem_read_params = (self.decl_type(s.var), s.var)
                    self.mem_load.append(mem_read_str % mem_read_params)
                elif s.ctx != '' and s.fld != '':
                    if shift_fld:
                        string = '%s = read%s<%s>(gpuDynInst, %s << 1);'
                    else:
                        string = '%s = read%s<%s>(gpuDynInst, %s);'
                    if s.ctx in SpecialCtx:
                        params = (s.var, SpecialCtx[s.ctx],
                                  self.decl_type(s.var), s.fld)
                    else:
                        params = (s.var,
                                  TypeToAccessMethod[self.decl_type(s.var)],
                                  self.decl_type(s.var), s.fld)
                    typ1 = self.decl_type(s.var)
                    typ2 = 'uint32_t'
                elif s.exp != '':
                    if shift_fld:
                        string = '%s = read%s<%s>(gpuDynInst, %s << 1);'
                    else:
                        string = '%s = read%s<%s>(gpuDynInst, %s);'
                    if s.ctx in SpecialCtx:
                        params = (s.var, SpecialCtx[s.ctx],
                                  self.decl_type(s.var), s.exp)
                    else:
                        params = (s.var,
                                  TypeToAccessMethod[self.decl_type(s.var)],
                                  self.decl_type(s.var), s.exp)
                    typ1 = self.decl_type(s.var)
                    typ2 = self.decl_type(s.exp)
                elif s.ctx != '':
                    string = '%s = read%s<%s>(gpuDynInst, REG_%s);'
                    if s.ctx in SpecialCtx:
                        params = (s.var, SpecialCtx[s.ctx],
                                  self.decl_type(s.var), s.ctx)
                    else:
                        params = (s.var,
                                  TypeToAccessMethod[self.decl_type(s.var)],
                                  self.decl_type(s.var))
                    typ1 = self.decl_type(s.var)
                elif s.fld != '':
                    string = '%s = %s;'
                    params = (s.var, s.fld)
                else:
                    continue
                self.load.append(string % params)

    def add_dst_set(self, dst_set):
        src_set = []
        for d in dst_set:
            assert type(d) is CodeFrag
            if d.key == 'src':
                src_set.append(d)
        if src_set:
            self.add_src_set(src_set)

        for d in dst_set:
            assert type(d) is CodeFrag
            if d.key == 'dst':
                if d.scn == 'mem':
                    if d.exp not in self.mem_vars.keys():
                        self.mem_vars[d.exp] = 'vmem_%d' % len(self.mem_vars)
                    d.var = self.mem_vars[d.exp]
                # handle destination declarations
                if d.var in self.store_vars:
                    continue
                self.store_vars.append(d.var)
                if d.vec != '':
                    vreg = d.typ.replace('S', 'V')
                    dst_dec = '%s %s;' % (vreg, d.var)
                    # vector destination regs are also source regs
                    src_set.append(d)
                else:
                    sreg = d.typ
                    dst_dec = '%s %s;' % (sreg, d.var)
                self.add_decl(d.var, dst_dec)

                # write back destinations
                if d.scn == 'mem':
                    if d.ctx != 'MEM':
                        import pdb; pdb.set_trace()
                    if '+' in d.exp:
                        pos = d.exp.find('+') + 1
                        off = d.exp[pos:].strip()
                        string = 'writeMem<%s>(gpuDynInst, %s, (%s)'\
                                 '.get(), %s);'
                        params = (self.decl_type(d.var), d.fld, off, d.var)
                        typ1 = self.decl_type(d.var)
                        typ2 = self.decl_type(d.fld)
                        typ3 = self.decl_type(off)
                    else:
                        string = 'writeMem<%s, %s>(gpuDynInst, %s, 0, %s);'
                        params = (self.decl_type(d.var), self.decl_type(d.fld),
                                  d.fld, d.var)
                        typ1 = self.decl_type(d.var)
                        typ2 = self.decl_type(d.fld)
                elif d.ctx != '' and d.fld != '':
                    if d.fld[0] == '-':
                        string = 'write%s<%s>(gpuDynInst, REG_%s, %s);'
                        if d.ctx in SpecialCtx:
                            params = (SpecialCtx[d.ctx], self.decl_type(d.var),
                                      d.ctx, d.var)
                        else:
                            params = (d.fld[1:], self.decl_type(d.var), d.var)
                        typ1 = self.decl_type(d.var)
                    else:
                        string = 'write%s<%s>(gpuDynInst, %s, %s);'
                        if d.ctx in SpecialCtx:
                            params = (SpecialCtx[d.ctx], self.decl_type(d.var),
                                      d.fld, d.var)
                        else:
                            params =
                                (TypeToAccessMethod[self.decl_type(d.var)],
                                 self.decl_type(d.var), d.fld, d.var)
                        typ1 = self.decl_type(d.var)
                        typ2 = 'uint32_t'
                elif d.exp != '':
                    string = 'write%s<%s>(gpuDynInst, %s, %s);'
                    if d.ctx in SpecialCtx:
                        params = (SpecialCtx[d.ctx], self.decl_type(d.var),
                                  d.exp, d.var)
                    else:
                        params = (d.ctx, self.decl_type(d.var), d.exp, d.var)
                    typ1 = self.decl_type(d.var)
                    typ2 = self.decl_type(d.exp)
                elif d.ctx != '':
                    string = 'write%s<%s>(gpuDynInst, REG_%s, %s);'
                    if d.ctx in SpecialCtx:
                        params = (SpecialCtx[d.ctx], self.decl_type(d.var),
                                  d.ctx, d.var)
                    else:
                        params = (d.ctx, self.decl_type(d.var), d.var)
                    typ1 = self.decl_type(d.var)
                else:
                    continue
                self.store.append(string % params)

    # once all he code for a single execute() method is built up
    # the C++ method is generated. memory ops are a special case
    # because of the 3-phase nature of their execution. to fully
    # execute a memory operation the instruction class must
    # implement the following methods:
    #
    # execute()     - issues the reqest to the proper memory pipe,
    #                 i.e., global/local
    #
    # initiateAcc() - builds a memory request/packet and sends the
    #                 req to memory
    #
    # completeAcc() - returned data are written back to the
    #                 register file
    def finish(self):
        self.info.decl = self.decl

        is_store = False
        is_load = False
        is_atomic = False

        if ('OPF_MEM_STORE' in self.info.flags or 'DS_WRITE_B32' in
            self.op_inst or 'DS_WRITE_B64' in self.op_inst):
            is_store = True
        elif 'OPF_MEM_ATOMIC' in self.info.flags:
            is_atomic = True
        elif ('LOAD' in self.op_inst or 'DS_READ_B32' in self.op_inst or
              'DS_READ_B64' in self.op_inst):
            is_load = True

        if is_store or is_load:
            is_smem = 'SMEM' in self.op_inst
            is_flat_mem = 'FLAT' in self.op_inst
            is_ds_mem = 'DS' in self.op_inst

            self.cg.cg_code('Wavefront *wf = gpuDynInst->wavefront();')
            #if not is_flat_mem:
            self.cg.cg_code('gpuDynInst->execUnitId = wf->execUnitId;')
            if not is_smem:
                self.cg.cg_code('gpuDynInst->exec_mask = '
                                'gpuDynInst->wavefront()->execMask();')
            self.cg.cg_code('gpuDynInst->latency.init(&gpuDynInst'\
                            '->computeUnit()->shader->tick_cnt);')
            if is_ds_mem:
                self.cg.cg_code('gpuDynInst->latency.set(gpuDynInst'\
                                '->computeUnit()->cyclesToTicks(Cycles(24)));')
            else:
                self.cg.cg_code('gpuDynInst->latency.set(gpuDynInst'\
                                '->computeUnit()->clockPeriod());')
            self.cg.cg_newline()
            if is_smem:
                self.cg.cg_if('instData.IMM')
                self.cg.cg_code('offset = extData.OFFSET;')
                self.cg.cg_else()
                self.cg.cg_code('offset = readScalarReg<SregU32>(gpuDynInst,'
                                'extData.OFFSET);')
                self.cg.cg_end('if')
                self.cg.cg_newline()

            self.cg.cg_block(self.load)

            if is_flat_mem and is_store:
                has_vgpr_addr = False
                for load_entry in self.load:
                    if 'vgpr_addr' in load_entry:
                        has_vgpr_addr = True
                if has_vgpr_addr:
                    self.cg.cg_newline()
                    self.cg.cg_code('typedef decltype(vgpr_src)::RegType '
                                    'RegType;')
                    self.cg.cg_newline()
                    self.cg.cg_code('assert(!(sizeof(RegType) % '
                                    'sizeof(uint32_t))')
                    self.cg.inc_indent()
                    self.cg.cg_code(' || sizeof(RegType) < sizeof(uint32_t));')
                    self.cg.dec_indent()
                    self.cg.cg_newline()
                    self.cg.cg_code('int num_words = sizeof(RegType) / '
                                    'sizeof(uint32_t);')
                    self.cg.cg_newline()
                    self.cg.cg_for('int lane = 0', 'lane < '
                                   'wf->computeUnit->wfSize()',
                                   '++lane')
                    self.cg.cg_if('gpuDynInst->wavefront()->execMask()[lane]')
                    self.cg.cg_for('int i = 0', 'i < num_words', '++i')
                    self.cg.cg_code('((uint32_t*)gpuDynInst->d_data)')
                    self.cg.inc_indent()
                    self.cg.cg_code('[i * wf->computeUnit->wfSize() + lane] =')
                    self.cg.inc_indent()
                    self.cg.cg_code('vgpr_src.getDword(i, lane);')
                    self.cg.dec_indent()
                    self.cg.dec_indent()
                    self.cg.cg_end('for')
                    self.cg.cg_end('if')
                    self.cg.cg_end('for')
                    self.cg.cg_newline()
                    self.cg.cg_code('calculateAddr<VregU64>(gpuDynInst, '
                                    'vgpr_addr);')
            if is_ds_mem and is_store:
                has_vgpr_addr = False
                for load_entry in self.load:
                    if 'vgpr_a' in load_entry:
                        has_vgpr_addr = True
                if has_vgpr_addr:
                    self.cg.cg_newline()
                    self.cg.cg_code('typedef decltype(vgpr_d0)::RegType '
                                    'RegType;')
                    self.cg.cg_newline()
                    self.cg.cg_code('assert(!(sizeof(RegType) % '
                                    'sizeof(uint32_t))')
                    self.cg.inc_indent()
                    self.cg.cg_code(' || sizeof(RegType) < sizeof(uint32_t));')
                    self.cg.dec_indent()
                    self.cg.cg_newline()
                    self.cg.cg_code('int num_words = sizeof(RegType) / '
                                    'sizeof(uint32_t);')
                    self.cg.cg_newline()
                    self.cg.cg_for('int lane = 0', 'lane < '
                                   'wf->computeUnit->wfSize()',
                                   '++lane')
                    self.cg.cg_if('gpuDynInst->wavefront()->execMask()[lane]')
                    self.cg.cg_for('int i = 0', 'i < num_words', '++i')
                    self.cg.cg_code('((uint32_t*)gpuDynInst->d_data)')
                    self.cg.inc_indent()
                    self.cg.cg_code('[i * wf->computeUnit->wfSize() + lane] =')
                    self.cg.inc_indent()
                    self.cg.cg_code('vgpr_d0.getDword(i, lane);')
                    self.cg.dec_indent()
                    self.cg.dec_indent()
                    self.cg.cg_end('for')
                    self.cg.cg_end('if')
                    self.cg.cg_end('for')
                    self.cg.cg_newline()
                    self.cg.cg_code('calculateAddr<VregU32>(gpuDynInst, '
                                    'vgpr_a, 0, 0);')

            self.cg.cg_newline()

            # generate execute(), i.e., issue to appropriate memory pipe
            if is_smem:
                self.cg.cg_code('gpuDynInst->computeUnit()->scalarMemoryPipe.')
                self.cg.inc_indent()
                self.cg.cg_code('getGMReqFIFO().push(gpuDynInst);')
                self.cg.dec_indent()
                self.cg.cg_newline()
                if is_load:
                    self.cg.cg_code('wf->scalarRdGmReqsInPipe--;')
                    self.cg.cg_code('wf->scalarOutstandingReqsRdGm++;')
                else:
                    self.cg.cg_code('wf->scalarWrGmReqsInPipe--;')
                    self.cg.cg_code('wf->scalarOutstandingReqsWrGm++;')
            elif is_flat_mem:
                self.cg.cg_if('gpuDynInst->executedAs() == enums::SC_GLOBAL')
                self.cg.cg_code('gpuDynInst->computeUnit()->globalMemoryPipe.')
                self.cg.inc_indent()
                self.cg.cg_code('getGMReqFIFO().push(gpuDynInst);')
                self.cg.dec_indent()
                if is_load:
                    self.cg.cg_code('wf->rdGmReqsInPipe--;')
                    self.cg.cg_code('wf->outstandingReqsRdGm++;')
                else:
                    self.cg.cg_code('wf->wrGmReqsInPipe--;')
                    self.cg.cg_code('wf->outstandingReqsWrGm++;')
                self.cg.cg_else()
                self.cg.cg_code('assert(false);')
                self.cg.cg_end('else')
                self.cg.cg_newline()
            elif is_ds_mem:
                self.cg.cg_code('gpuDynInst->computeUnit()->localMemoryPipe.')
                self.cg.inc_indent()
                self.cg.cg_code('getLMReqFIFO().push(gpuDynInst);')
                self.cg.dec_indent()
                self.cg.cg_newline()
                if is_load:
                    self.cg.cg_code('wf->rdLmReqsInPipe--;')
                    self.cg.cg_code('wf->outstandingReqsRdLm++;')
                else:
                    self.cg.cg_code('wf->wrLmReqsInPipe--;')
                    self.cg.cg_code('wf->outstandingReqsWrLm++;')
            else:
                self.cg.cg_if('isLocalMem()')
                self.cg.cg_code('gpuDynInst->computeUnit()->localMemoryPipe.')
                self.cg.inc_indent()
                self.cg.cg_code('getLMReqFIFO().push(gpuDynInst);')
                self.cg.dec_indent()
                self.cg.cg_else()
                self.cg.cg_code('gpuDynInst->computeUnit()->globalMemoryPipe.')
                self.cg.inc_indent()
                self.cg.cg_code('getGMReqFIFO().push(gpuDynInst);')
                self.cg.dec_indent()
                self.cg.cg_end('if')
                self.cg.cg_newline()
            self.cg.cg_code('gpuDynInst->wavefront()->outstandingReqs++;')
            self.cg.cg_code('gpuDynInst->wavefront()'\
                            '->validateRequestCounters();')
            self.cg.cg_end('execute')
            self.cg.cg_newline()

            # generate initiateAcc()
            self.cg.cg_method('void', self.op_inst, 'initiateAcc',
                              ['GPUDynInstPtr gpuDynInst'], [])

            #if self.mem_load and 'SMEM' in self.op_inst:
            if self.mem_load:
                self.cg.cg_block(self.mem_load);

            if is_store:
                if self.vector:
                    self.cg.cg_for('int t = 0', 'exec != 0', 't++, exec >>= 1')
                    self.cg.cg_if('(exec & 1) != 0')
                    self.cg.cg_block(self.vector)
                    self.cg.cg_end('if') # cg_if
                    self.cg.cg_end('for') # cg_for

                self.cg.cg_block(self.scalar)
                self.cg.cg_block(self.store)

            self.cg.cg_end('initiateAcc')
            self.cg.cg_newline()

            # generate completeAcc()
            self.cg.cg_method('void', self.op_inst, 'completeAcc',
                              ['GPUDynInstPtr gpuDynInst'], [])

            if is_load:
                if self.vector:
                    self.cg.cg_for('int t = 0', 'exec != 0', 't++, exec >>= 1')
                    self.cg.cg_if('(exec & 1) != 0')
                    self.cg.cg_block(self.vector)
                    self.cg.cg_end('if') # cg_if
                    self.cg.cg_end('for') # cg_for

                self.cg.cg_block(self.scalar)
                self.cg.cg_block(self.store)

        else:
            self.cg.cg_block(self.load)

            if self.mem_load:
                self.cg.cg_block(self.mem_load);

            if self.vector:
                self.cg.cg_for('int t = 0', 'exec != 0', 't++, exec >>= 1')
                self.cg.cg_if('(exec & 1) != 0')
                self.cg.cg_block(self.vector)
                self.cg.cg_end('if') # cg_if
                self.cg.cg_end('for') # cg_for

            self.cg.cg_block(self.scalar)
            self.cg.cg_block(self.store)

        if self.store:
            if is_load or is_store:
                self.cg.cg_end('completeAcc')
            else:
                self.cg.cg_end('execute')
            return True
        else:
            return False

# a list where list[n] is always list[n] and
# the unreferenced elements have a default fill value
class IndexedList(object):
    def __init__(self, size, fill):
        self.list = []
        self.size = size
        self.fill = fill
        self.next_index = 0
        for i in range(0, size):
            self.list.append(fill)
    def grow(self, index, fill):
        old_size = self.size
        for i in range(old_size, index):
            self.list.append(self.fill)
        self.list.append(fill)
        self.size = index + 1
    def __len__(self):
        return self.size
    def __iter__(self):
        self.next_index = 0
        return self
    def next(self):
        while True:
            if self.next_index >= self.size:
                self.next_index = 0
                raise StopIteration
            entry = self.list[self.next_index]
            self.next_index += 1
            if entry != None:
                return entry
    def __getitem__(self, index):
        if index >= self.size:
            self.grow(index, self.fill)
        return self.list[index]
    def __setitem__(self, index, value):
        if index >= self.size:
            self.grow(index, value)
        else:
            self.list[index] = value
    def __repr__(self):
        text = '[\n'
        for i in range(0, self.size):
            text += '\t' + str(i) + ' :\t' + repr(self.list[i]) + '\n'
        text += ']'
        return text

# Abstract Syntax Tree Interpreter
class RefinedOpInfo(object):
    def __init__(self, name, inst, encode, op_typ):
        self.name = name
        self.inst = inst.name
        self.enc = encode.name
        self.sub_enc = op_typ.sub_enc
        self.desc = op_typ.desc
        self.flags = op_typ.flags
        self.num_dst = 0
        self.num_src = 0
        self.dst = []
        self.src = []
        self.decl = []

    def __repr__(self):
        text  = '"name":\t' + repr(self.name) + ',\n'
        text += '"enc":\t' + repr(self.enc) + ',\n'
        text += '"sub enc":\t' + repr(self.sub_enc) + ',\n'
        text += '"desc":\t' + repr(self.desc) + ',\n'
        text += '"flags":\t' + repr(self.flags) + ',\n'
        text += '"num_dst":\t' + repr(self.num_dst) + ',\n'
        text += '"num_src":\t' + repr(self.num_src) + ',\n'
        text += '"dst":\t' + repr(self.dst) + ',\n'
        text += '"src":\t' + repr(self.src) + '\n'
        return text

    def override(self, operand):
        assert type(operand) is Operand
        if operand.num_dst > 0:
            self.num_dst = operand.num_dst
        if operand.num_src > 0:
            self.num_src = operand.num_src
        for od in operand.dst:
            match = False
            for sd in self.dst:
                if sd.match(od):
                    sd.override(od)
                    match = True
            if not match:
                self.dst.append(copy.copy(od))
        for os in operand.src:
            match = False
            for ss in self.src:
                if ss.match(os):
                    ss.override(os)
                    match = True
            if not match:
                self.src.append(copy.copy(os))

class AstInterpreter(object):
    def __init__(self):
        self.high_bits = 9
        self.max_bits = 6
        self.prefix = ''
        self.constant = {}
        self.type_by_value = {}
        self.decode_info = {}
        self.enc_by_name = {}
        self.decode_tables = {}
        self.inst_by_name = {}
        self.inst_formats = {}
        self.inst_fields = {}
        self.inst_by_optype = {}
        self.op_types_seen = []
        self.invalid_type = TypeClause()
        self.invalid_type.name = 'invalid'
        self.invalid_type.desc = ['invalid']
        self.default_type = TypeClause()
        self.default_type.name = 'default'
        self.default_type.desc = ['default']
        main_decode_table = IndexedList(512, 'subDecode_invalid')
        self.decode_tables['tableDecodePrimary'] = main_decode_table
        self.inst_with_encodings = []
        self.refined_op_info = []
        self.desc_parser = DescriptionParser()
        self.look_for_comma_before_equal = False
        self.ref_op_info_fixup = ['DS']
        self.methods = []

    def bits_info(self, pattern, bits):
        care_bits = 0
        match = 0
        for b in range(len(pattern) - 1, -1, -1):
            if pattern[b] == '0':
                care_bits += 1
            elif pattern[b] == '1':
                match += 1 << care_bits
                care_bits += 1
        shift = bits - care_bits
        base = match << shift
        copy = 1 << shift
        return (base, copy)

    def handle_const(self, inst):
        for clause in inst.clauses:
            self.constant[clause.name] = clause.value

    def get_type_value_desc(self, type_value, type_list):
        for t in type_list:
            if t.name == type_value:
                return t.desc
        names = []
        for k in self.type_by_value.keys():
            tbv = self.type_by_value[k]
            for t in range(0, len(tbv)):
                if tbv[t] and tbv[t].name == type_value:
                    return tbv[t].desc
        import pdb; pdb.set_trace()
        return ['get_type_value_desc(%s)' % (type_value)]

    def handle_type(self, inst):
        type_list = IndexedList(0, None)
        regexp = re.compile('get_type_value_desc\(([^\)]+)\)')
        for clause in inst.clauses:
            assert type(clause) is TypeClause
            if clause.v_max != 0:
                # handle 'ID = NUMBER:NUMBER' clauses
                for v in range(clause.value, clause.v_max + 1):
                    newc = copy.deepcopy(clause)
                    newc.name = clause.name + str(v)
                    newc.v_max = 0
                    newc.value = v
                    type_list[v] = newc
            elif clause.var == False:
                if clause.name != '':
                    # handle 'ID = NUMBER' clauses
                    newc = copy.deepcopy(clause)
                    if newc.desc:
                        found = regexp.match(newc.desc[0])
                        if found:
                            tval = found.group(1)
                            tlst = type_list
                            newc.desc = self.get_type_value_desc(tval, tlst)
                    type_list[newc.value] = newc
                elif clause.type != '':
                    # handle 'type ID' clauses
                    for t in self.type_by_value[clause.type]:
                        type_list[t.value] = t
            else:
                # handle $(ID} substitution
                orig_name = clause.name
                a = orig_name.find('${')
                b = orig_name.find('}')
                var = orig_name[a:b+1]
                vartype = self.type_by_value[var[2:-1]]
                assert type(vartype) is IndexedList
                for t in vartype:
                    assert type(t) is TypeClause
                    newc = copy.deepcopy(clause)
                    newc.name = orig_name.replace(var, t.name)
                    newc.desc = []
                    for d in clause.desc:
                        edited = d.replace(var, t.desc[0])
                        if self.look_for_comma_before_equal:
                            m = re.match('([^,]+(,[^,=]+)+)=', edited)
                            if m:
                                # import pdb; pdb.set_trace()
                                orig = m.group(1).strip()
                                repl = '{%s}' % orig
                                edited = edited.replace(orig, repl)
                        newc.desc.append(edited)
                    newc.value = clause.value + t.value
                    type_list[newc.value] = newc
        self.type_by_value[inst.name] = type_list

    # decode an encoding with an OP field
    def decode_enc_op(self, inst, enc_field, op_field):
        assert type(inst) is InstBlock
        assert type(enc_field) is InstField
        assert type(op_field) is InstField
        op_type = self.type_by_value[op_field.type]
        assert type(op_type) is IndexedList
        encode = self.enc_by_name[enc_field.enc]
        assert type(encode) is EncodingBlock
        if op_field.type not in self.inst_by_optype.keys():
            self.inst_by_optype[op_field.type] = inst.name
        (base, copy) = self.bits_info(encode.bits, self.high_bits)
        low_bit = 32 - self.high_bits
        if op_field.value >= low_bit:
            shift = op_field.value - low_bit
            c = 1 << shift
            for t in op_type:
                assert type(t) is TypeClause
                if 'OPF_INTERNAL' in t.flags:
                    continue
                b = base + (t.value << shift)
                n = 'decode_' + op_field.type + '__' + t.name
                self.decode_info[n] = [ op_field.type, t ]
                for i in range(b, b + c):
                    self.decode_tables['tableDecodePrimary'][i] = n
        elif op_field.v_max >= low_bit:
            blk_bits = low_bit - op_field.value
            c = 1 << blk_bits
            n = 'decode_' + op_field.type + '__invalid'
            self.decode_info[n] = [ op_field.type, self.invalid_type ]
            decode_table = IndexedList(c, n)
            block = 0
            for i in range(base, base + copy):
                valid = 0
                for t in op_type:
                    assert type(t) is TypeClause
                    if 'OPF_INTERNAL' in t.flags:
                        continue
                    if op_field.type == 'OPU_VOP3':
                        if 'OPF_NOVOP3' in t.flags:
                            continue
                    if (t.value >> blk_bits) == block:
                        valid += 1
                        n = 'decode_' + op_field.type + '__' + t.name
                        self.decode_info[n] = [ op_field.type, t ]
                        decode_table[t.value] = n
                if valid > 0:
                    # make sure table grows to a multiple of
                    # block size by referencing last entry
                    n = decode_table[((block + 1) << blk_bits) - 1]
                    #
                    n = 'tableSubDecode_' +  op_field.type
                    self.decode_tables[n] = decode_table
                    n = 'subDecode_' +  op_field.type
                    self.decode_tables['tableDecodePrimary'][i] = n
                else:
                    n = 'decode_' + op_field.type + '__invalid'
                    self.decode_info[n] = [ op_field.type, self.invalid_type ]
                    self.decode_tables['tableDecodePrimary'][i] = n
                block += 1
        else:
            op_bits = 1 + op_field.v_max - op_field.value
            for i in range(base, base + copy):
                c = 1 << op_bits
                n = 'decode_' + op_field.type + '__invalid'
                self.decode_info[n] = [ op_field.type, self.invalid_type ]
                decode_table = IndexedList(c, n)
                valid = 0
                for t in op_type:
                    assert type(t) is TypeClause
                    if 'OPF_INTERNAL' in t.flags:
                        continue
                    valid += 1
                    n = 'decode_' + op_field.type + '__' + t.name
                    self.decode_info[n] = [ op_field.type, t ]
                    decode_table[t.value] = n
                if valid > 0:
                    n = 'tableSubDecode_' + op_field.type
                    self.decode_tables[n] = decode_table
                    n = 'subDecode_' + op_field.type
                    self.decode_tables['tableDecodePrimary'][i] = n
                else:
                    n = decode_table[0]
                    self.decode_tables['tableDecodePrimary'][i] = n

    def find_relevant_operand(self, operands, op_typ):
        default_operand = None
        for operand in operands:
            if operand.parent_enc and operand.sub_enc:
                if operand.parent_enc == op_typ.parent_enc:
                    if operand.sub_enc == op_typ.sub_enc:
                        return operand
                continue
            if operand.parent_enc:
                if operand.parent_enc == op_typ.parent_enc:
                    return operand
                continue
            if operand.sub_enc:
                if operand.sub_enc == op_typ.sub_enc:
                    return operand
                if operand.sub_enc == 'NEVER':
                    # NEVER indicates a deeper set of operands
                    #       which share this operand's attributes
                    if operand.operands:
                        sub = self.find_relevant_operand(operand.operands,
                                                         op_typ)
                        if sub:
                            merge = copy.deepcopy(operand)
                            merge.override(sub)
                            return merge
                continue
            if operand.flags:
                if operand.flags[0] in op_typ.flags:
                    return operand
                continue
            default_operand = operand
        return default_operand

    # return True if when clause matches op type
    def when_match(self, when, op_typ):
        if when.left == 'flags':
            for f in when.right:
                if f not in op_typ.flags:
                    return False
            return True
        return False

    def refine_op_info(self, ref_op_info, encode, op_typ):
        assert type(ref_op_info) is RefinedOpInfo
        assert type(encode) is EncodingBlock
        assert type(op_typ) is TypeClause
        relevant_operand = self.find_relevant_operand(encode.operands, op_typ)
        if not relevant_operand:
             return
        ref_op_info.override(relevant_operand)
        if op_typ.flags:
            for when in relevant_operand.when:
                if self.when_match(when, op_typ):
                    ref_op_info.override(when.operand)
        sub_operands = relevant_operand.operands
        if sub_operands:
            sub_operand = self.find_relevant_operand(sub_operands, op_typ)
            if sub_operand:
                ref_op_info.override(sub_operand)
        if op_typ.size >= 0 or encode.size >= 0:
            default_size = op_typ.size
            if default_size < 0:
                default_size = encode.size
            for d in ref_op_info.dst:
                if not d.fmt:
                    if d.size < 0:
                        d.size = default_size
            for s in ref_op_info.src:
                if not s.fmt:
                    if s.size < 0:
                        s.size = default_size
        if encode.name in self.ref_op_info_fixup:
            suffix = op_typ.name[-3:]
            if suffix not in SuffixToFmt.keys():
                return
            fmt = SuffixToFmt[suffix]
            for d in ref_op_info.dst:
                d.fmt = fmt
            for s in ref_op_info.src:
                if s.index != 0:
                    s.fmt = fmt

    # generate instructions for an encoding with an OP field
    def gen_inst(self, inst_tag, inst, encode, op_typ):
        assert type(inst_tag) is str
        assert type(inst) is InstBlock
        assert type(encode) is EncodingBlock
        assert type(op_typ) is TypeClause
        if 'OPF_INTERNAL' in op_typ.flags:
            return
        if encode.name == 'VOP3':
            if 'OPF_NOVOP3' in op_typ.flags:
                return
        ref_op_info = RefinedOpInfo(inst_tag, inst, encode, op_typ)
        self.refine_op_info(ref_op_info, encode, op_typ)
        self.refined_op_info.append(ref_op_info)

    # generate instructions for an encoding with an OP field
    def gen_inst_enc_op(self, inst, enc_field, op_field):
        op_type = self.type_by_value[op_field.type]
        encode = self.enc_by_name[enc_field.enc]
        for t in op_type:
            inst_tag = enc_field.enc + '__' + t.name
            self.gen_inst(inst_tag, inst, encode, t)

    # an encoding with an OP field
    def handle_enc_op(self, inst, enc_field, op_field):
        if op_field.type not in self.op_types_seen:
            self.op_types_seen.append(op_field.type)
            self.handle_parent_enc(inst, enc_field, op_field)
            self.decode_enc_op(inst, enc_field, op_field)
            self.gen_inst_enc_op(inst, enc_field, op_field)

    # decode an encoding with no OP field
    def decode_enc_no_op(self, inst, enc_field):
        encode = self.enc_by_name[enc_field.enc]
        (base, copy) = self.bits_info(encode.bits, self.high_bits)
        low_bit = 32 - self.high_bits
        n = 'decode_OP_' + enc_field.enc
        self.decode_info[n] = [ 'OP_' + enc_field.enc, self.default_type ]
        for i in range(base, base + copy):
            self.decode_tables['tableDecodePrimary'][i] = n

    # generate instructions for an encoding with no OP field
    def gen_inst_enc_no_op(self, inst, enc_field):
        op_field = InstField()
        op_field.tag = 'virtual'
        op_field.name = 'OP'
        op_field.desc = 'default OP field for encodings with no OP field'
        op_field.type = 'OP_' + enc_field.enc
        self.gen_inst_enc_op(inst, enc_field, op_field)

    # an encoding with no OP field
    def handle_enc_no_op(self, inst, enc_field):
        self.decode_enc_no_op(inst, enc_field)
        self.gen_inst_enc_no_op(inst, enc_field)


    def handle_parent_enc_search_replace(self, operands, op_type):
        for opr in operands:
            if opr.sub_enc == 'NEVER':
                self.handle_parent_enc_search_replace(opr.operands, op_type)
            elif opr.parent_enc != '':
                # add '_' after first character for OFFSET/COUNT prefix
                tag = opr.parent_enc[0] + '_' + opr.parent_enc[1:]
                match = tag + '_OFFSET'
                offset = 0
                found = False
                for t in op_type:
                    if t.name == match:
                        offset = t.value
                        found = True
                if found != False:
                    op = 'OP_' + opr.parent_enc
                    par_enc_type = self.type_by_value[op]
                    add_flags = ''
                    for t in par_enc_type:
                        newt = copy.deepcopy(t)
                        newt.value += offset
                        newt.parent_enc = opr.parent_enc
                        newt.flags.append('OPF_PEN_%s' % opr.parent_enc)
                        op_type[newt.value] = newt
                    # check to make sure entry at offset was replaced
                    if op_type[offset].name == match:
                        op_type[offset] = None

    # handle an encoding with a parent_enc entry
    def handle_parent_enc(self, inst, enc_field, op_field):
        encode = self.enc_by_name[enc_field.enc]
        op_type = self.type_by_value[op_field.type]
        self.handle_parent_enc_search_replace(encode.operands, op_type)

    # build a table to lookup in which word an inst.field is found
    def build_inst_fields(self, inst):
        assert type(inst) is InstBlock
        if re.search('[Ss]econd', inst.desc):
            which_word = 'extData'
        else:
            which_word = 'instData'

        key = re.match('([^_]+)', inst.name).group(1)
        if key == 'VOP':
            key = inst.name

        if key in self.inst_fields.keys():
            table = self.inst_fields[key]
        else:
            self.inst_fields[key] = table = {}
        for field in inst.fields:
            table[field.name] = which_word

    def handle_inst(self, inst):
        assert type(inst) is InstBlock
        self.inst_by_name[inst.name] = inst
        self.build_inst_fields(inst)
        op_field = None
        enc_field = None
        # field_list will be in a form that cg_struct can use
        field_list = []
        for field in inst.fields:
            field_list.append([ field.name, field.value, field.v_max ])
            if field.name == 'OP':
                op_field = field
            elif field.name == 'ENCODING':
                if field.enc != '':
                    enc_field = field
        self.inst_formats[inst.name] = field_list
        if enc_field != None:
            if inst.name not in self.inst_with_encodings:
                self.inst_with_encodings.append(inst.name)
            if op_field != None:
                self.handle_enc_op(inst, enc_field, op_field)
            else:
                self.handle_enc_no_op(inst, enc_field)
        else:
            assert op_field == None, 'inst %s enc:None op: %s' % (
                inst.name, op_field.type)

    def handle_encoding(self, statement):
        self.enc_by_name[statement.name] = statement
        (base, copy) = self.bits_info(statement.bits, self.high_bits)
        n = 'decode_OP_' + statement.name + '__invalid'
        self.decode_info[n] = [ 'OP_' + statement.name, self.invalid_type ]
        primary_decode_table = self.decode_tables['tableDecodePrimary']
        for i in range(base, base + copy):
            if primary_decode_table[i] == 'subDecode_invalid':
                primary_decode_table[i] = n

    # check for vector v scalar instruction encodings
    def vector_or_scalar(self):
        enc_vec = {}
        for k in self.enc_by_name.keys():
            e = self.enc_by_name[k]
            if 'Vector ALU' in e.desc[0]:
                enc_vec[e.name] = True
            else:
                enc_vec[e.name] = False

        inst_vec = {}
        for k in self.inst_by_name.keys():
            i = self.inst_by_name[k]
            is_vec = False
            for f in i.fields:
                if f.type == 'VGPR':
                    is_vec = True
            inst_vec[i.name] = is_vec

        # check the second word if a second word exists
        for k in self.inst_by_name.keys():
            i = self.inst_by_name[k]
            if '_1' in i.name:
                continue
            if inst_vec[i.name]:
                continue
            if '_' in i.name:
                other_word = re.sub('_.*', '_1', i.name)
            else:
                other_word = '%s_1' % i.name
            if other_word in inst_vec.keys():
                if inst_vec[other_word]:
                    inst_vec[i.name] = True

        for info in self.refined_op_info:
            if enc_vec[info.enc] or inst_vec[info.inst]:
                if 'OPF_VECTOR' not in info.flags:
                    info.flags.append('OPF_VECTOR')
            else:
                if 'OPF_SCALAR' not in info.flags:
                    info.flags.append('OPF_SCALAR')

    def post_process_statements(self):
        self.vector_or_scalar()

    def process_statements(self, statements):
        for statement in statements:
            if statement.keyword == 'const':
                self.handle_const(statement)
            elif statement.keyword == 'type':
                self.handle_type(statement)
            elif statement.keyword == 'encoding':
                self.handle_encoding(statement)
            elif statement.keyword == 'inst':
                self.handle_inst(statement)
        self.post_process_statements()

    def is_special_lit(self, inst):
        for field in inst.fields:
            if field.type == '':
                continue
            tbv = self.type_by_value[field.type]
            assert type(tbv) is IndexedList
            for t in range(0, len(tbv)):
                if tbv[t] and tbv[t].name == 'SRC_LITERAL':
                    return True
        return False

    def is_var_size_enc(self, pri, sec):
        if self.is_special_lit(self.inst_by_name[pri]):
            return True
        if sec in self.inst_by_name.keys():
            if self.is_special_lit(self.inst_by_name[sec]):
                return True
        return False

    # gpu_decoder.hh
    def generate_decoder_hh(self, output_dir):
        file = os.path.join(output_dir, 'gpu_decoder.hh')
        cg = CodeGen(file)

        cg.cg_code('#ifndef __GPU_INTERNAL_ARCH_VI_DECODER_HH__')
        cg.cg_code('#define __GPU_INTERNAL_ARCH_VI_DECODER_HH__')
        cg.cg_newline()

        cg.cg_include('<string>')
        cg.cg_include('<vector>')
        cg.cg_newline()

        cg.cg_include('gpu-internal/arch/vi/gpu_types.hh')
        cg.cg_newline()
        cg.cg_code('class GPUStaticInst;')
        cg.cg_newline()

        cg.cg_namespace('ViISA')

        cg.cg_code('class Decoder;')
        cg.cg_code('union InstFormat;')
        cg.cg_newline()
        cg.cg_code('using IsaDecodeMethod = GPUStaticInst*'
                   '(Decoder::*)(MachInst);')
        cg.cg_newline()
        cg.cg_class('Decoder', [])
        cg.cg_scope('public:')
        cg.cg_code('Decoder();')
        cg.cg_code('~Decoder();')
        cg.cg_newline()

        cg.cg_code('GPUStaticInst* decode(MachInst);')
        cg.cg_newline()

        cg.cg_method('GPUStaticInst*', '', 'decode', ['RawMachInst inst'], [])
        cg.cg_code('return inst < decodedInsts.size() ? decodedInsts'\
                   '.at(inst) : nullptr;')
        cg.cg_end(None)
        cg.cg_newline()
        cg.cg_newline()

        cg.cg_method('RawMachInst', '',
                     'saveInst', [ 'GPUStaticInst *decodedInst' ], [])
        cg.cg_code('decodedInsts.push_back(decodedInst);')
        cg.cg_code('return decodedInsts.size() - 1;')
        cg.cg_end(None)
        cg.cg_newline()
        cg.cg_newline()

        cg.cg_scope('private:')
        cg.cg_code('static std::vector<GPUStaticInst*> decodedInsts;')
        cg.cg_newline()

        methods = []
        for key in sorted(self.decode_tables.keys()):
            table = self.decode_tables[key]
            for m in table:
                if m not in methods:
                    methods.append(m)
            size = len(table)
            cg.cg_code('static IsaDecodeMethod %s[%d];' % (key, size))

        cg.cg_newline()

        for m in sorted(methods):
            cg.cg_code('GPUStaticInst* %s(MachInst);' % m)
        cg.cg_code('GPUStaticInst* decode_invalid(MachInst);')
        cg.cg_end('class Decoder') # cg_class

        for key in sorted(self.inst_formats.keys()):
            cg.cg_newline()
            cg.cg_struct('InFmt_' + key, self.inst_formats[key])

        cg.cg_newline()
        cg.cg_union('InstFormat')
        max = 0
        for key in sorted(self.inst_formats.keys()):
            sz = len(key)
            if max < sz:
                max = sz
        for key in sorted(self.inst_formats.keys()):
            pad = ' ' * (max - len(key))
            cg.cg_code('InFmt_%s %siFmt_%s;' % (key, pad, key))
        dtyp = 'unsigned int'
        dfld = 'imm_u32'
        pad = ' ' * (max - len(dtyp))
        cg.cg_code('%s       %s%s;' % (dtyp, pad, dfld))
        dtyp = 'float'
        dfld = 'imm_f32'
        pad = ' ' * (max - len(dtyp))
        cg.cg_code('%s       %s%s;' % (dtyp, pad, dfld))
        cg.cg_end('union InstFormat') # cg_union

        cg.cg_end('namespace ViISA') # cg_namespace
        cg.cg_newline()

        cg.cg_code('#endif // __GPU_INTERNAL_ARCH_VI_DECODER_HH__')
        cg.generate()

    # decoder.cc base
    def generate_decoder_cc(self, output_dir):
        file = os.path.join(output_dir, 'decoder.cc')
        cg = CodeGen(file)

        cg.cg_include('<vector>')
        cg.cg_newline()

        cg.cg_include('gpu-internal/arch/vi/gpu_decoder.hh')
        cg.cg_include('gpu-internal/arch/vi/gpu_static_inst.hh')
        cg.cg_include('gpu-internal/arch/vi/instructions.hh')
        cg.cg_newline()

        cg.cg_namespace('ViISA')

        cg.cg_method(None, 'Decoder', 'Decoder', [], [])
        cg.cg_end('Decoder') # cg_method
        cg.cg_newline()

        cg.cg_method(None, 'Decoder', '~Decoder', [], [])
        cg.cg_end('~Decoder') # cg_method
        cg.cg_newline()

        for key in sorted(self.decode_tables.keys()):
            table = self.decode_tables[key]
            cg.cg_table('IsaDecodeMethod', 'Decoder', key, table)

        cg.cg_method('GPUStaticInst*', 'Decoder',
                     'decode', [ 'MachInst iFmt' ], [])
        cg.cg_code('InFmt_SOP1 *enc = &iFmt->iFmt_SOP1;')
        cg.cg_code('IsaDecodeMethod method = ' +
                   'tableDecodePrimary[enc->ENCODING];')
        cg.cg_code('return (this->*method)(iFmt);')
        cg.cg_end('decode') # cg_method

        decoders = []
        sub_decoders = []
        for key in sorted(self.decode_tables.keys()):
            table = self.decode_tables[key]
            for m in table:
                if m[0:7] == 'decode_':
                    if m not in decoders:
                        decoders.append(m)
                elif m[0:10] == 'subDecode_':
                    if m not in sub_decoders:
                        sub_decoders.append(m)

        for m in sub_decoders:
            cg.cg_newline()
            cg.cg_method('GPUStaticInst*', 'Decoder', \
                         m, ['MachInst iFmt'], [])
            if m == 'subDecode_invalid':
                cg.cg_code('return decode_invalid(iFmt);')
            else:
                o = m[10:]
                e = self.inst_by_optype[o]
                t = 'tableSubDecode_' + o
                cg.cg_code('InFmt_' + e + ' *enc = &iFmt->iFmt_' + e + ';')
                cg.cg_code('IsaDecodeMethod method = ' + t + '[enc->OP];')
                cg.cg_code('return (this->*method)(iFmt);')
            cg.cg_end(m) # cg_method

        for m in decoders:
            cg.cg_newline()
            cg.cg_method('GPUStaticInst*', 'Decoder',
                         m, ['MachInst iFmt'], [])
            [ op_type, t ] = self.decode_info[m]
            op_enc = re.sub('OP[U]?_', '', op_type)
            op_fmt = 'iFmt_%s' % op_enc
            if op_fmt == 'iFmt_VOP3':
                vccd = 'OPF_VCCD' in t.flags
                vopc = 'OPF_PEN_VOPC' in t.flags
                if vccd or vopc:
                    op_fmt = 'iFmt_VOP3_SDST_ENC'
            op_inst = 'Inst_%s__%s' % (op_enc, t.name)
            cg.cg_code('return new %s(&iFmt->%s);' % (op_inst, op_fmt))
            cg.cg_end(m) # cg_method

        cg.cg_newline()
        cg.cg_method('GPUStaticInst*', 'Decoder',
                     'decode_invalid', [ 'MachInst iFmt' ], [])
        cg.cg_code('return new Inst_invalid(iFmt);')
        cg.cg_end('decode_invalid') # cg_method

        cg.cg_code('std::vector<GPUStaticInst*> Decoder::decodedInsts;')
        cg.cg_newline()

        cg.cg_end('namespace ViISA') # cg_namespace

        cg.generate()

    # instructions.hh
    def generate_instructions_hh(self, output_dir):
        file = os.path.join(output_dir, 'instructions.hh')
        cg = CodeGen(file)

        cg.cg_code('#ifndef __GPU_INTERNAL_ARCH_VI_INSTRUCTIONS_HH__')
        cg.cg_code('#define __GPU_INTERNAL_ARCH_VI_INSTRUCTIONS_HH__')
        cg.cg_newline()

        cg.cg_include('gpu-internal/arch/vi/gpu_decoder.hh')
        cg.cg_include('gpu-internal/arch/vi/gpu_static_inst.hh')
        cg.cg_include('gpu-internal/arch/vi/op_encodings.hh')
        cg.cg_newline()

        cg.cg_namespace('ViISA')

        for info in self.refined_op_info:
            op_op = re.sub('.*__', '', info.name)
            op_enc = re.sub('__.*', '', info.name)
            op_inst = 'Inst_%s__%s' % (op_enc, op_op)
            op_base = 'Inst_%s' % op_enc
            op_fmt = 'InFmt_%s' % op_enc
            if op_enc == 'VOP3':
                vccd = 'OPF_VCCD' in info.flags
                vopc = 'OPF_PEN_VOPC' in info.flags
                if vccd or vopc:
                    op_base = 'Inst_VOP3_SDST_ENC'
                    op_fmt = 'InFmt_VOP3_SDST_ENC'

            cg.cg_class(op_inst, [ op_base ])
            cg.cg_scope('public:')
            cg.cg_code('%s(%s*);' % (op_inst, op_fmt))

            cg.cg_code('~%s();' % op_inst)
            cg.cg_newline()

            n_dst = 0
            n_src = 0

            if info.sub_enc != 'SEN_NODST' and info.sub_enc != 'SEN_G_FORK':
                n_dst = info.num_dst

            if info.sub_enc != 'SEN_NOSRC':
                n_src = info.num_src

            if 'OPF_RDVCC' in info.flags:
                n_src += 1

            cg.cg_method('int', None, 'getNumOperands', None, None,
                         'override')
            cg.cg_code('return numDstRegOperands() + numSrcRegOperands();')
            cg.cg_end('getNumOperands')

            cg.cg_newline()
            cg.cg_code('int numDstRegOperands() override { return %i; }'
                       % n_dst)
            cg.cg_code('int numSrcRegOperands() override { return %i; }'
                       % n_src)

            # int getOperandSize(int opIdx);
            op_idx = 0
            cg.cg_newline()
            cg.cg_method('int', None, 'getOperandSize', ['int opIdx'], None,
                         'override')
            cg.cg_code('switch (opIdx) {')
            for src_op in info.src:
                if ((src_op.name == 'carryin' or src_op.name == 'vcc') and
                    'OPF_VCCS' not in info.flags):
                    continue
                if src_op.name == 'vgpr_d0' and 'OPF_DS1D' not in info.flags:
                    continue
                if src_op.name == 'vgpr_d1' and 'OPF_DS2D' not in info.flags:
                    continue
                if not src_op.fmt:
                    op_size = (src_op.size * 32) / 8
                elif src_op.fmt == 'RSRC_TYPED' or src_op.fmt == 'SAMP':
                    op_size = 4
                else:
                    op_size = self.fmt_to_details(src_op.fmt)[1] / 8
                cg.cg_code('  case %i: //%s' % (op_idx, src_op.name))
                cg.cg_code('    return %i;' % op_size)
                op_idx += 1
            if 'OPF_RDVCC' in info.flags:
                cg.cg_code('  case %i:' % op_idx)
                cg.cg_code('    return 8;')
                op_idx += 1
            for dst_op in info.dst:
                if ((dst_op.name == 'carryout' or dst_op.name == 'vcc') and
                    'OPF_VCCD' not in info.flags and op_enc != 'VOPC'):
                    continue
                if dst_op.name == 'vgpr_rtn' and 'OPF_DSRTN' not in info.flags:
                    continue
                if (op_inst == 'Inst_SMEM__S_LOAD_DWORD'
                    or op_inst == 'Inst_FLAT__FLAT_LOAD_DWORD'):
                    op_size = 4
                elif not dst_op.fmt:
                    op_size = (dst_op.size * 32) / 8
                else:
                    op_size = self.fmt_to_details(dst_op.fmt)[1] / 8
                cg.cg_code('  case %i: //%s' % (op_idx, dst_op.name))
                cg.cg_code('    return %i;' % op_size)
                op_idx += 1
            cg.cg_code('  default:')
            cg.cg_code(r'    fatal("op idx %i out of bounds\n", opIdx);')
            cg.cg_code('    return -1;')
            cg.cg_code('}')
            cg.cg_end('getOperandSize')

            # bool isSrcOperand(int opIdx);
            op_idx = 0
            cg.cg_newline()
            cg.cg_method('bool', None, 'isSrcOperand', ['int opIdx'], None,
                         'override')
            cg.cg_code('switch (opIdx) {')
            for src_op in info.src:
                if ((src_op.name == 'carryin' or src_op.name == 'vcc') and
                    'OPF_VCCS' not in info.flags):
                    continue
                if src_op.name == 'vgpr_d0' and 'OPF_DS1D' not in info.flags:
                    continue
                if src_op.name == 'vgpr_d1' and 'OPF_DS2D' not in info.flags:
                    continue
                cg.cg_code('  case %i: //%s' % (op_idx, src_op.name))
                cg.cg_code('    return true;')
                op_idx += 1
            if 'OPF_RDVCC' in info.flags:
                cg.cg_code('  case %i:' % op_idx)
                cg.cg_code('    return true;')
                op_idx += 1
            for dst_op in info.dst:
                if ((dst_op.name == 'carryout' or dst_op.name == 'vcc') and
                    'OPF_VCCD' not in info.flags):
                    continue
                if dst_op.name == 'vgpr_rtn' and 'OPF_DSRTN' not in info.flags:
                    continue
                cg.cg_code('  case %i: //%s' % (op_idx, dst_op.name))
                cg.cg_code('    return false;')
                op_idx += 1
            cg.cg_code('  default:')
            cg.cg_code(r'    fatal("op idx %i out of bounds\n", opIdx);')
            cg.cg_code('    return false;')
            cg.cg_code('}')
            cg.cg_end('isSrcOperand')

            # bool isDstOperand(int opIdx);
            op_idx = 0
            cg.cg_newline()
            cg.cg_method('bool', None, 'isDstOperand', ['int opIdx'], None,
                         'override')
            cg.cg_code('switch (opIdx) {')
            for src_op in info.src:
                if ((src_op.name == 'carryin' or src_op.name == 'vcc') and
                    'OPF_VCCS' not in info.flags):
                    continue
                if src_op.name == 'vgpr_d0' and 'OPF_DS1D' not in info.flags:
                    continue
                if src_op.name == 'vgpr_d1' and 'OPF_DS2D' not in info.flags:
                    continue
                cg.cg_code('  case %i: //%s' % (op_idx, src_op.name))
                cg.cg_code('    return false;')
                op_idx += 1
            if 'OPF_RDVCC' in info.flags:
                cg.cg_code('  case %i:' % op_idx)
                cg.cg_code('    return false;')
                op_idx += 1
            for dst_op in info.dst:
                if ((dst_op.name == 'carryout' or dst_op.name == 'vcc') and
                    'OPF_VCCD' not in info.flags):
                    continue
                if dst_op.name == 'vgpr_rtn' and 'OPF_DSRTN' not in info.flags:
                    continue
                cg.cg_code('  case %i: //%s' % (op_idx, dst_op.name))
                cg.cg_code('    return true;')
                op_idx += 1
            cg.cg_code('  default:')
            cg.cg_code(r'    fatal("op idx %i out of bounds\n", opIdx);')
            cg.cg_code('    return false;')
            cg.cg_code('}')
            cg.cg_end('isDstOperand')

            # void execute(GPUDynInstPtr gpuDynInst);
            cg.cg_newline()
            cg.cg_code('void execute(GPUDynInstPtr) override;')
            if ('OPF_MEM_STORE' in info.flags or 'LOAD' in op_op or
                op_op == 'DS_WRITE_B32' or op_op == 'DS_WRITE_B64' or
                op_op == 'DS_READ_B32' or op_op == 'DS_READ_B64'):
                cg.cg_code('void initiateAcc(GPUDynInstPtr) override;')
                cg.cg_code('void completeAcc(GPUDynInstPtr) override;')

            #if op_enc == 'SMEM':
                #info.decl.append('SregU64 offset(extData.OFFSET);')
                #info.decl.append('SregU64 offset(665);')

            if op_inst in HandCodedDecl.keys():
                cg.cg_newline()
                cg.cg_scope('private:')
                cg.cg_block(HandCodedDecl[op_inst])
            elif info.decl:
                cg.cg_newline()
                cg.cg_scope('private:')
                cg.cg_block(info.decl)

            cg.cg_end(op_inst) # cg_class
            cg.cg_newline()

        for op_enc in self.inst_with_encodings:
            if op_enc == 'EXP':
                op_op = 'default'
            else:
                op_op = 'invalid'
            op_inst = 'Inst_%s__%s' % (op_enc, op_op)
            op_fmt = 'InFmt_%s' % op_enc
            cg.cg_newline()
            cg.cg_class(op_inst, ['Inst_%s' % op_enc ])
            cg.cg_scope('public:')
            cg.cg_code('%s(%s*);' % (op_inst, op_fmt))
            cg.cg_code('~%s();' % op_inst)
            cg.cg_newline()

            cg.cg_code('bool isValid() const override;')
            cg.cg_code('int getNumOperands() override { return -1; }')
            cg.cg_code('int numDstRegOperands() override { return -1; }')
            cg.cg_code('int numSrcRegOperands() override { return -1; }')
            cg.cg_code('void execute(GPUDynInstPtr) override;')
            if ('OPF_MEM_STORE' in info.flags or 'LOAD' in op_op or
                op_op == 'DS_WRITE_B32' or op_op == 'DS_WRITE_B64' or
                op_op == 'DS_READ_B32' or op_op == 'DS_READ_B64'):
                cg.cg_code('void initiateAcc(GPUDynInstPtr) override;')
            cg.cg_end(op_inst) # cg_class

        cg.cg_newline()
        op_inst = 'Inst_invalid'
        op_fmt = 'InstFormat'
        cg.cg_class(op_inst, ['ViGPUStaticInst'])
        cg.cg_scope('public:')
        cg.cg_code('%s(%s*);' % (op_inst, op_fmt))
        cg.cg_code('~%s();' % op_inst)
        cg.cg_newline()
        cg.cg_code('int getNumOperands() override { return -1; }')
        cg.cg_code('int numDstRegOperands() override { return -1; }')
        cg.cg_code('int numSrcRegOperands() override { return -1; }')
        cg.cg_code('void execute(GPUDynInstPtr gpuDynInst) { }')
        cg.cg_code('bool isValid() const override;')
        cg.cg_code('uint32_t instSize();')
        cg.cg_end(op_inst) # cg_class

        cg.cg_end('namespace ViISA') # cg_namespace

        cg.cg_code('#endif // __GPU_INTERNAL_ARCH_VI_INSTRUCTIONS_HH__')
        cg.generate()

    def find_opr_info(self, oi_list, opr, idx):
        assert type(oi_list) is list
        assert type(opr) is str
        assert type(idx) is int
        for oi in oi_list:
            assert type(oi) is OpInfo
            if oi.opr == opr and oi.index == idx:
                return oi
        import pdb; pdb.set_trace()
        return None

    def fmt_to_details(self, fmt):
        if fmt in FmtToDetails.keys():
            return FmtToDetails[fmt]
        if fmt == '':
            return ('unknown', -1)
        import pdb; pdb.set_trace()
        return ('unknown', -1)

    def type_to_details(self, typ):
        if typ in TypeToDetails.keys():
            return TypeToDetails[typ]
        # caller can handle 'unknown'
        return ('unknown', -1)

    def size_to_details(self, size):
        if size > 0:
            bits = 32 * size
            return ('SregU%d' % bits, bits)
        import pdb; pdb.set_trace()
        return ('unknown', -1)

    def reg_info_to_field(self, reg, info, op_info):
        assert type(reg) is str
        assert type(op_info) is OpInfo
        key = '%s:%s:%s:%s' % (info.enc, reg, op_info.iseq, op_info.name)
        if key in EncRegInfoToField.keys():
            return EncRegInfoToField[key]
        key = '%s:%s:%s' % (reg, op_info.iseq, op_info.name)
        if key in RegInfoToField.keys():
            return RegInfoToField[key]
        import pdb; pdb.set_trace()
        return ''

    def typ_to_dtyp_dsiz(self, typ, op_info):
        assert type(typ) is list
        assert type(op_info) is OpInfo
        # instruction desc info gets priority
        details = self.type_to_details(typ[0])
        dtyp = details[0]
        if dtyp == 'unknown':
            details = self.fmt_to_details(op_info.fmt)
            dtyp = details[0]
            if dtyp == 'unknown':
                details = self.size_to_details(op_info.size)
                dtyp = details[0]
        dsiz = details[1]
        if dsiz < 0:
            dsiz = op_info.size
        return (dtyp, dsiz)

    def spec_reg_to_frag(self, reg, info, key):
        frag = CodeFrag(key)
        details = SpecRegToDetails[reg]
        frag.setup(details)
        if frag.ctx == '' and frag.fld == '':
            op_info = self.find_opr_info(info.dst, 'dst', 0)
            no_typ = ['unknown', -1]
            (dtyp, dsiz) = self.typ_to_dtyp_dsiz(no_typ, op_info)
            frag.typ = dtyp
        return frag

    def reg_access_fragment(self, reg, info, op_info, vop, key):
        assert type(reg) is DataRegClause
        assert type(info) is RefinedOpInfo
        assert type(op_info) is OpInfo
        assert type(vop) is str
        assert type(key) is str
        (dtyp, dsiz) = self.typ_to_dtyp_dsiz(reg.typ, op_info)
        field = self.reg_info_to_field(reg.reg, info, op_info)
        if field in self.inst_fields[info.enc].keys():
            data = self.inst_fields[info.enc][field]
            word = '%s.%s' % (data, field)
        else:
            word = field
        if reg.reg in DataRegisters:
            if reg.reg[-2:] == '_1':
                word += ' + 1'
            elif reg.reg[-2:] == '_2':
                word += ' + 2'
        if field and field[0] == '-':
            vop = ''
        var = op_info.name
        codeFrag = CodeFrag(key)
        codeFrag.ctx = op_info.iseq # context accessor
        codeFrag.fld = word         # field reference
        codeFrag.var = var          # variable name
        codeFrag.typ = dtyp         # variable type
        codeFrag.exp = ''           # general expression
        codeFrag.vec = vop          # vector index
        return codeFrag

    def dst_fragment(self, reg, info):
        assert type(reg) is DataRegClause
        assert type(info) is RefinedOpInfo
        vop = ''
        if reg.reg == 'D':
            op_info = self.find_opr_info(info.dst, 'dst', 0)
        elif reg.reg in DataRegisters:
            op_info = self.find_opr_info(info.dst, 'dst', 0)
        elif reg.reg in SpecRegToDetails.keys():
            return self.spec_reg_to_frag(reg.reg, info, 'dst')
        else:
            import pdb; pdb.set_trace()
            return None
        assert type(op_info) is OpInfo
        if 'OPF_VECTOR' in info.flags:
            if op_info.iseq != 'SREG':  # VOP3_VOPC uses a scalar dst
                vop = '[t]'
        return self.reg_access_fragment(reg, info, op_info, vop, 'dst')

    def get_addr_op_info(self, info):
        assert type(info) is RefinedOpInfo
        if info.enc in ['SMEM', 'MUBUF']:
            if 'OPF_MEM_STORE' in info.flags:
                return self.find_opr_info(info.src, 'src', 1)
        return self.find_opr_info(info.src, 'src', 0)

    def get_data_op_info(self, info):
        assert type(info) is RefinedOpInfo
        if info.enc in ['SMEM', 'MUBUF']:
            if 'OPF_MEM_STORE' in info.flags:
                return self.find_opr_info(info.src, 'src', 0)
        if 'OPF_MEM_ATOMIC' in info.flags:
            return self.find_opr_info(info.dst, 'dst', 0)
        return self.find_opr_info(info.src, 'src', 1)

    def src_fragment(self, reg, info):
        assert type(reg) is DataRegClause
        assert type(info) is RefinedOpInfo
        vop = ''
        if reg.reg == 'S0' or reg.reg == 'S':
            op_info = self.find_opr_info(info.src, 'src', 0)
        elif reg.reg == 'S1':
            op_info = self.find_opr_info(info.src, 'src', 1)
        elif reg.reg == 'S2':
            op_info = self.find_opr_info(info.src, 'src', 2)
        elif reg.reg == 'D':
            op_info = self.find_opr_info(info.dst, 'dst', 0)
        elif reg.reg == 'ADDR':
            op_info = self.get_addr_op_info(info)
        elif reg.reg == 'ADDR_BASE':
            op_info = self.find_opr_info(info.src, 'src', 0)
        elif reg.reg == 'DATA2':
            op_info = self.find_opr_info(info.src, 'src', 2)
        elif reg.reg in DataRegisters:
            op_info = self.get_data_op_info(info)
        elif reg.reg in SpecRegToDetails.keys():
            return self.spec_reg_to_frag(reg.reg, info, 'src')
        else:
            import pdb; pdb.set_trace()
            return None
        assert type(op_info) is OpInfo
        if 'OPF_VECTOR' in info.flags:
            if op_info.iseq != 'SREG':  # VOP3_VOPC uses a scalar dst
                vop = '[t]'
        return self.reg_access_fragment(reg, info, op_info, vop, 'src')

    def unary_expr(self, clause, info):
        assert type(clause) is UnaryClause
        assert type(info) is RefinedOpInfo
        (src_set, operand) = self.expression(clause.oprnd, info)
        exp = '%s%s' % (clause.op, operand)
        return (src_set, exp)

    def binary_expr(self, clause, info):
        assert type(clause) is BinaryClause
        assert type(info) is RefinedOpInfo
        (src_set, lex) = self.expression(clause.left, info)
        (rex_ss, rex) = self.expression(clause.right, info)
        src_set.extend(rex_ss)
        exp = '%s %s %s' % (lex, clause.op, rex)
        return (src_set, exp)

    def function_expr(self, clause, info):
        assert type(clause) is FunctionClause
        assert type(info) is RefinedOpInfo
        if clause.args:
            (src_set, arg_exp) = self.expression(clause.args, info)
        else:
            src_set = []
            arg_exp = ''
        exp = '%s(%s)' % (clause.func, arg_exp)
        return (src_set, exp)

    def cond_expr(self, clause, info):
        assert isinstance(clause, Clause)
        assert type(info) is RefinedOpInfo
        (src_set, cond_exp) = self.expression(clause.cond, info)
        (true_ss, true_exp) = self.expression(clause.true, info)
        (false_ss, false_exp) = self.expression(clause.false, info)
        src_set.extend(true_ss)
        src_set.extend(false_ss)
        exp = '%s ? %s : %s' % (cond_exp, true_exp, false_exp)
        return (src_set, exp)

    def const_expr(self, clause, info):
        assert type(clause) is ConstantClause
        assert type(info) is RefinedOpInfo
        if type(clause.value) is float:
            exp = '%f' % clause.value
        elif clause.value < 1000:
            exp = '%d' % clause.value
        else:
            exp = '0x%x' % clause.value
        return ([], exp)

    def cast_expr(self, clause, info):
        assert type(clause) is CastClause
        assert type(info) is RefinedOpInfo
        (src_set, var_exp) = self.expression(clause.var, info)
        exp = '(%s)%s' % (clause.typ, var_exp)
        return (src_set, exp)

    def func_expr(self, clause, info):
        assert type(clause) is FunctionClause
        assert type(info) is RefinedOpInfo
        if clause.args == []:
            src_set = []
            exp = '%s()' % clause.func
        else:
            (src_set, arg_exp) = self.expression(clause.args[0], info)
            exp = '%s(%s' % (clause.func, arg_exp)
            for a in clause.args[1:]:
                (arg_ss, arg_exp) = self.expression(a, info)
                exp += ', %s' % (arg_exp)
                src_set.extend(arg_ss)
            exp += ')'
        return (src_set, exp)

    def comma_expr(self, clause, info):
        assert type(clause) is CommaClause
        assert type(info) is RefinedOpInfo
        (src_set, lex) = self.expression(clause.left, info)
        (right_ss, rex) = self.expression(clause.right, info)
        src_set.extend(right_ss)
        exp = '%s, %s' % (lex, rex)
        return (src_set, exp)

    def src_gpr_expr(self, clause, info):
        assert type(clause) is GprClause
        assert type(info) is RefinedOpInfo
        # use the S0 op_info as a reference
        op_info = self.find_opr_info(info.src, 'src', 0)
        (dtyp, dsiz) = self.typ_to_dtyp_dsiz(clause.typ, op_info)
        if type(clause.idx) is BinaryClause:
            (src_set, rex) = self.expression(clause.idx.right, info)
            (left_ss, lex) = self.src_operand(clause.idx.left, info)
            left_ss[0].fld += ' %s %s' % (clause.idx.op, rex)
            src_set.extend(left_ss)
            return (src_set, lex)
        elif type(clause.idx) is DataRegClause:
            return self.src_operand(clause.idx, info)
        else:
            import pdb; pdb.set_trace()
        return ([], '')

    def dst_gpr_expr(self, clause, info):
        assert type(clause) is GprClause
        assert type(info) is RefinedOpInfo
        # use the D0 op_info as a reference
        op_info = self.find_opr_info(info.dst, 'dst', 0)
        (dtyp, dsiz) = self.typ_to_dtyp_dsiz(clause.typ, op_info)
        if type(clause.idx) is BinaryClause:
            (src_set, rex) = self.expression(clause.idx.right, info)
            (left_ss, lex) = self.dst_operand(clause.idx.left, info)
            left_ss[0].fld += ' %s %s' % (clause.idx.op, rex)
            src_set.extend(left_ss)
            return (src_set, lex)
        elif type(clause.idx) is DataRegClause:
            return self.dst_operand(clause.idx, info)
        else:
            import pdb; pdb.set_trace()
        return ([], '')
        return (src_set, exp)

    def range_expr(self, range, info, var):
        assert type(info) is RefinedOpInfo
        if range:
            if type(range) is list:
                (src_set, lex) = self.expression(range[0], info)
                (rex_ss, rex) = self.expression(range[1], info)
                src_set.extend(rex_ss)
                exp = '%s(%s, %s)' % (var, lex, rex)
            else:
                (src_set, lex) = self.expression(range, info)
                exp = '%s(%s)' % (var, lex)
        else:
            src_set = []
            exp = var
        return (src_set, exp)

    def src_mem_expr(self, clause, info):
        assert type(clause) is MemClause
        assert type(info) is RefinedOpInfo
        if self.prefix:
            dtyp = self.prefix
        else:
            if info.enc == 'SMEM':
                if info.name[-5:] == 'DWORD':
                    # fixup hack - dst:0 size is wrong
                    op_info = OpInfo()
                    op_info.size = 1
                else:
                    # use the D0 op_info as a reference
                    op_info = self.find_opr_info(info.dst, 'dst', 0)
            elif info.enc == 'FLAT':
                if info.name[-5:] == 'DWORD':
                    # fixup hack - dst:0 size is wrong
                    op_info = OpInfo()
                    op_info.size = 1
                else:
                    # use the D0 op_info as a reference
                    op_info = self.find_opr_info(info.dst, 'dst', 0)
            elif info.enc == 'MUBUF':
                if info.name[-5:] == 'DWORD':
                    # fixup hack - dst:0 size is wrong
                    op_info = OpInfo()
                    op_info.size = 1
                else:
                    # use the D0 op_info as a reference
                    op_info = self.find_opr_info(info.dst, 'dst', 0)
            else:
                # use the S1 op_info as a reference
                op_info = self.find_opr_info(info.src, 'src', 1)
            no_typ = ['unknown', -1]
            (dtyp, dsiz) = self.typ_to_dtyp_dsiz(no_typ, op_info)
        (src_set, addr_exp) = self.expression(clause.addr, info)
        for addr_frag in src_set:
            if 'ADDR' in addr_frag.fld:
                break
        mem_frag = CodeFrag('src')
        mem_frag.ctx = clause.mem
        mem_frag.fld = addr_frag.var
        mem_frag.var = ''
        mem_frag.typ = dtyp
        mem_frag.exp = addr_exp
        mem_frag.vec = addr_frag.vec
        mem_frag.scn = 'mem'
        src_set.append(mem_frag)
        exp = 'vmem_$%s$%s' % (mem_frag.exp, mem_frag.vec)
        (rng_ss, exp) = self.range_expr(clause.rng, info, exp)
        src_set.extend(rng_ss)
        return (src_set, exp)

    def dst_mem_expr(self, clause, info):
        assert type(clause) is MemClause
        assert type(info) is RefinedOpInfo
        if self.prefix:
            dtyp = self.prefix
        else:
            if info.enc == 'SMEM':
                if info.name[-5:] == 'DWORD':
                    # fixup hack - src:0 size is wrong
                    op_info = OpInfo()
                    op_info.size = 1
                else:
                    # use the S0 op_info as a reference
                    op_info = self.find_opr_info(info.src, 'src', 0)
            elif info.enc == 'FLAT':
                if info.name[-5:] == 'DWORD':
                    # fixup hack - src:0 size is wrong
                    op_info = OpInfo()
                    op_info.size = 1
                else:
                    # use the S1 op_info as a reference
                    op_info = self.find_opr_info(info.src, 'src', 1)
            elif info.enc == 'MUBUF':
                if info.name[-5:] == 'DWORD':
                    # fixup hack - src:0 size is wrong
                    op_info = OpInfo()
                    op_info.size = 1
                else:
                    # use the S0 op_info as a reference
                    op_info = self.find_opr_info(info.src, 'src', 0)
            elif info.dst:
                # use the D0 op_info as a reference
                op_info = self.find_opr_info(info.dst, 'dst', 0)
            else:
                import pdb; pdb.set_trace()
                # use the S0 op_info as a reference
                op_info = self.find_opr_info(info.src, 'src', 0)
            no_typ = ['unknown', -1]
            (dtyp, dsiz) = self.typ_to_dtyp_dsiz(no_typ, op_info)
        (src_set, addr_exp) = self.expression(clause.addr, info)
        for addr_frag in src_set:
            if 'ADDR' in addr_frag.fld:
                break
        mem_frag = CodeFrag('dst')
        mem_frag.ctx = clause.mem
        mem_frag.fld = addr_frag.var
        mem_frag.var = ''
        mem_frag.typ = dtyp
        mem_frag.exp = addr_exp
        mem_frag.vec = addr_frag.vec
        mem_frag.scn = 'mem'
        src_set.append(mem_frag)
        exp = 'vmem_$%s$%s' % (mem_frag.exp, mem_frag.vec)
        return (src_set, exp)

    def group_expr(self, clause, info):
        assert type(clause) is GroupClause
        assert type(info) is RefinedOpInfo
        (src_set, grp_exp) = self.expression(clause.group[0], info)
        exp = 'Group('
        comma = ''
        src_set = []
        for grp in clause.group:
            (grp_ss, grp_exp) = self.expression(grp, info)
            src_set.extend(grp_ss)
            exp += comma + grp_exp
            comma = ', '
        exp += ')'
        return (src_set, exp)

    def paren_expr(self, clause, info):
        assert type(clause) is ParenClause
        assert type(info) is RefinedOpInfo
        (src_set, par_exp) = self.expression(clause.parexp, info)
        exp = '(%s)' % par_exp
        return (src_set, exp)

    def dst_operand(self, clause, info):
        assert type(clause) is DataRegClause
        assert type(info) is RefinedOpInfo
        frag = self.dst_fragment(clause, info)
        if not type(frag) is CodeFrag:
            import pdb; pdb.set_trace()
        var = frag.var + frag.vec

        if clause.idx:
            (idx_ss, idx_exp) = self.expression(clause.idx, info)
            frag.exp = idx_exp
        else:
            idx_ss = []

        (rng_ss, var) = self.range_expr(clause.rng, info, var)

        # dst frag comes first
        src_set = [ frag ]
        src_set.extend(idx_ss)
        src_set.extend(rng_ss)
        return (src_set, var)

    def src_operand(self, clause, info):
        assert type(clause) is DataRegClause
        assert type(info) is RefinedOpInfo
        frag = self.src_fragment(clause, info)
        var = frag.var + frag.vec

        if clause.idx:
            (idx_ss, idx_exp) = self.expression(clause.idx, info)
            frag.exp = idx_exp
        else:
            idx_ss = []

        (rng_ss, var) = self.range_expr(clause.rng, info, var)

        # src frag comes last
        src_set = idx_ss
        src_set.extend(rng_ss)
        src_set.append(frag)
        return (src_set, var)

    def expression(self, clause, info):
        if not isinstance(clause, Clause):
            import pdb; pdb.set_trace()
        assert isinstance(clause, Clause)
        assert type(info) is RefinedOpInfo
        if type(clause) is DataRegClause:
            (src_set, exp) = self.src_operand(clause, info)
        elif type(clause) is BinaryClause:
            (src_set, exp) = self.binary_expr(clause, info)
        elif type(clause) is ConditionalClause:
            (src_set, exp) = self.cond_expr(clause, info)
        elif type(clause) is UnaryClause:
            (src_set, exp) = self.unary_expr(clause, info)
        elif type(clause) is FunctionClause:
            (src_set, exp) = self.function_expr(clause, info)
        elif type(clause) is ConstantClause:
            (src_set, exp) = self.const_expr(clause, info)
        elif type(clause) is CastClause:
            (src_set, exp) = self.cast_expr(clause, info)
        elif type(clause) is FunctionClause:
            (src_set, exp) = self.func_expr(clause, info)
        elif type(clause) is CommaClause:
            (src_set, exp) = self.comma_expr(clause, info)
        elif type(clause) is GprClause:
            (src_set, exp) = self.src_gpr_expr(clause, info)
        elif type(clause) is MemClause:
            (src_set, exp) = self.src_mem_expr(clause, info)
        elif type(clause) is GroupClause:
            (src_set, exp) = self.group_expr(clause, info)
        elif type(clause) is ParenClause:
            (src_set, exp) = self.paren_expr(clause, info)
        else:
            exp = 'error'
            import pdb; pdb.set_trace()
        return (src_set, exp)

    def assignment(self, clause, info, g):
        assert type(clause) is AssignmentClause
        assert type(info) is RefinedOpInfo
        assert type(g) is GenOne

        dst_regs = []
        # process the left hand side
        if type(clause.dst) is DataRegClause:
            (dst_set, dst_var) = self.dst_operand(clause.dst, info)
            g.add_dst_set(dst_set)
            for d in dst_set:
                if d.key == 'dst' or d.key == 'mem':
                    dst_regs.append(d.var)
        elif type(clause.dst) is GprClause:
            (dst_set, dst_var) = self.dst_gpr_expr(clause.dst, info)
            g.add_dst_set(dst_set)
            for d in dst_set:
                if d.key == 'dst' or d.key == 'mem':
                    dst_regs.append(d.var)
        elif type(clause.dst) is MemClause:
            (dst_set, dst_var) = self.dst_mem_expr(clause.dst, info)
            g.add_dst_set(dst_set)
            for d in dst_set:
                if d.key == 'dst' or d.key == 'mem':
                    dst_regs.append(d.var)
        elif type(clause.dst) is GroupClause:
            dst_var = 'Group('
            comma = ''
            for grp in clause.dst.group:
                if type(grp) is DataRegClause:
                    (dst_set, grp_var) = self.dst_operand(grp, info)
                elif type(grp) is MemClause:
                    (dst_set, grp_var) = self.dst_mem_expr(grp, info)
                else:
                    import pdb; pdb.set_trace()
                g.add_dst_set(dst_set)
                for d in dst_set:
                    if d.key == 'dst' or d.key == 'mem':
                        dst_regs.append(d.var)
                dst_var += comma + grp_var
                comma = ', '
            dst_var += ')'
        else:
            assert False, 'unexpected clause'

        # process the right hand side
        (src_set, src_exp) = self.expression(clause.src, info)
        g.add_src_set(src_set)

        # main assignment string
        g.add_math('%s %s %s;' % (dst_var, clause.op, src_exp))

        # mark dst regs as modified so we don't load them later
        g.modified.extend(dst_regs)

    def ifthenelse(self, clause, info, g):
        assert type(clause) is IfThenElseClause
        assert type(info) is RefinedOpInfo
        assert type(g) is GenOne
        (cond_ss, cond_exp) = self.expression(clause.cond, info)
        g.add_src_set(cond_ss)
        g.add_math('if (%s) {' % cond_exp)
        g.inc_indent()
        for stmt in clause.then_stmt:
            if type(stmt) is AssignmentClause:
                self.assignment(stmt, info, g)
            elif type(stmt) is CommentClause:
                pass
            else:
                (then_ss, then_exp) = self.expression(stmt, info)
                g.add_src_set(then_ss)
                g.add_math('%s;' % then_exp)
        if clause.else_stmt:
            g.dec_indent()
            g.add_math('} else {')
            g.inc_indent()
            for stmt in clause.else_stmt:
                if type(stmt) is AssignmentClause:
                    self.assignment(stmt, info, g)
                elif type(stmt) is CommentClause:
                    pass
                else:
                    (else_ss, else_exp) = self.expression(stmt, info)
                    g.add_src_set(else_ss)
                    g.add_math('%s;' % else_exp)
        g.dec_indent()
        g.add_math('}')

    def if_clause(self, clause, info, g):
        assert type(clause) is IfClause
        assert type(info) is RefinedOpInfo
        assert type(g) is GenOne

    def tab_clause(self, clause, info, g):
        assert type(clause) is TabClause
        assert type(info) is RefinedOpInfo
        assert type(g) is GenOne

    def else_clause(self, clause, info, g):
        assert type(clause) is ElseClause
        assert type(info) is RefinedOpInfo
        assert type(g) is GenOne

    def chain_clause(self, clause, info, g):
        assert type(clause) is ChainClause
        assert type(info) is RefinedOpInfo
        assert type(g) is GenOne
        if type(clause.right) is AssignmentClause:
            self.assignment(clause.right, info, g)
        right_dst = copy.deepcopy(clause.right.dst)
        right_dst.rng = None
        assign = AssignmentClause()
        assign.dst = clause.left
        assign.op = '='
        assign.src = right_dst
        g.set_vector(False)
        self.assignment(assign, info, g)
        g.set_vector('OPF_VECTOR' in info.flags)

    def generate_execute_code(self, op_inst, info, ast, cg):
        assert type(info) is RefinedOpInfo
        assert type(ast) is list
        assert type(cg) is CodeGen
        # fix a bug in the sq_uc.arch source
        if 'OPF_MOVRELS' in info.flags:
            for si in info.src:
                if si.iseq == 'SREG' and si.name == 'sdst':
                    si.name = 'ssrc'
        is_vec = 'OPF_VECTOR' in info.flags
        gen_one = GenOne(op_inst, cg, is_vec, self.methods, info)
        self.prefix = ''
        for clause in ast:
            if type(clause) is AssignmentClause:
                self.assignment(clause, info, gen_one)
            elif type(clause) is IfThenElseClause:
                self.ifthenelse(clause, info, gen_one)
            elif type(clause) is IfClause:
                self.if_clause(clause, info, gen_one)
            elif type(clause) is TabClause:
                self.tab_clause(clause, info, gen_one)
            elif type(clause) is ElseClause:
                self.else_clause(clause, info, gen_one)
            elif type(clause) is ChainClause:
                self.chain_clause(clause, info, gen_one)
            elif type(clause) is SizeClause:
                if 'OPF_MEM_ATOMIC' in info.flags:
                    #import pdb; pdb.set_trace()
                    self.prefix = 'SregU%d' % clause.size
                    info_copy = copy.deepcopy(info)
                    info_copy.dst[0].size = clause.size / 32
                    info = info_copy
            elif type(clause) is CommentClause:
                pass
            else:
                import pdb; pdb.set_trace()
                assert False, 'should not get here'
        return gen_one.finish()

    def parse_and_generate(self, op_inst, info, cg):
        assert type(op_inst) is str
        assert type(info) is RefinedOpInfo
        assert type(cg) is CodeGen
        try:
            ast = self.desc_parser.parse_description(info.desc)
            # print '--------------------------\n%s' % (op_inst)
            # pprint(info.desc)
            # pprint(ast)
            if not self.generate_execute_code(op_inst, info, ast, cg):
                cg.cg_comment('Could not parse sq_uc.arch desc field')
                cg.cg_code('//gpuDynInst->warnUnimplemented("TBD: %s");'
                           % op_inst)
                return 'empty'
        except ParseError:
            # print '--------------------------\n%s' % (op_inst)
            # pprint(info.desc)
            cg.cg_comment('Could not parse sq_uc.arch desc field')
            cg.cg_code('//gpuDynInst->warnUnimplemented("TBD: %s");'
                       % op_inst)
            return 'except'
        except:
            raise
        return 'success'

    def setOpTypeFlags(self, info, op_enc, op_op, cg):
        # Op type flags:
        # Nop - op_op contains NOP
        # ALU
        # Branch - op_op contains BRANCH or CBRANCH
        # Conditional Branch - op_op contains CBRANCH
        # Return - not implemented
        # MemFence - no matching VI ISA operations
        # MemBarrier - S_BARRIER
        # Flat - op_enc = FLAT
        # SpecialOp - not implemented
        if 'CBRANCH' in op_op:
            cg.cg_code('setFlag(CondBranch);')
        if op_op == 'S_BARRIER':
            cg.cg_code('setFlag(MemBarrier);')
        elif op_op == 'S_WAITCNT':
            cg.cg_code('setFlag(ALU);')
            cg.cg_code('setFlag(Waitcnt);')
        elif op_op == 'S_ENDPGM':
            cg.cg_code('setFlag(ALU);')
        elif 'NOP' in op_op:
            cg.cg_code('setFlag(Nop);')
            if op_op == 'V_NOP':
                cg.cg_code('setFlag(ALU);')
        elif 'BRANCH' in op_op:
            cg.cg_code('setFlag(Branch);')
        elif op_op in ['S_SETPC', 'S_SWAPPC', 'S_SETVSKIP']:
            cg.cg_code('setFlag(UnconditionalJump);')
        # set ALU flag for each encoding
        elif (op_enc == 'SOP2' and not op_op in
              ['S_CBRANCH_G_FORK', 'S_RFE_RESTORE_B64']):
            cg.cg_code('setFlag(ALU);')
        elif (op_enc == 'SOPK' and not op_op in
              ['S_GETREG_B32', 'S_SETREG_B32', 'S_SETREG_IMM32_B32']):
            cg.cg_code('setFlag(ALU);')
        elif (op_enc == 'SOP1' and not op_op in
              ['S_GETPC_B64', 'S_SETPC_B64', 'S_SWAPPC_B64', 'S_RFE_B64',
              'S_SET_GPR_IDX_IDX']):
            cg.cg_code('setFlag(ALU);')
        elif (op_enc == 'SOPC' and not op_op in
              ['S_SETVSKIP', 'S_SET_GPR_IDX_ON']):
            cg.cg_code('setFlag(ALU);')
        elif op_enc == 'VOP2':
            cg.cg_code('setFlag(ALU);')
        elif op_enc == 'VOP1' and not op_op in ['V_READFIRSTLANE_B32']:
            cg.cg_code('setFlag(ALU);')
        elif op_enc == 'VOPC':
            cg.cg_code('setFlag(ALU);')
        elif op_enc == 'VINTRP':
            cg.cg_code('setFlag(ALU);')
        elif (op_enc == 'VOP3' and not op_op in
              ['V_CLREXCP', 'V_READLANE_B32', 'V_WRITELANE_B32']):
            cg.cg_code('setFlag(ALU);')
        elif (op_enc in ['SOPP', 'SMEM', 'DS', 'MUBUF', 'MTBUF', 'MIMG', 'EXP',
             'FLAT']):
            # remaining opcodes in these encodings are
            # considered ALU operations
            pass

    def setMemoryAccessFlags(self, info, op_enc, op_op, cg):
        # set memory access flags for non-atomic operations
        if 'OPF_MEM_STORE' in info.flags or 'DS_WRITE' in op_op:
            cg.cg_code('setFlag(MemoryRef);')
            cg.cg_code('setFlag(Store);')
        elif 'LOAD' in op_op or 'DS_READ' in op_op:
            cg.cg_code('setFlag(MemoryRef);')
            cg.cg_code('setFlag(Load);')

    def setSegmentAccessFlags(self, info, op_enc, op_op, cg):
        if op_enc in ['MUBUF', 'MTBUF', 'MIMG']:
            cg.cg_code('setFlag(GlobalSegment);')

    def setAtomicFlags(self, info, op_enc, op_op, cg):
        ops = {'AND' : 'AtomicAnd',
               'OR' : 'AtomicOr',
               'XOR' : 'AtomicXor',
               'CMPSWAP' : 'AtomicCAS',
               'ADD' : 'AtomicAdd',
               'SUB' : 'AtomicSub',
               'INC' : 'AtomicInc',
               'DEC' : 'AtomicDec',
               'MAX' : 'AtomicMax',
               'SMAX' : 'AtomicMax',
               'UMAX' : 'AtomicMax',
               'MIN' : 'AtomicMin',
               'SMIN' : 'AtomicMin',
               'UMIN' : 'AtomicMin',
               'SWAP' : 'AtomicExch'
        }
        atomic_op = re.sub('.*ATOMIC_', '', op_op)
        atomic_op = re.sub('_.*', '', atomic_op)

        if atomic_op in ops.keys():
            cg.cg_code('setFlag(%s);' % ops[atomic_op])
            # for atomics, the GLC bit determines if
            # an atomic returns the pre-op value
            cg.cg_if('instData.GLC')
            cg.cg_code('setFlag(AtomicReturn);')
            cg.cg_else()
            cg.cg_code('setFlag(AtomicNoReturn);')
            cg.cg_end('if')
            cg.cg_code('setFlag(MemoryRef);')


    def setModeFlags(self, info, op_op, cg):
        if ('S_SETVSKIP' in op_op or 'S_SETREG' in op_op
            or 'S_SET_GPR' in op_op):
            cg.cg_code('setFlag(WritesMode);')
        if 'S_GETREG' in op_op:
            cg.cg_code('setFlag(ReadsMode);')

    def setEXECFlags(self, info, cg):
        if 'OPF_RDEX' in info.flags:
            cg.cg_code('setFlag(ReadsEXEC);')
        if 'OPF_WREX' in info.flags:
            cg.cg_code('setFlag(WritesEXEC);')

    def setVCCFlags(self, info, cg):
        if 'OPF_VCCD' in info.flags:
            cg.cg_code('setFlag(WritesVCC);')
        if 'OPF_VCCS' in info.flags or 'OPF_RDVCC' in info.flags:
            cg.cg_code('setFlag(ReadsVCC);')

    def setFlags(self, info, op_inst, op_enc, op_op, cg):
        self.setOpTypeFlags(info, op_enc, op_op, cg)
        self.setVCCFlags(info, cg)
        self.setEXECFlags(info, cg)
        self.setModeFlags(info, op_op, cg)
        if 'OPF_MEM_ATOMIC' in info.flags:
            self.setAtomicFlags(info, op_enc, op_op, cg)
        else:
            self.setMemoryAccessFlags(info, op_enc, op_op, cg)
        self.setSegmentAccessFlags(info, op_enc, op_op, cg)

    # instructions.cc
    def generate_instructions_cc(self, output_dir):
        file = os.path.join(output_dir, 'instructions.cc')
        cg = CodeGen(file)

        cg.cg_include('<cmath>')
        cg.cg_newline()

        cg.cg_include('gpu-compute/shader.hh')
        cg.cg_include('gpu-internal/arch/vi/instructions.hh')
        cg.cg_include('gpu-internal/arch/vi/inst_util.hh')
        cg.cg_newline()

        cg.cg_namespace('ViISA')

        instruction_index = 0
        empty_count = 0
        exception_count = 0
        found_exceptions = []
        known_except = copy.deepcopy(KnownExceptions)
        found_empty = []
        known_empty = copy.deepcopy(KnownEmpty)

        for info in self.refined_op_info:
            op_op = re.sub('.*__', '', info.name)
            op_enc = re.sub('__.*', '', info.name)
            op_inst = 'Inst_%s__%s' % (op_enc, op_op)
            op_base = 'Inst_%s' % op_enc
            op_fmt = 'InFmt_%s' % op_enc
            if op_fmt == 'InFmt_VOP3':
                vccd = 'OPF_VCCD' in info.flags
                vopc = 'OPF_PEN_VOPC' in info.flags
                if vccd or vopc:
                    op_base = 'Inst_VOP3_SDST_ENC'
                    op_fmt = 'InFmt_VOP3_SDST_ENC'
            arg = '%s *iFmt' % op_fmt
            ini = '%s(iFmt, "%s")' % (op_base, op_op.lower())

            cg.cg_comment('--- %s class methods ---' % op_inst)
            cg.cg_newline()
            cg.cg_method(None, op_inst, op_inst, [arg], [ini])
            self.setFlags(info, op_inst, op_enc, op_op, cg) # set Flags
            cg.cg_end(op_inst) # cg_method
            cg.cg_newline()
            cg.cg_method(None, op_inst, '~%s' % op_inst, [], [])
            cg.cg_end('~%s' % op_inst) # cg_method
            cg.cg_newline()

            cg.cg_comment('--- description from .arch file ---')
            edited = []
            for d in info.desc:
                edited.extend(d.split('\\n'))
            for e in edited:
                if e:
                    line = e.replace('\\t', '    ')
                    lead = ''
                    abs_max = 80 - 3 # cg_comment prepends '// '
                    len_max = abs_max - len(lead)
                    while len(line) > len_max:
                        brk = line.rfind(' ', 0, len_max)
                        cg.cg_comment(lead + line[0:brk])
                        line = line[brk:]
                        lead = '--- '
                        len_max = abs_max - len(lead)
                    cg.cg_comment(lead + line)
            cg.cg_method('void', op_inst, 'execute',
                         ['GPUDynInstPtr gpuDynInst'], [])
            # breakpoint
            #if op_inst == 'Inst_MUBUF__BUFFER_ATOMIC_SWAP':
            #    import pdb; pdb.set_trace()
            if op_inst in HandCodedExecMethods.keys():
                cg.cg_block(HandCodedExecMethods[op_inst])
                cg.cg_end('execute') # cg_method
            else:
                result = self.parse_and_generate(op_inst, info, cg)
                if result == 'success':
                    pass
                elif result == 'empty':
                    empty_count += 1
                    if op_inst not in known_empty:
                        found_empty.append(op_inst)
                        print 'Parse Empty'
                        pprint(found_empty)
                        import pdb; pdb.set_trace()
                    else:
                        known_empty.remove(op_inst)
                    cg.cg_end('execute') # cg_method
                elif result == 'except':
                    exception_count += 1
                    if op_inst not in known_except:
                        found_exceptions.append(op_inst)
                        print 'Parse Error'
                        pprint(found_exceptions)
                        import pdb; pdb.set_trace()
                    else:
                        known_except.remove(op_inst)
                    cg.cg_end('execute') # cg_method
            instruction_index += 1
        if known_empty:
            print 'Problems in instructions %s are no longer seen.' % (
                repr(known_empty))
        if known_except:
            print 'Exceptions in instructions %s are no longer seen.' % (
                repr(known_except))
        i = instruction_index
        e = empty_count
        print 'Found no pseudo code in %d of %d instructions' % (e, i)
        e = exception_count
        print 'Found exceptions in %d of %d instructions' % (e, i)

        for op_enc in self.inst_with_encodings:
            if op_enc == 'EXP':
                op_op = 'default'
            else:
                op_op = 'invalid'
            op_inst = 'Inst_%s__%s' % (op_enc, op_op)
            cg.cg_newline()
            cg.cg_comment('--- %s class methods ---' % op_inst)
            arg = 'InFmt_%s *iFmt' % op_enc
            ini = 'Inst_%s(iFmt, "%s_%s")' % (op_enc, op_enc, op_op)
            cg.cg_method(None, op_inst, op_inst, [arg], [ini])
            cg.cg_end(op_inst) # cg_method
            cg.cg_method(None, op_inst, '~%s' % op_inst, [], [])
            cg.cg_end('~%s' % op_inst) # cg_method
            cg.cg_newline()
            cg.cg_method('bool', op_inst, 'isValid', [], [], 'const')
            cg.cg_code('return false;')
            cg.cg_end('isValid') # cg_method
            cg.cg_method('void', op_inst, 'execute',
                         ['GPUDynInstPtr gpuDynInst'], [])
            cg.cg_code('//gpuDynInst->warnUnimplemented("%s");' % op_inst)
            cg.cg_end('execute') # cg_method

        op_inst = 'Inst_invalid'
        cg.cg_newline()
        cg.cg_comment('--- %s class methods ---' % op_inst)
        cg.cg_method(None, op_inst, op_inst,
                     ['MachInst'],
                     ['ViGPUStaticInst("Inst_invalid")'])
        cg.cg_end(op_inst) # cg_method
        cg.cg_method(None, op_inst, '~%s' % op_inst, [], [])
        cg.cg_end('~%s' % op_inst) # cg_method
        cg.cg_newline()
        cg.cg_method('uint32_t', op_inst, 'instSize', [], [])
        cg.cg_code('return 4;')
        cg.cg_end('instSize') # cg_method
        cg.cg_method('bool', op_inst, 'isValid', [], [], 'const')
        cg.cg_code('return false;')
        cg.cg_end('isValid') # cg_method

        cg.cg_end('namespace ViISA') # cg_namespace

        cg.generate()

    def generate_code(self, output_dir):
        # pprint(self.refined_op_info)
        self.generate_decoder_hh(output_dir)
        self.generate_decoder_cc(output_dir)
        self.generate_instructions_cc(output_dir)
        self.generate_instructions_hh(output_dir)
