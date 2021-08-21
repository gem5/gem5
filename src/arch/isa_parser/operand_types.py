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

class Operand(object):
    '''Base class for operand descriptors.  An instance of this class
    (or actually a class derived from this one) represents a specific
    operand for a code block (e.g, "Rc.sq" as a dest). Intermediate
    derived classes encapsulates the traits of a particular operand
    type (e.g., "32-bit integer register").'''

    src_reg_constructor = '\n\tsetSrcRegIdx(_numSrcRegs++, RegId(%s, %s));'
    dst_reg_constructor = '\n\tsetDestRegIdx(_numDestRegs++, RegId(%s, %s));'

    def buildReadCode(self, predRead, func=None):
        subst_dict = {"name": self.base_name,
                      "func": func,
                      "reg_idx": self.reg_spec,
                      "ctype": self.ctype}
        if hasattr(self, 'src_reg_idx'):
            subst_dict['op_idx'] = \
                    '_sourceIndex++' if predRead else str(self.src_reg_idx)
        code = self.read_code % subst_dict
        return '%s = %s;\n' % (self.base_name, code)

    def buildWriteCode(self, predWrite, func=None):
        subst_dict = {"name": self.base_name,
                      "func": func,
                      "reg_idx": self.reg_spec,
                      "ctype": self.ctype,
                      "final_val": self.base_name}
        if hasattr(self, 'dest_reg_idx'):
            subst_dict['op_idx'] = \
                    '_destIndex++' if predWrite else str(self.dest_reg_idx)
        code = self.write_code % subst_dict
        return '''
        {
            %s final_val = %s;
            %s;
            if (traceData) { traceData->setData(final_val); }
        }''' % (self.dflt_ctype, self.base_name, code)

    def __init__(self, parser, full_name, ext, is_src, is_dest):
        self.parser = parser
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

class RegOperand(Operand):
    def isReg(self):
        return 1

    def makeConstructor(self, predRead, predWrite):
        c_src = ''
        c_dest = ''

        if self.is_src:
            c_src = self.src_reg_constructor % (self.reg_class, self.reg_spec)
            if self.hasReadPred():
                c_src = '\n\tif (%s) {%s\n\t}' % \
                        (self.read_predicate, c_src)

        if self.is_dest:
            c_dest = self.dst_reg_constructor % (self.reg_class, self.reg_spec)
            c_dest += f'\n\t_numTypedDestRegs[{self.reg_class}]++;'
            if self.hasWritePred():
                c_dest = '\n\tif (%s) {%s\n\t}' % \
                         (self.write_predicate, c_dest)

        return c_src + c_dest

class IntRegOperand(RegOperand):
    reg_class = 'IntRegClass'

    def makeRead(self, predRead):
        if (self.ctype == 'float' or self.ctype == 'double'):
            error('Attempt to read integer register as FP')
        if self.read_code != None:
            return self.buildReadCode(predRead, 'getRegOperand')

        int_reg_val = ''
        if predRead:
            int_reg_val = 'xc->getRegOperand(this, _sourceIndex++)'
            if self.hasReadPred():
                int_reg_val = '(%s) ? %s : 0' % \
                              (self.read_predicate, int_reg_val)
        else:
            int_reg_val = 'xc->getRegOperand(this, %d)' % self.src_reg_idx

        return '%s = %s;\n' % (self.base_name, int_reg_val)

    def makeWrite(self, predWrite):
        if (self.ctype == 'float' or self.ctype == 'double'):
            error('Attempt to write integer register as FP')
        if self.write_code != None:
            return self.buildWriteCode(predWrite, 'setRegOperand')

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
            xc->setRegOperand(this, %s, final_val);\n
            if (traceData) { traceData->setData(final_val); }
        }''' % (wcond, self.ctype, self.base_name, windex)

        return wb

class FloatRegOperand(RegOperand):
    reg_class = 'FloatRegClass'

    def makeRead(self, predRead):
        if self.read_code != None:
            return self.buildReadCode(predRead, 'getRegOperand')

        if predRead:
            rindex = '_sourceIndex++'
        else:
            rindex = '%d' % self.src_reg_idx

        code = 'xc->getRegOperand(this, %s)' % rindex
        if self.ctype == 'float':
            code = 'bitsToFloat32(%s)' % code
        elif self.ctype == 'double':
            code = 'bitsToFloat64(%s)' % code
        return '%s = %s;\n' % (self.base_name, code)

    def makeWrite(self, predWrite):
        if self.write_code != None:
            return self.buildWriteCode(predWrite, 'setRegOperand')

        if predWrite:
            wp = '_destIndex++'
        else:
            wp = '%d' % self.dest_reg_idx

        val = 'final_val'
        if self.ctype == 'float':
            val = 'floatToBits32(%s)' % val
        elif self.ctype == 'double':
            val = 'floatToBits64(%s)' % val

        wp = 'xc->setRegOperand(this, %s, %s);' % (wp, val)

        wb = '''
        {
            %s final_val = %s;
            %s\n
            if (traceData) { traceData->setData(final_val); }
        }''' % (self.ctype, self.base_name, wp)
        return wb

class VecRegOperand(RegOperand):
    reg_class = 'VecRegClass'

    def __init__(self, parser, full_name, ext, is_src, is_dest):
        super().__init__(parser, full_name, ext, is_src, is_dest)
        self.elemExt = None

    def makeDeclElem(self, elem_op):
        (elem_name, elem_ext) = elem_op
        (elem_spec, dflt_elem_ext) = self.elems[elem_name]
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

    # Read destination register to write
    def makeReadWElem(self, elem_op):
        (elem_name, elem_ext) = elem_op
        (elem_spec, dflt_elem_ext) = self.elems[elem_name]
        if elem_ext:
            ext = elem_ext
        else:
            ext = dflt_elem_ext
        ctype = self.parser.operandTypeMap[ext]
        c_read = '\t\t%s& %s = %s[%s];\n' % \
                  (ctype, elem_name, self.base_name, elem_spec)
        return c_read

    def makeReadW(self, predWrite):
        func = 'getWritableRegOperand'
        if self.read_code != None:
            return self.buildReadCode(predWrite, func)

        if predWrite:
            rindex = '_destIndex++'
        else:
            rindex = '%d' % self.dest_reg_idx

        c_readw = f'\t\tauto &tmp_d{rindex} = \n' \
                  f'\t\t    *({self.parser.namespace}::VecRegContainer *)\n' \
                  f'\t\t    xc->{func}(this, {rindex});\n'
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
        (elem_spec, dflt_elem_ext) = self.elems[elem_name]

        if elem_ext:
            ext = elem_ext
        else:
            ext = dflt_elem_ext
        ctype = self.parser.operandTypeMap[ext]
        c_read = '\t\t%s = %s[%s];\n' % \
                  (elem_name, name, elem_spec)
        return c_read

    def makeRead(self, predRead):
        func = 'getRegOperand'
        if self.read_code != None:
            return self.buildReadCode(predRead, func)

        if predRead:
            rindex = '_sourceIndex++'
        else:
            rindex = '%d' % self.src_reg_idx

        name = self.base_name
        if self.is_dest and self.is_src:
            name += '_merger'

        c_read = f'\t\t{self.parser.namespace}::VecRegContainer ' \
                 f'\t\t        tmp_s{rindex};\n' \
                 f'\t\txc->{func}(this, {rindex}, &tmp_s{rindex});\n'
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
        func = 'setRegOperand'
        if self.write_code != None:
            return self.buildWriteCode(predWrite, func)

        wb = '''
        if (traceData) {
            traceData->setData(tmp_d%d);
        }
        ''' % self.dest_reg_idx
        return wb

    def finalize(self, predRead, predWrite):
        super().finalize(predRead, predWrite)
        if self.is_dest:
            self.op_rd = self.makeReadW(predWrite) + self.op_rd

class VecElemOperand(RegOperand):
    reg_class = 'VecElemClass'

    def makeRead(self, predRead):
        c_read = f'xc->getRegOperand(this, {self.src_reg_idx})'

        if self.ctype == 'float':
            c_read = f'bitsToFloat32({c_read})'
        elif self.ctype == 'double':
            c_read = f'bitsToFloat64({c_read})'

        return f'{self.base_name} = {c_read};\n'

    def makeWrite(self, predWrite):
        val = self.base_name
        if self.ctype == 'float':
            val = f'floatToBits32({val})'
        elif self.ctype == 'double':
            val = f'floatToBits64({val})'
        return f'\n\txc->setRegOperand(this, {self.dest_reg_idx}, {val});'

class VecPredRegOperand(RegOperand):
    reg_class = 'VecPredRegClass'

    def makeDecl(self):
        return ''

    def makeRead(self, predRead):
        func = 'getRegOperand'
        if self.read_code != None:
            return self.buildReadCode(predRead, func)

        if predRead:
            rindex = '_sourceIndex++'
        else:
            rindex = '%d' % self.src_reg_idx

        c_read =  f'\t\t{self.parser.namespace}::VecPredRegContainer ' \
                  f'\t\t        tmp_s{rindex}; ' \
                  f'xc->{func}(this, {rindex}, &tmp_s{rindex});\n'
        if self.ext:
            c_read += f'\t\tauto {self.base_name} = ' \
                      f'tmp_s{rindex}.as<' \
                      f'{self.parser.operandTypeMap[self.ext]}>();\n'
        return c_read

    def makeReadW(self, predWrite):
        func = 'getWritableRegOperand'
        if self.read_code != None:
            return self.buildReadCode(predWrite, func)

        if predWrite:
            rindex = '_destIndex++'
        else:
            rindex = '%d' % self.dest_reg_idx

        c_readw = f'\t\tauto &tmp_d{rindex} = \n' \
                  f'\t\t    *({self.parser.namespace}::' \
                  f'VecPredRegContainer *)xc->{func}(this, {rindex});\n'
        if self.ext:
            c_readw += '\t\tauto %s = tmp_d%s.as<%s>();\n' % (
                    self.base_name, rindex,
                    self.parser.operandTypeMap[self.ext])
        return c_readw

    def makeWrite(self, predWrite):
        func = 'setRegOperand'
        if self.write_code != None:
            return self.buildWriteCode(predWrite, func)

        wb = '''
        if (traceData) {
            traceData->setData(tmp_d%d);
        }
        ''' % self.dest_reg_idx
        return wb

    def finalize(self, predRead, predWrite):
        super().finalize(predRead, predWrite)
        if self.is_dest:
            self.op_rd = self.makeReadW(predWrite) + self.op_rd

class CCRegOperand(RegOperand):
    reg_class = 'CCRegClass'

    def makeRead(self, predRead):
        if (self.ctype == 'float' or self.ctype == 'double'):
            error('Attempt to read condition-code register as FP')
        if self.read_code != None:
            return self.buildReadCode(predRead, 'getRegOperand')

        int_reg_val = ''
        if predRead:
            int_reg_val = 'xc->getRegOperand(this, _sourceIndex++)'
            if self.hasReadPred():
                int_reg_val = '(%s) ? %s : 0' % \
                              (self.read_predicate, int_reg_val)
        else:
            int_reg_val = 'xc->getRegOperand(this, %d)' % self.src_reg_idx

        return '%s = %s;\n' % (self.base_name, int_reg_val)

    def makeWrite(self, predWrite):
        if (self.ctype == 'float' or self.ctype == 'double'):
            error('Attempt to write condition-code register as FP')
        if self.write_code != None:
            return self.buildWriteCode(predWrite, 'setRegOperand')

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
            xc->setRegOperand(this, %s, final_val);\n
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
            c_src = self.src_reg_constructor % (self.reg_class, self.reg_spec)

        if self.is_dest:
            c_dest = self.dst_reg_constructor % (self.reg_class, self.reg_spec)

        return c_src + c_dest

    def makeRead(self, predRead):
        bit_select = 0
        if (self.ctype == 'float' or self.ctype == 'double'):
            error('Attempt to read control register as FP')
        if self.read_code != None:
            return self.buildReadCode(predRead, 'readMiscRegOperand')

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
            return self.buildWriteCode(predWrite, 'setMiscRegOperand')

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
        return '%s %s = {};\n' % (self.ctype, self.base_name)

    def makeRead(self, predRead):
        if self.read_code != None:
            return self.buildReadCode(predRead)
        return ''

    def makeWrite(self, predWrite):
        if self.write_code != None:
            return self.buildWriteCode(predWrite)
        return ''

class PCStateOperand(Operand):
    def __init__(self, parser, *args, **kwargs):
        super().__init__(parser, *args, **kwargs)
        self.parser = parser

    def makeConstructor(self, predRead, predWrite):
        return ''

    def makeRead(self, predRead):
        if self.reg_spec:
            # A component of the PC state.
            return '%s = __parserAutoPCState.%s();\n' % \
                (self.base_name, self.reg_spec)
        else:
            # The whole PC state itself.
            return f'{self.base_name} = ' \
                    f'xc->pcState().as<{self.parser.namespace}::PCState>();\n'

    def makeWrite(self, predWrite):
        if self.reg_spec:
            # A component of the PC state.
            return '__parserAutoPCState.%s(%s);\n' % \
                (self.reg_spec, self.base_name)
        else:
            # The whole PC state itself.
            return f'xc->pcState({self.base_name});\n'

    def makeDecl(self):
        ctype = f'{self.parser.namespace}::PCState'
        if self.isPCPart():
            ctype = self.ctype
        # Note that initializations in the declarations are solely
        # to avoid 'uninitialized variable' errors from the compiler.
        return '%s %s = 0;\n' % (ctype, self.base_name)

    def isPCState(self):
        return 1
