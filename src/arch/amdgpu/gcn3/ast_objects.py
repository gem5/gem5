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

class Statement(object):
    def __init__(self):
        self.keyword = ''

class ImportStatement(Statement):
    def __init__(self):
        self.keyword = 'import'
        self.what = ''
        self.name = ''

class FlagBlock(Statement):
    def __init__(self):
        self.keyword = 'flag'
        self.name = ''
        self.desc = []

    def update(self, ref):
        if ref.tag == 'name':
            self.name = ref.name
        elif ref.tag == 'desc':
            self.desc = ref.desc
        elif ref.tag == 'desc+':
            self.desc = self.desc + ref.desc
        else:
            assert (False), 'error: unexpected FlagBlock.tag' + ref.tag
        self.tag = 'updated'

class FlagsField(object):
    def __init__(self):
        self.tag = ''
        self.name = ''
        self.private = 0
        self.desc = []
        self.group = ''

    def update(self, ref):
        if ref.tag == 'name':
            self.name = ref.name
        elif ref.tag == 'private':
            self.private = ref.private
        elif ref.tag == 'desc':
            self.desc = ref.desc
        elif ref.tag == 'desc+':
            self.desc = self.desc + ref.desc
        elif ref.tag == 'group':
            self.group = ref.group
        else:
            assert (False),  'error: unexpected FlagsField.tag ' + ref.tag
        self.tag = 'updated'

class FlagsBlock(Statement):
    def __init__(self):
        self.keyword = 'flags'
        self.clauses = []

class OpInfo(object):
    def __init__(self):
        self.tag = ''
        self.opr = ''
        self.index = -1
        self.iseq = ''
        self.name = ''
        self.fmt = ''
        self.size = -1
        self.inout = -1

    def __repr__(self):
        text = '\n    [\n'
        text += '\topr:   ' + repr(self.opr) + ',\n'
        text += '\tindex: ' + repr(self.index) + ',\n'
        text += '\tiseq:  ' + repr(self.iseq) + ',\n'
        text += '\tname:  ' + repr(self.name) + ',\n'
        text += '\tfmt:   ' + repr(self.fmt) + ',\n'
        text += '\tsize:  ' + repr(self.size) + ',\n'
        text += '\tinout: ' + repr(self.inout) + '\n'
        text += '\n    ]\n'
        return text

    def update(self, ref):
        if ref.tag == 'dst':
            self.opr = ref.opr
            self.index = ref.index
            self.iseq = ref.iseq
        elif ref.tag == 'src':
            self.opr = ref.opr
            self.index = ref.index
            self.iseq = ref.iseq
        elif ref.tag == 'name':
            self.name = ref.name
        elif ref.tag == 'fmt':
            self.fmt = ref.fmt
        elif ref.tag == 'size':
            self.size = ref.size
        elif ref.tag == 'inout':
            self.inout = ref.inout
        else:
            assert (False), 'error: unexpected OpInfo.tag ' + ref.tag
        self.tag = 'updated'

    def match(self, ref):
        if self.opr != ref.opr:
            return False
        if self.index != ref.index:
            return False
        return True

    def override(self, ref):
        if ref.iseq:
            self.iseq = ref.iseq
        if ref.name:
            self.name = ref.name
        if ref.fmt:
            self.fmt = ref.fmt
        if ref.size >= 0:
            self.size = ref.size
        if ref.inout >= 0:
            self.inout = ref.inout

class Operand(object):
    def __init__(self):
        self.tag = ''
        self.num_dst = -1
        self.num_src = -1
        self.parent_enc = ''
        self.sub_enc = ''
        self.flags = []
        self.dst = []
        self.src = []
        self.when = []
        self.operands = []

    def __repr__(self):
        text  = '\tnum_dst: ' + repr(self.num_dst) + ',\n'
        text += '\tnum_src: ' + repr(self.num_src) + ',\n'
        text += '\tparent_enc: ' + repr(self.parent_enc) + ',\n'
        text += '\tsub_enc: ' + repr(self.sub_enc) + ',\n'
        text += '\tflags: ' + repr(self.flags) + ',\n'
        text += '\tdst: ' + repr(self.dst) + ',\n'
        text += '\tsrc: ' + repr(self.src) + ',\n'
        text += '\twhen: ' + repr(self.when) + ',\n'
        text += '\toperands: ' + repr(self.operands) + '\n'
        return text

    def update(self, ref):
        if ref.tag == 'num_dst':
            self.num_dst = ref.num_dst
        elif ref.tag == 'num_src':
            self.num_src = ref.num_src
        elif ref.tag == 'parent_enc':
            self.parent_enc = ref.parent_enc
        elif ref.tag == 'sub_enc':
            self.sub_enc = ref.sub_enc
        elif ref.tag == 'flags':
            self.flags = self.flags + ref.flags
        elif ref.tag == 'dst':
            self.dst = self.dst + ref.dst
        elif ref.tag == 'src':
            self.src = self.src + ref.src
        elif ref.tag == 'when':
            self.when = self.when + ref.when
        elif ref.tag == 'operands':
            self.operands = self.operands + ref.operands
        else:
            assert (False), 'error: unexpected Operand.tag ' + ref.tag
        self.tag = 'updated'

    def override(self, ref):
        if ref.num_dst > 0:
            self.num_dst = ref.num_dst
        if ref.num_src > 0:
            self.num_src = ref.num_src
        for r in ref.dst:
            for s in self.dst:
                if s.match(r):
                    s.override(r)
        for r in ref.src:
            for s in self.src:
                if s.match(r):
                    s.override(r)

class WhenBlock(object):
    def __init__(self):
        self.left = ''
        self.right = []
        self.operand = None

class EncodingBlock(Statement):
    def __init__(self):
        self.keyword = 'encoding'
        self.tag = ''
        self.name = ''
        self.bits = ''
        self.size = -1
        self.desc = []
        self.operands = []

    def update(self, ref):
        if ref.tag == 'name':
            self.name = ref.name
        elif ref.tag == 'bits':
            self.bits = ref.bits
        elif ref.tag == 'size':
            self.size = ref.size
        elif ref.tag == 'desc':
            self.desc = self.desc + ref.desc
        elif ref.tag == 'operands':
            self.operands = self.operands + ref.operands
        else:
            assert (False), 'error: unexpected EncodingBlock.tag ' + ref.tag
        self.tag = 'updated'

class ConstClause(object):
    def __init__(self):
        self.name = ''
        self.value = 0

class ConstBlock(Statement):
    def __init__(self):
        self.keyword = 'const'
        self.clauses = []

class TypeClause(object):
    def __init__(self):
        self.tag = ''
        self.name = ''
        self.v_max = 0
        self.value = 0
        self.dp_only = 0
        self.size = -1
        self.var = False
        self.desc = []
        self.flags = []
        self.src_flags = ''
        self.sp3_desc = []
        self.sp3_name = ''
        self.sp3_ncomp = 0
        self.sp3_num = ''
        self.parent_enc = ''
        self.sub_enc = ''
        self.op_type = ''
        self.fmt = ''
        self.type = ''
        self.range = []
        self.size_bits = -1

    def update(self, ref):
        if ref.tag == 'id_range':
            self.name = ref.name
            self.v_max = ref.v_max
            self.value = ref.value
        elif ref.tag == 'id_number':
            self.name = ref.name
            self.value = ref.value
        elif ref.tag == 'id_var_number':
            self.name = ref.name
            self.value = ref.value
            self.var = ref.var
        elif ref.tag == 'desc':
            self.desc = ref.desc + self.desc
        elif ref.tag == 'desc+':
            self.desc = self.desc + ref.desc
        elif ref.tag == 'flags':
            self.flags = ref.flags + self.flags
        elif ref.tag == 'flags+':
            self.flags = self.flags + ref.flags
        elif ref.tag == 'src_flags':
            self.src_flags = ref.src_flags
        elif ref.tag == 'sp3_desc':
            self.sp3_desc = ref.sp3_desc + self.sp3_desc
        elif ref.tag == 'sp3_desc+':
            self.sp3_desc = self.sp3_desc + ref.sp3_desc
        elif ref.tag == 'sp3_name':
            self.sp3_name = ref.sp3_name
        elif ref.tag == 'sp3_ncomp':
            self.sp3_ncomp = ref.sp3_ncomp
        elif ref.tag == 'sp3_num':
            self.sp3_num = ref.sp3_num
        elif ref.tag == 'parent_enc':
            self.parent_enc = ref.parent_enc
        elif ref.tag == 'sub_enc':
            self.sub_enc = ref.sub_enc
        elif ref.tag == 'op_type':
            self.op_type = ref.op_type
        elif ref.tag == 'dp_only':
            self.dp_only = ref.dp_only
        elif ref.tag == 'size':
            self.size = ref.size
        elif ref.tag == 'fmt':
            self.fmt = ref.fmt
        elif ref.tag == 'type':
            self.type = ref.type
        elif ref.tag == 'range':
            self.range = ref.range
        else:
            assert (False), 'error: unexpected TypeClause.tag ' + ref.tag
        self.tag = 'updated'

class TypeBlock(Statement):
    def __init__(self):
        self.keyword = 'type'
        self.name = ''
        self.clauses = []

class InstField(object):
    def __init__(self):
        self.tag = ''
        self.name = ''
        self.v_max = 0
        self.value = 0
        self.desc = ''
        self.type = ''
        self.enc = ''

    def update(self, ref):
        if ref.tag == 'id_range':
            self.name = ref.name
            self.v_max = ref.v_max
            self.value = ref.value
        elif ref.tag == 'id_number':
            self.name = ref.name
            self.value = ref.value
        elif ref.tag == 'desc':
            self.desc = ref.desc
        elif ref.tag == 'type':
            self.type = ref.type
        elif ref.tag == 'enc':
            self.enc = ref.enc
        else:
            assert (False), 'error: unexpected InstField.tag ' + ref.tag
        self.tag = 'updated'

class InstBlock(Statement):
    def __init__(self):
        self.keyword = 'inst'
        self.tag = ''
        self.name = ''
        self.desc = ''
        self.fields = []

    def update(self, ref):
        if ref.tag == 'name':
            self.name = ref.name
        elif ref.tag == 'desc':
            self.desc = ref.desc
        elif ref.tag == 'fields':
            self.fields = ref.fields
        else:
            assert (False), 'error: unexpected InstBlock.tag ' + ref.tag
        self.tag = 'updated'

FmtToDetails = {
    'NUM_B8'       : ( 'SregU8',   8   ),
    'NUM_I8'       : ( 'SregI8',   8   ),
    'NUM_B16'      : ( 'SregU16',  16  ),
    'NUM_F16'      : ( 'SregF16',  16  ),
    'NUM_B32'      : ( 'SregU32',  32  ),
    'NUM_F32'      : ( 'SregF32',  32  ),
    'NUM_I32'      : ( 'SregI32',  32  ),
    'NUM_U32'      : ( 'SregU32',  32  ),
    'NUM_B64'      : ( 'SregU64',  64  ),
    'NUM_I64'      : ( 'SregI64',  64  ),
    'NUM_F64'      : ( 'SregF64',  64  ),
    'NUM_U64'      : ( 'SregU64',  64  ),
    'BUF'          : ( 'SregU64',  64  ),
    'NUM_B96'      : ( 'SregU96',  96  ),
    'NUM_B128'     : ( 'SregU128', 128 ),
    'RSRC_SCRATCH' : ( 'SregU128', 128 ),
    'RSRC_SCALAR'  : ( 'SregU128', 128 ),
    'IMG'          : ( 'SregU256', 256 ),
}

# which get method is used to retrieve
# or set data
TypeToAccessMethod = {
    # scalar reg file get
    'SregU8'   : 'ScalarReg',
    'SregI8'   : 'ScalarReg',
    'SregU16'  : 'ScalarReg',
    'SregI16'  : 'ScalarReg',
    'SregU32'  : 'ScalarReg',
    'SregI32'  : 'ScalarReg',
    'SregU64'  : 'ScalarReg',
    'SregI64'  : 'ScalarReg',
    'SregU96'  : 'ScalarReg',
    'SregU128' : 'ScalarReg',
    'SregU256' : 'ScalarReg',
    'SregU512' : 'ScalarReg',
    'SregF16'  : 'ScalarReg',
    'SregF32'  : 'ScalarReg',
    'SregF64'  : 'ScalarReg',

    # vector reg file get
    'VregU8'   : 'VectorReg',
    'VregI8'   : 'VectorReg',
    'VregU16'  : 'VectorReg',
    'VregI16'  : 'VectorReg',
    'VregU32'  : 'VectorReg',
    'VregI32'  : 'VectorReg',
    'VregU64'  : 'VectorReg',
    'VregI64'  : 'VectorReg',
    'VregU96'  : 'VectorReg',
    'VregU128' : 'VectorReg',
    'VregU256' : 'VectorReg',
    'VregU512' : 'VectorReg',
    'VregF16'  : 'VectorReg',
    'VregF32'  : 'VectorReg',
    'VregF64'  : 'VectorReg',
}

SpecialCtx = {
    'SRC_NOLIT'  : 'SrcReg',
    'SRC'        : 'SrcReg',
    'SRC_NOLDS'  : 'SrcReg',
    'SRC_SIMPLE' : 'SrcReg',
    'EXEC'       : 'SpecialReg',
    'VCC'        : 'SpecialReg',
    'SCC'        : 'SpecialReg',
    'PC'         : 'SpecialReg',
    'PRIV'       : 'SpecialReg',
    'INST_ATC'   : 'SpecialReg',
    'M0'         : 'SpecialReg',
    'VSKIP'      : 'SpecialReg',
    'PI'         : 'SpecialReg',
    'NAN'        : 'SpecialReg',
    'INF'        : 'SpecialReg',
    'P0'         : 'SpecialReg',
    'P10'        : 'SpecialReg',
    'P20'        : 'SpecialReg',
    'TBA'        : 'SpecialReg',
}

# for DS fixup
SuffixToFmt = {
    '_I8' : 'NUM_I8',
    'I32' : 'NUM_I32',
    'I64' : 'NUM_I64'
}

EncRegInfoToField = {
    'DS:ADDR:VGPR:vgpr_a'          : 'ADDR',
    'MUBUF:ADDR:VGPR:vgpr_a'       : 'VADDR',
    'MIMG:ADDR:VGPR:vgpr_a'        : 'VADDR',
}

RegInfoToField = {
    'D:SDST:sdst'                  : 'SDST',
    'D:SREG:sdst'                  : 'SDST',
    'S0:SDST:ssrc'                 : 'SDST',
    'S:SSRC:ssrc'                  : 'SSRC0',
    'S0:SREG:ssrc'                 : 'SSRC0',
    'S0:SSRC:ssrc'                 : 'SSRC0',
    'S0:SSRC:ssrc_0'               : 'SSRC0',
    'S1:SSRC:ssrc_1'               : 'SSRC1',
    'SGPR:SREG:sgpr'               : '',
    'D:VGPR:vdst'                  : 'VDST',
    'D:VGPR:vgpr_dst'              : 'VDST',
    'S0:SRC:src'                   : 'SRC0',
    'S0:SRC:src_0'                 : 'SRC0',
    'S0:SRC_NOLDS:src_0'           : 'SRC0',
    'S0:SRC_NOLIT:src_0'           : 'SRC0',
    'S0:SRC_NOLIT:src'             : 'SRC0',
    'S0:SRC_VGPR:src'              : 'SRC0',
    'S:SRC:src'                    : 'SRC0',
    'S:SRC_NOLIT:src'              : 'SRC0',
    'S0:SRC_SIMPLE:src_0'          : 'SRC0',
    'S1:SRC_SIMPLE:src_1'          : 'SRC1',
    'S2:SRC_SIMPLE:src_2'          : 'SRC2',
    'S1:VGPR:src_1'                : 'VSRC1',
    'S:VGPR:vgpr_ij'               : 'VSRC',
    'S:SRC_VGPR:vgpr_ij'           : 'SRC0',
    'S0:SRC_VGPR:vgpr_ij'          : 'SRC0',
    'S2:SRC_VGPR:vgpr_add'         : 'SRC2',
    'S2:VGPR:src_2'                : 'VSRC1',
    'VGPR:VGPR:vgpr'               : '',
    'VGPR:SRC:vgpr'                : '',
    'VGPR:SRC_VGPR:vgpr'           : '',
    'VGPR:SRC_NOLIT:vgpr'          : '',
    'MEM:MEM:Mem()'                : '',
    'A:VGPR:vgpr_a'                : '',
    'B:VGPR:vgpr_a'                : '',
    'ADDR:SREG:sgpr_base'          : 'SBASE',
    'DATA:SMWR_OFFSET:offset'      : 'OFFSET',
    'DATA:SREG:sgpr_data'          : 'SDATA',
    'RETURN_DATA:SREG:sgpr_data'   : 'SDATA',
    'ADDR_BASE:VGPR:vgpr_a'        : 'ADDR',
    'DATA:VGPR:vgpr_d0'            : 'DATA0',
    'DATA_0:VGPR:vgpr_d0'          : 'DATA0',
    'DATA_1:VGPR:vgpr_d0'          : 'DATA0',
    'DATA_2:VGPR:vgpr_d0'          : 'DATA0',
    'RETURN_DATA:VGPR:vgpr_rtn'    : 'VDST',
    'RETURN_DATA_0:VGPR:vgpr_rtn'  : 'VDST',
    'RETURN_DATA_1:VGPR:vgpr_rtn'  : 'VDST',
    'DATA2:VGPR:vgpr_d1'           : 'DATA1',
    'DATA:SREG:sgpr_r'             : 'SRSRC',
    'DATA_0:SREG:sgpr_r'           : 'SRSRC',
    'DATA_1:SREG:sgpr_r'           : 'SRSRC',
    'DATA_2:SREG:sgpr_r'           : 'SRSRC',
    'DATA:SREG:sgpr_dst'           : 'SDATA',
    'DATA:VGPR:vgpr_d'             : 'VDATA',
    'DATA_0:VGPR:vgpr_d'           : 'VDATA',
    'DATA_1:VGPR:vgpr_d'           : 'VDATA',
    'DATA_2:VGPR:vgpr_d'           : 'VDATA',
    'RETURN_DATA:VGPR:vgpr_d'      : 'VDATA',
    'RETURN_DATA_0:VGPR:vgpr_d'    : 'VDATA',
    'RETURN_DATA_1:VGPR:vgpr_d'    : 'VDATA',
    'ADDR:VGPR:vgpr_addr'          : 'ADDR',
    'ADDR:VGPR:vgpr_src'           : 'ADDR',
    'DATA:VGPR:vgpr_src'           : 'DATA',
    'DATA_0:VGPR:vgpr_src'         : 'DATA',
    'DATA_1:VGPR:vgpr_src'         : 'DATA',
    'DATA_2:VGPR:vgpr_src'         : 'DATA',
    'DATA:VGPR:vgpr_dst'           : 'VDST',
    'DATA_0:VGPR:vgpr_dst'         : 'VDST',
    'DATA_1:VGPR:vgpr_dst'         : 'VDST',
    'DATA_2:VGPR:vgpr_dst'         : 'VDST',
    'RETURN_DATA:VGPR:vgpr_dst'    : 'VDST',
    'RETURN_DATA_0:VGPR:vgpr_dst'  : 'VDST',
    'RETURN_DATA_1:VGPR:vgpr_dst'  : 'VDST',
    'D:VCC:vcc'                    : '-VCC',
}

DataRegisters = ['DATA', 'DATA_0', 'DATA_1', 'DATA_2', 'RETURN_DATA',
                 'RETURN_DATA_0', 'RETURN_DATA_1']

SpecRegToDetails = {
    'SCC'       : ['SCC',      '',         'scc',       'SregU32', '', ''],
    'M0'        : ['M0',       '',         'm0',        'SregU32', '', ''],
    'PRIV'      : ['PRIV',     '',         'priv',      'SregU32', '', ''],
    'VSKIP'     : ['VSKIP',    '',         'vskip',     'SregU32', '', ''],
    'INST_ATC'  : ['INST_ATC', '',         'atc',       'SregU32', '', ''],
    'VCC'       : ['VCC',      '',         'vcc',       'SregU64', '', ''],
    'PC'        : ['PC',       '',         'pc',        'SregU64', '', ''],
    'TBA'       : ['TBA',      '',         'tba',       'SregU64', '', ''],
    'EXEC'      : ['EXEC',     '',         'exec',      'SregU64', '', ''],
    'PI'        : ['PI',       '',         'pi',        'SregF32', '', ''],
    'INF'       : ['INF',      '',         'inf',       'SregF32', '', ''],
    'NAN'       : ['NAN',      '',         'nan',       'SregF32', '', ''],
    'P0'        : ['P0',       '',         'p0',        'SregF32', '', ''],
    'P10'       : ['P10',      '',         'p10',       'SregF32', '', ''],
    'P20'       : ['P20',      '',         'p20',       'SregF32', '', ''],
    'offset0'   : ['', 'instData.OFFSET0', 'offset0',   'SregU16', '', ''],
    'OFFSET0'   : ['', 'instData.OFFSET0', 'offset0',   'SregU16', '', ''],
    'offset1'   : ['', 'instData.OFFSET1', 'offset1',   'SregU16', '', ''],
    'OFFSET1'   : ['', 'instData.OFFSET1', 'offset1',   'SregU16', '', ''],
    'SIMM16'    : ['', 'instData.SIMM16',  'simm16',    'SregI16', '', ''],
    'SIMM4'     : ['', 'instData.SSRC1',   'simm4',     'SregU16', '', ''],
    'threadID'  : ['',         '',         't',         'SregU32', '', ''],
    'cmp'       : ['',         '',         'cmp',       'SregU64', '', ''],
    'src'       : ['',         '',         'src',       'SregU64', '', ''],
    'tmp'       : ['',         '',         'tmp',       'SregU64', '', ''],
    'attr_word' : ['ATTR',     '',         'attr_word', 'SregU32', '', ''],
    'K'         : ['', 'extData.imm_u32',  'k',         'SregU32', '', '']
}

CoreMethodMap = {
    'getSRC_NOLDS_U32'  : 'getSRC_U32',
    'getSRC_NOLDS_I32'  : 'getSRC_I32',
    'getSRC_NOLDS_F16'  : 'getSRC_F16',
    'getSRC_NOLDS_U16'  : 'getSRC_U16',
    'getSRC_NOLIT_F16'  : 'getSRC_F16',
    'getSRC_SIMPLE_F16' : 'getSRC_F16',
    'getSRC_NOLIT_F32'  : 'getSRC_F32',
    'getSRC_SIMPLE_F32' : 'getSRC_F32',
    'getSRC_NOLIT_F64'  : 'getSRC_F64',
    'getSRC_SIMPLE_F64' : 'getSRC_F64',
    'getSRC_NOLIT_U16'  : 'getSRC_U16',
    'getSRC_SIMPLE_U16' : 'getSRC_U16',
    'getSRC_NOLIT_U32'  : 'getSRC_U32',
    'getSRC_SIMPLE_U32' : 'getSRC_U32',
    'getSRC_NOLIT_U64'  : 'getSRC_U64',
    'getSRC_SIMPLE_U64' : 'getSRC_U64',
    'getSRC_NOLIT_I32'  : 'getSRC_I32',
    'getSRC_SIMPLE_I32' : 'getSRC_I32',
    'getSRC_VGPR_F32'   : 'getSRC_F32',
    'getSRC_SIMPLE_I64' : 'getSRC_I64'
}

KnownExceptions = [
    'Inst_SOP2__S_CBRANCH_G_FORK',
    'Inst_SOPK__S_ADDK_I32',
    'Inst_SOPK__S_CBRANCH_I_FORK',
    'Inst_SOPK__S_GETREG_B32',
    'Inst_SOP1__S_CBRANCH_JOIN',
    'Inst_SOPP__S_CBRANCH_CDBGSYS',
    'Inst_SOPP__S_CBRANCH_CDBGUSER',
    'Inst_SOPP__S_CBRANCH_CDBGSYS_OR_USER',
    'Inst_SOPP__S_CBRANCH_CDBGSYS_AND_USER',
    'Inst_SOPP__S_SET_GPR_IDX_MODE',
    'Inst_VOP1__V_CVT_OFF_F32_I4',
    'Inst_VOP1__V_FFBH_U32',
    'Inst_VOP1__V_FFBL_B32',
    'Inst_VOP1__V_FFBH_I32',
    'Inst_VOPC__V_CMP_CLASS_F32',
    'Inst_VOPC__V_CMPX_CLASS_F32',
    'Inst_VOPC__V_CMP_CLASS_F64',
    'Inst_VOPC__V_CMPX_CLASS_F64',
    'Inst_VOPC__V_CMP_CLASS_F16',
    'Inst_VOPC__V_CMPX_CLASS_F16',
    'Inst_VINTRP__V_INTERP_MOV_F32',
    'Inst_VOP3__V_CMP_CLASS_F32',
    'Inst_VOP3__V_CMPX_CLASS_F32',
    'Inst_VOP3__V_CMP_CLASS_F64',
    'Inst_VOP3__V_CMPX_CLASS_F64',
    'Inst_VOP3__V_CMP_CLASS_F16',
    'Inst_VOP3__V_CMPX_CLASS_F16',
    'Inst_VOP3__V_CVT_OFF_F32_I4',
    'Inst_VOP3__V_FFBH_U32',
    'Inst_VOP3__V_FFBL_B32',
    'Inst_VOP3__V_FFBH_I32',
    'Inst_VOP3__V_CUBEID_F32',
    'Inst_VOP3__V_CUBESC_F32',
    'Inst_VOP3__V_CUBETC_F32',
    'Inst_VOP3__V_CUBEMA_F32',
    'Inst_VOP3__V_DIV_FIXUP_F32',
    'Inst_VOP3__V_DIV_FIXUP_F64',
    'Inst_VOP3__V_DIV_SCALE_F32',
    'Inst_VOP3__V_DIV_SCALE_F64',
    'Inst_VOP3__V_DIV_FMAS_F32',
    'Inst_VOP3__V_DIV_FMAS_F64',
    'Inst_VOP3__V_MSAD_U8',
    'Inst_VOP3__V_QSAD_PK_U16_U8',
    'Inst_VOP3__V_MQSAD_PK_U16_U8',
    'Inst_VOP3__V_MQSAD_U32_U8',
    'Inst_VOP3__V_PERM_B32',
    'Inst_VOP3__V_DIV_FIXUP_F16',
    'Inst_VOP3__V_CVT_PKACCUM_U8_F32',
    'Inst_VOP3__V_INTERP_MOV_F32',
    'Inst_VOP3__V_INTERP_P1LV_F16',
    'Inst_VOP3__V_MBCNT_LO_U32_B32',
    'Inst_VOP3__V_MBCNT_HI_U32_B32',
    'Inst_VOP3__V_TRIG_PREOP_F64',
    'Inst_VOP3__V_CVT_PKNORM_I16_F32',
    'Inst_VOP3__V_CVT_PKNORM_U16_F32',
    'Inst_VOP3__V_CVT_PKRTZ_F16_F32',
    'Inst_VOP3__V_CVT_PK_U16_U32',
    'Inst_VOP3__V_CVT_PK_I16_I32',
    'Inst_DS__DS_READ_U8',
    'Inst_DS__DS_READ_U16',
    'Inst_DS__DS_SWIZZLE_B32',
    'Inst_DS__DS_GWS_BARRIER',
    'Inst_DS__DS_CONSUME',
    'Inst_DS__DS_APPEND',
    'Inst_DS__DS_ORDERED_COUNT',
    'Inst_MIMG__IMAGE_GATHER4',
    'Inst_MIMG__IMAGE_GATHER4_CL',
    'Inst_MIMG__IMAGE_GATHER4_L',
    'Inst_MIMG__IMAGE_GATHER4_B',
    'Inst_MIMG__IMAGE_GATHER4_B_CL',
    'Inst_MIMG__IMAGE_GATHER4_LZ',
    'Inst_MIMG__IMAGE_GATHER4_C',
    'Inst_MIMG__IMAGE_GATHER4_C_CL',
    'Inst_MIMG__IMAGE_GATHER4_C_L',
    'Inst_MIMG__IMAGE_GATHER4_C_B',
    'Inst_MIMG__IMAGE_GATHER4_C_B_CL',
    'Inst_MIMG__IMAGE_GATHER4_C_LZ'
]

KnownEmpty = [
    'Inst_SOPK__S_SETREG_B32',
    'Inst_SOPK__S_SETREG_IMM32_B32',
    'Inst_SOPP__S_WAKEUP',
    'Inst_SOPP__S_BARRIER',
    'Inst_SOPP__S_SETKILL',
    'Inst_SOPP__S_WAITCNT',
    'Inst_SOPP__S_SETHALT',
    'Inst_SOPP__S_SLEEP',
    'Inst_SOPP__S_SETPRIO',
    'Inst_SOPP__S_SENDMSG',
    'Inst_SOPP__S_SENDMSGHALT',
    'Inst_SOPP__S_ICACHE_INV',
    'Inst_SOPP__S_INCPERFLEVEL',
    'Inst_SOPP__S_DECPERFLEVEL',
    'Inst_SOPP__S_TTRACEDATA',
    'Inst_SOPP__S_SET_GPR_IDX_OFF',
    'Inst_SMEM__S_DCACHE_INV',
    'Inst_SMEM__S_DCACHE_WB',
    'Inst_SMEM__S_DCACHE_INV_VOL',
    'Inst_SMEM__S_DCACHE_WB_VOL',
    'Inst_SMEM__S_MEMTIME',
    'Inst_SMEM__S_MEMREALTIME',
    'Inst_SMEM__S_ATC_PROBE',
    'Inst_SMEM__S_ATC_PROBE_BUFFER',
    'Inst_VOP1__V_READFIRSTLANE_B32',
    'Inst_VOP1__V_FREXP_EXP_I32_F64',
    'Inst_VOP1__V_FREXP_MANT_F64',
    'Inst_VOP1__V_FRACT_F64',
    'Inst_VOP1__V_CLREXCP',
    'Inst_VOP1__V_RCP_F16',
    'Inst_VOP1__V_SQRT_F16',
    'Inst_VOP1__V_RSQ_F16',
    'Inst_VOP1__V_LOG_F16',
    'Inst_VOP1__V_EXP_F16',
    'Inst_VOP1__V_FREXP_MANT_F16',
    'Inst_VOP1__V_FREXP_EXP_I16_F16',
    'Inst_VOP3__V_FREXP_EXP_I32_F64',
    'Inst_VOP3__V_FREXP_MANT_F64',
    'Inst_VOP3__V_FRACT_F64',
    'Inst_VOP3__V_CLREXCP',
    'Inst_VOP3__V_RCP_F16',
    'Inst_VOP3__V_SQRT_F16',
    'Inst_VOP3__V_RSQ_F16',
    'Inst_VOP3__V_LOG_F16',
    'Inst_VOP3__V_EXP_F16',
    'Inst_VOP3__V_FREXP_MANT_F16',
    'Inst_VOP3__V_FREXP_EXP_I16_F16',
    'Inst_VOP3__V_READLANE_B32',
    'Inst_VOP3__V_WRITELANE_B32',
    'Inst_DS__DS_WRXCHG2_RTN_B32',
    'Inst_DS__DS_WRXCHG2ST64_RTN_B32',
    'Inst_DS__DS_PERMUTE_B32',
    'Inst_DS__DS_BPERMUTE_B32',
    'Inst_DS__DS_WRXCHG2_RTN_B64',
    'Inst_DS__DS_WRXCHG2ST64_RTN_B64',
    'Inst_DS__DS_CONDXCHG32_RTN_B64',
    'Inst_DS__DS_GWS_SEMA_RELEASE_ALL',
    'Inst_DS__DS_GWS_INIT',
    'Inst_DS__DS_GWS_SEMA_V',
    'Inst_DS__DS_GWS_SEMA_BR',
    'Inst_DS__DS_GWS_SEMA_P',
    'Inst_DS__DS_READ_B96',
    'Inst_DS__DS_READ_B128',
    'Inst_MUBUF__BUFFER_LOAD_FORMAT_X',
    'Inst_MUBUF__BUFFER_LOAD_FORMAT_XY',
    'Inst_MUBUF__BUFFER_LOAD_FORMAT_XYZ',
    'Inst_MUBUF__BUFFER_LOAD_FORMAT_XYZW',
    'Inst_MUBUF__BUFFER_STORE_FORMAT_X',
    'Inst_MUBUF__BUFFER_STORE_FORMAT_XY',
    'Inst_MUBUF__BUFFER_STORE_FORMAT_XYZ',
    'Inst_MUBUF__BUFFER_STORE_FORMAT_XYZW',
    'Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_X',
    'Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XY',
    'Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZ',
    'Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZW',
    'Inst_MUBUF__BUFFER_STORE_FORMAT_D16_X',
    'Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XY',
    'Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XYZ',
    'Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XYZW',
    'Inst_MUBUF__BUFFER_LOAD_UBYTE',
    'Inst_MUBUF__BUFFER_LOAD_SBYTE',
    'Inst_MUBUF__BUFFER_LOAD_USHORT',
    'Inst_MUBUF__BUFFER_LOAD_SSHORT',
    'Inst_MUBUF__BUFFER_STORE_BYTE',
    'Inst_MUBUF__BUFFER_STORE_SHORT',
    'Inst_MUBUF__BUFFER_STORE_LDS_DWORD',
    'Inst_MUBUF__BUFFER_WBINVL1',
    'Inst_MUBUF__BUFFER_WBINVL1_VOL',
    'Inst_MTBUF__TBUFFER_LOAD_FORMAT_X',
    'Inst_MTBUF__TBUFFER_LOAD_FORMAT_XY',
    'Inst_MTBUF__TBUFFER_LOAD_FORMAT_XYZ',
    'Inst_MTBUF__TBUFFER_LOAD_FORMAT_XYZW',
    'Inst_MTBUF__TBUFFER_STORE_FORMAT_X',
    'Inst_MTBUF__TBUFFER_STORE_FORMAT_XY',
    'Inst_MTBUF__TBUFFER_STORE_FORMAT_XYZ',
    'Inst_MTBUF__TBUFFER_STORE_FORMAT_XYZW',
    'Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_X',
    'Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XY',
    'Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZ',
    'Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZW',
    'Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_X',
    'Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XY',
    'Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZ',
    'Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZW',
    'Inst_MIMG__IMAGE_LOAD',
    'Inst_MIMG__IMAGE_LOAD_MIP',
    'Inst_MIMG__IMAGE_LOAD_PCK',
    'Inst_MIMG__IMAGE_LOAD_PCK_SGN',
    'Inst_MIMG__IMAGE_LOAD_MIP_PCK',
    'Inst_MIMG__IMAGE_LOAD_MIP_PCK_SGN',
    'Inst_MIMG__IMAGE_STORE',
    'Inst_MIMG__IMAGE_STORE_MIP',
    'Inst_MIMG__IMAGE_STORE_PCK',
    'Inst_MIMG__IMAGE_STORE_MIP_PCK',
    'Inst_MIMG__IMAGE_GET_RESINFO',
    'Inst_MIMG__IMAGE_SAMPLE',
    'Inst_MIMG__IMAGE_SAMPLE_CL',
    'Inst_MIMG__IMAGE_SAMPLE_D',
    'Inst_MIMG__IMAGE_SAMPLE_D_CL',
    'Inst_MIMG__IMAGE_SAMPLE_L',
    'Inst_MIMG__IMAGE_SAMPLE_B',
    'Inst_MIMG__IMAGE_SAMPLE_B_CL',
    'Inst_MIMG__IMAGE_SAMPLE_LZ',
    'Inst_MIMG__IMAGE_SAMPLE_C',
    'Inst_MIMG__IMAGE_SAMPLE_C_CL',
    'Inst_MIMG__IMAGE_SAMPLE_C_D',
    'Inst_MIMG__IMAGE_SAMPLE_C_D_CL',
    'Inst_MIMG__IMAGE_SAMPLE_C_L',
    'Inst_MIMG__IMAGE_SAMPLE_C_B',
    'Inst_MIMG__IMAGE_SAMPLE_C_B_CL',
    'Inst_MIMG__IMAGE_SAMPLE_C_LZ',
    'Inst_MIMG__IMAGE_SAMPLE_O',
    'Inst_MIMG__IMAGE_SAMPLE_CL_O',
    'Inst_MIMG__IMAGE_SAMPLE_D_O',
    'Inst_MIMG__IMAGE_SAMPLE_D_CL_O',
    'Inst_MIMG__IMAGE_SAMPLE_L_O',
    'Inst_MIMG__IMAGE_SAMPLE_B_O',
    'Inst_MIMG__IMAGE_SAMPLE_B_CL_O',
    'Inst_MIMG__IMAGE_SAMPLE_LZ_O',
    'Inst_MIMG__IMAGE_SAMPLE_C_O',
    'Inst_MIMG__IMAGE_SAMPLE_C_CL_O',
    'Inst_MIMG__IMAGE_SAMPLE_C_D_O',
    'Inst_MIMG__IMAGE_SAMPLE_C_D_CL_O',
    'Inst_MIMG__IMAGE_SAMPLE_C_L_O',
    'Inst_MIMG__IMAGE_SAMPLE_C_B_O',
    'Inst_MIMG__IMAGE_SAMPLE_C_B_CL_O',
    'Inst_MIMG__IMAGE_SAMPLE_C_LZ_O',
    'Inst_MIMG__IMAGE_GATHER4_O',
    'Inst_MIMG__IMAGE_GATHER4_CL_O',
    'Inst_MIMG__IMAGE_GATHER4_L_O',
    'Inst_MIMG__IMAGE_GATHER4_B_O',
    'Inst_MIMG__IMAGE_GATHER4_B_CL_O',
    'Inst_MIMG__IMAGE_GATHER4_LZ_O',
    'Inst_MIMG__IMAGE_GATHER4_C_O',
    'Inst_MIMG__IMAGE_GATHER4_C_CL_O',
    'Inst_MIMG__IMAGE_GATHER4_C_L_O',
    'Inst_MIMG__IMAGE_GATHER4_C_B_O',
    'Inst_MIMG__IMAGE_GATHER4_C_B_CL_O',
    'Inst_MIMG__IMAGE_GATHER4_C_LZ_O',
    'Inst_MIMG__IMAGE_GET_LOD',
    'Inst_MIMG__IMAGE_SAMPLE_CD',
    'Inst_MIMG__IMAGE_SAMPLE_CD_CL',
    'Inst_MIMG__IMAGE_SAMPLE_C_CD',
    'Inst_MIMG__IMAGE_SAMPLE_C_CD_CL',
    'Inst_MIMG__IMAGE_SAMPLE_CD_O',
    'Inst_MIMG__IMAGE_SAMPLE_CD_CL_O',
    'Inst_MIMG__IMAGE_SAMPLE_C_CD_O',
    'Inst_MIMG__IMAGE_SAMPLE_C_CD_CL_O',
    'Inst_EXP__EXP',
    'Inst_FLAT__FLAT_LOAD_UBYTE',
    'Inst_FLAT__FLAT_LOAD_SBYTE',
    'Inst_FLAT__FLAT_LOAD_USHORT',
    'Inst_FLAT__FLAT_LOAD_SSHORT',
    'Inst_FLAT__FLAT_STORE_BYTE',
    'Inst_FLAT__FLAT_STORE_SHORT',
]
