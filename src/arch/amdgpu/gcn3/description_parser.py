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

import sys

from description_objects import *
from m5.util.grammar import Grammar
from pprint import pprint, pformat

class ParseError(Exception):
    pass

class DescriptionParser(Grammar):
    def __init__(self):
        super(DescriptionParser, self).__init__()
        self.in_text = False
        self.single_subs = [
            [ '1 if S0 is chosen as the minimum value.',
              '(S0 < S1) ? 1 : 0;' ],
            [ '1 if S0 is chosen as the maximum value.',
              '(S0 > S1) ? 1 : 0;' ],
            [ '1 if result is non-zero.',
              '(D != 0) ? 1 : 0;' ],
            [ '(SCC) then D.u = S0.u;',
              'if(SCC) then D.u = S0.u;' ],
            [ 'SCC = 1 if the new value of EXEC is non-zero.',
              'SCC = (EXEC != 0) ? 1 : 0;' ],
            [ 'D.u = (VCC[i] ? S1.u : S0.u) (i = threadID in wave); '
              'VOP3: specify VCC as a scalar GPR in S2.',
              'D.u = (VCC[threadID] ? S1.u : S0.u);' ],
            [ 'D.u = QuadMask(S0.u):',
              'D.u = QuadMask(S0.u);' ],
            [ 'D.u64 = QuadMask(S0.u64):',
              'D.u64 = QuadMask(S0.u64);' ],
            [ 'A = ADDR_BASE;',
              '' ],
            [ 'B = A + 4*(offset1[7] ? {A[31],A[31:17]} : '
              '{offset1[6],offset1[6:0],offset0});',
              '' ],
            [ 'MEM_ADDR',
              'MEM' ],
            [ 'Untyped buffer load dword.',
              'DATA.u32 = MEM[ADDR];' ],
            [ 'Untyped buffer load 2 dwords.',
              'DATA.u64 = MEM[ADDR];' ],
            [ 'Untyped buffer load 3 dwords.',
              'DATA.u96 = MEM[ADDR];' ],
            [ 'Untyped buffer load 4 dwords.',
              'DATA.u128 = MEM[ADDR];' ],
            [ 'Untyped buffer store dword.',
              'MEM[ADDR] = DATA.u32;' ],
            [ 'Untyped buffer store 2 dwords.',
              'MEM[ADDR] = DATA.u64;' ],
            [ 'Untyped buffer store 3 dwords.',
              'MEM[ADDR] = DATA.u96;' ],
            [ 'Untyped buffer store 4 dwords.',
              'MEM[ADDR] = DATA.u128;' ],
            [ 'Read 1 dword from scalar data cache.',
              'DATA.u32 = MEM[ADDR];' ],
            [ 'Read 2 dwords from scalar data cache.',
              'DATA.u64 = MEM[ADDR];' ],
            [ 'Read 4 dwords from scalar data cache.',
              'DATA.u128 = MEM[ADDR];' ],
            [ 'Read 8 dwords from scalar data cache.',
              'DATA.u256 = MEM[ADDR];' ],
            [ 'Read 16 dwords from scalar data cache.',
              'DATA.u512 = MEM[ADDR];' ],
            [ 'Write 1 dword to scalar data cache.',
              'MEM[ADDR] = DATA.u32;' ],
            [ 'Write 2 dwords to scalar data cache.',
              'MEM[ADDR] = DATA.u64;' ],
            [ 'Write 4 dwords to scalar data cache.',
              'MEM[ADDR] = DATA.u128;' ],
            [ 'D[0] = OR(S0[3:0]),',
              '# D[0] = OR(S0[3:0]),' ],
            [ '(unsigned 16-bit integer domain)',
              'in the unsigned 16-bit integer domain' ],
            [ ' (unsigned compare);',
              '; # (unsigned compare)' ],
            [ '2s_complement',
              'TwosComplement' ],
            [ 'floor(S0.f16) is even',
              'floor_is_even(S0.f16)' ],
            [ 'D.u[31:0] = S0.u[0:31],',
              'D.u[31:0] = S0.u[0:31];' ],
            [ 'D.d = trunc(S0.d), return',
              'D.d = trunc(S0.d); return' ],
            [ 'D.f = trunc(S0.f), return',
              'D.f = trunc(S0.f); return' ],
             [ 'D[i] = (S0[(i & ~3):(i | 3)] != 0)',
              'D = whole_quad_mode(S0)' ],
            [ 'M0[15:12] = SIMM4.',
              'M0[15:12] = SIMM16.' ],
            [ 'D = VCC in VOPC encoding.',
              '# D = VCC in VOPC encoding.' ],
            [ 'S0.u64 is a',
              '# S0.u64 is a' ],
            [ 'D.f = S0.f * K + S1.f; K is a',
              'D.f = S0.f * K + S2.f; # K is a' ],
            [ 'D.f16 = S0.f16 * K.f16 + S1.f16; K is a',
              'D.f16 = S0.f16 * K.f16 + S2.f16; # K is a' ],
            [ 'K is a',
              '# K is a' ],
            [ '(2 ** S1.i16)',
              'pow(2.0, S1.i16)' ],
            [ 'original',
              '# original' ],
            [ 'attr_word selects',
              '# attr_word selects' ],
            [ '; S0 is ',
              '; # S0 is ' ],
            [ '. S0 is ',
              '. # S0 is ' ],
            [ 'SIMM16 = ',
              '# SIMM16 = ' ],
            [ 'SIMM16[3:0] = ',
              '# SIMM16[3:0] = ' ],
            [ 'SIMM16[6:4] = ',
              '# SIMM16[6:4] = ' ],
            [ 'SIMM16[12:8] = ',
              '# SIMM16[12:8] = ' ],
            [ 'SIMM16[9:0] contains',
              '# SIMM16[9:0] contains' ],
            [ 'Exponent',
              'exponent' ],
            [ 'Mantissa',
              'mantissa' ],
            [ 'Corrupted',
              'corrupted' ],
            [ 'threadId',
              'threadID' ],
            [ '16\'h0',
              'zeros(16)' ],
            [ '24\'h0',
              'zeros(24)' ],
            [ 'DATA.',
              'DATA;' ],
            [ 'DATA2.',
              'DATA2;' ],
            [ 'tmp.',
              'tmp;' ],
            [ 'vcc_out',
              'VCC[threadID]' ],
            [ '].\n',
              '];\n' ],
            [ '\\t tD',
              '\\t D' ]
        ]
        self.repeat_subs = [
            ['MEM[A]', 'MEM[ADDR]'],
            ['MEM[B]', 'MEM[ADDR, offset1, offset0]'],
            ['DATA[0]', 'DATA_0'],
            ['DATA[1]', 'DATA_1'],
            ['DATA[0:1]', 'DATA_0'],
            ['DATA[2:3]', 'DATA_2'],
            ['0x800000000ULL', '0x100000000ULL'],
            ['<>', '!=']
        ]

    # lexer
    states = (
        ('comment', 'exclusive'),
        ('notes', 'exclusive'),
    )

    reserved = (
        'A',
        'ABS',
        'ADDR',
        'ADDR_BASE',
        'ATTR_WORD',
        'APPROXIMATE2TOX',
        'APPROXIMATELOG2',
        'APPROXIMATERECIP',
        'APPROXIMATERECIPSQRT',
        'APPROXIMATESQRT',
        'B',
        'CMP',
        'COUNTONEBITS',
        'COUNTZEROBITS',
        'CORRUPTED',
        'COS',
        'D',
        'DATA',
        'DATA2',
        'DATA_0',
        'DATA_1',
        'DATA_2',
        'DOUBLE',
        'ELSE',
        'ELSIF',
        'END',
        'EXEC',
        'EXPONENT',
        'F',
        'F16',
        'F32',
        'FINDFIRSTONE',
        'FINDFIRSTZERO',
        'FIRSTOPPOSITESIGNBIT',
        'FLOAT',
        'FLOOR',
        'FLOOR_IS_EVEN',
        'FLT16_TO_FLT32',
        'FLT16_TO_INT16',
        'FLT16_TO_UINT16',
        'FLT32_TO_FLT16',
        'FLT32_TO_FLT64',
        'FLT32_TO_INT32',
        'FLT32_TO_UINT32',
        'FLT32_TO_UINT8',
        'FLT64_TO_INT32',
        'FLT64_TO_FLT32',
        'FLT64_TO_UINT32',
        'FRACT',
        'I',
        'I16',
        'I32',
        'INF',
        'INT',
        'INT16_TO_FLT16',
        'INT32_FLOOR',
        'INT32_TO_FLT32',
        'INT32_TO_FLT64',
        'I64',
        'IF',
        'INST_ATC',
        'ISNAN',
        'K',
        'LOG2',
        'MANTISSA',
        'MAX',
        'MEDIAN',
        'MEM',
        'MIN',
        'M0',
        'NAN',
        'NOP',
        'OFFSET0',
        'OFFSET1',
        'OPCODE_SIZE_IN_BITS',
        'PC',
        'PI',
        'POW',
        'POWER2',
        'PRIV',
        'P0',
        'P10',
        'P20',
        'QUADMASK',
        'RETURN_DATA',
        'RETURN_DATA_0',
        'RETURN_DATA_1',
        'ROUND_NEAREST_EVEN',
        'QUOTE',
        'S',
        'S0',
        'S1',
        'S2',
        'SAD_U8',
        'SCC',
        'SGPR',
        'SIGNEXT',
        'SIN',
        'SIMM16',
        'SIMM4',
        'SNORM',
        'SQRT',
        'SRC',
        'TBA',
        'THEN',
        'THREADID',
        'TMP',
        'TRUNC',
        'TWOSCOMPLEMENT',
        'U',
        'U16',
        'U32',
        'U64',
        'U96',
        'U128',
        'U256',
        'U512',
        'UINT16_TO_FLT16',
        'UINT32_TO_FLT32',
        'UINT32_TO_FLT64',
        'UINT8_TO_FLT32',
        'UNORM',
        'UNSIGNED',
        'VCC',
        'VGPR',
        'VSKIP',
        'WHOLE_QUAD_MODE',
        'ZEROS'
    )

    tokens = reserved + (
        'ADD',
        'ADDEQ',
        'ADDADD',
        'AND',
        'ANDAND',
        'ANDEQ',
        'COLON',
        'COMMA',
        'COMMENT',
        'DIVEQ',
        'DIV',
        'DO',
        'DOT',
        'DOTDOTDOT',
        'EQ',
        'EQUALS',
        'FOR',
        'GE',
        'GT',
        'IN',
        'INV',
        'LBRACE',
        'LBRACKET',
        'LE',
        'LG',
        'LPAREN',
        'LSH',
        'LSHEQ',
        'LT',
        'MODEQ',
        'MOD',
        'MUL',
        'MULEQ',
        'MULMUL',
        'NE',
        'NEWLINE',
        'NOT',
        'NUMBER',
        'OR',
        'OREQ',
        'OROR',
        'RBRACE',
        'RBRACKET',
        'RPAREN',
        'RSH',
        'RSHEQ',
        'QUESTION',
        'SEMI',
        'SUB',
        'SUBEQ',
        'SUBSUB',
        'TAB',
        'XOR',
        'XOREQ',
        'ID',
    )

    t_ADD       = r'\+'
    t_ADDADD    = r'\+\+'
    t_ADDEQ     = r'\+='
    t_AND       = r'&'
    t_ANDAND    = r'&&'
    t_ANDEQ     = r'&='
    t_COLON     = r':'
    t_COMMA     = r','
    t_DIV       = r'/'
    t_DIVEQ     = r'/='
    t_DOT       = r'\.'
    t_DOTDOTDOT = r'\.\.\.'
    t_EQ        = r'=='
    t_EQUALS    = r'='
    t_GE        = r'>='
    t_GT        = r'>'
    t_INV       = r'~'
    t_LBRACE    = r'\{'
    t_LBRACKET  = r'\['
    t_LE        = r'<='
    t_LPAREN    = r'\('
    t_LSH       = r'<<'
    t_LSHEQ     = r'<<='
    t_LT        = r'<'
    t_LG        = r'<>'
    t_MOD       = r'%'
    t_MODEQ     = r'%='
    t_MUL       = r'\*'
    t_MULMUL    = r'\*\*'
    t_NE        = r'!='
    t_NOT       = r'!'
    t_OR        = r'\|'
    t_OREQ      = r'\|='
    t_OROR      = r'\|\|'
    t_RBRACE    = r'\}'
    t_RBRACKET  = r'\]'
    t_RPAREN    = r'\)'
    t_RSH       = r'>>'
    t_RSHEQ     = r'>>='
    t_QUESTION  = r'\?'
    t_QUOTE     = r'\''
    t_SEMI      = r';'
    t_SUB       = r'-'
    t_SUBEQ     = r'-='
    t_SUBSUB    = r'--'
    t_TAB       = r'\\t'
    t_XOR       = r'\^'
    t_XOREQ     = r'\^='

    reserved_map = { }
    for r in reserved:
        reserved_map[r.lower()] = r

    def t_HASH(self, t):
        r'\#'
        t.lexer.begin('comment')
        text = ''
        while True:
            tok = t.lexer.token()
            if not tok:
                break;
            text += tok.value
            if tok.type == 'NEWLINE':
                break;
        t.lexer.begin('INITIAL')
        t.type = 'COMMENT'
        t.value = text
        return t

    def t_DIVDIV(self, t):
        r'//'
        t.lexer.begin('comment')
        text = ''
        while True:
            tok = t.lexer.token()
            if not tok:
                break;
            text += tok.value
            if tok.type == 'NEWLINE':
                break;
        t.lexer.begin('INITIAL')
        t.type = 'COMMENT'
        t.value = text
        return t

    def t_CODE(self, t):
        r'@code'
        t.lexer.begin('INITIAL')
        pass

    def t_TEXT(self, t):
        r'@text'
        t.lexer.begin('comment')
        text = ''
        while True:
            tok = t.lexer.token()
            if not tok:
                break;
            text += tok.value
        t.lexer.begin('INITIAL')
        t.type = 'COMMENT'
        t.value = text
        return t

    def t_ID(self, t):
        r'[a-zA-Z_][a-zA-Z0-9_]*'
        if t.value[0].isupper() and t.value[1:].islower():
            t.lexer.begin('comment')
        else:
            t.type = self.reserved_map.get(t.value.lower(), 'ID')
            if t.type == 'ID':
                t.lexer.begin('notes')
        return t

    # comment state lexer tokens
    # --- consume everything up to the next NEWLINE
    def t_comment_COMMENT(self, t):
        r'[^\n]+'
        return t

    def t_comment_NEWLINE(self, t):
        r'\n+'
        t.lexer.lineno += t.value.count('\n')
        t.lexer.begin('INITIAL')
        return t

    t_comment_ignore = ' \t\x0c'

    def t_comment_error(self, t):
        sys.exit('%d: illegal character "%s"' % (t.lexer.lineno, t.value[0]))

    # notes state lexer tokens
    # -- consume everything but '(' and ')' up to the next NEWLINE
    def t_notes_COMMENT(self, t):
        r'[^\(\)\n]+'
        return t

    def t_notes_LPAREN(self, t):
        r'\('
        t.lexer.begin('INITIAL')
        return t

    def t_notes_RPAREN(self, t):
        r'\)'
        t.lexer.begin('INITIAL')
        return t

    def t_notes_NEWLINE(self, t):
        r'\n+'
        t.lexer.lineno += t.value.count('\n')
        t.lexer.begin('INITIAL')
        return t

    t_notes_ignore = ' \t\x0c'

    def t_notes_error(self, t):
        sys.exit('%d: illegal character "%s"' % (t.lexer.lineno, t.value[0]))

    def t_NEWLINE(self, t):
        r'\n+'
        t.lexer.lineno += t.value.count('\n')
        return t

    def t_HEXADECIMAL(self, t):
        r'0[xX][0-9a-fA-F]+(ULL)?'
        t.value = int(t.value.replace('ULL', ''), 16)
        t.type = 'NUMBER'
        return t

    def t_FLOAT(self, t):
        r'[0-9][0-9]*\.[0-9][0-9]*(f)?'
        t.value = float(t.value.replace('f', ''))
        t.type = 'NUMBER'
        return t

    def t_DECIMAL(self, t):
        r'[0-9][0-9]*(ULL)?'
        t.value = int(t.value.replace('ULL', ''), 10)
        t.type = 'NUMBER'
        return t

    t_ignore = ' \t\x0c'

    def t_error(self, t):
        sys.exit('%d: illegal character "%s"' % (t.lexer.lineno, t.value[0]))


    ## parser
    start = 'specification'

    precedence = (
        ('left', 'OROR', 'ANDAND'),
        ('left', 'OR', 'XOR', 'AND'),
        ('left', 'EQ', 'NE'),
        ('left', 'GE', 'GT', 'LE', 'LG', 'LT'),
        ('left', 'LSH', 'RSH'),
        ('left', 'ADD', 'SUB'),
        ('left', 'DIV', 'MUL'),
        ('left', 'MOD', 'MULMUL')
    )

    # empty rule
    def p_empty_0(self, t):
        '''
        empty :
        '''
        pass

    # comment rule
    def p_comment_0(self, t):
        '''
        comment : COMMENT
                | ID
                | QUOTE
        '''
        clause = CommentClause()
        clause.content = [t[1]]
        t[0] = clause

    def p_comment_1(self, t):
        '''
        comment : LPAREN comment RPAREN
        '''
        clause = CommentClause()
        clause.content = [t[1], t[2], t[3]]
        t[0] = clause

    def p_comment_2(self, t):
        '''
        comment : LPAREN comment RPAREN COMMA
        '''
        clause = CommentClause()
        clause.content = [t[1], t[2], t[3], t[4]]
        t[0] = clause

    def p_comment_3(self, t):
        '''
        comment : comment COMMENT
        '''
        t[1].content.append(t[2])
        t[0] = t[1]

    # data_reg rules
    def p_data_reg_0(self, t):
        '''
        data_reg : D
                 | S
                 | S0
                 | S1
                 | S2
        '''
        t[0] = t[1]

    # spec_reg rules
    def p_spec_reg_0(self, t):
        '''
        spec_reg : A
                 | ADDR
                 | ADDR_BASE
                 | ATTR_WORD
                 | B
                 | CMP
                 | DATA
                 | DATA2
                 | DATA_0
                 | DATA_1
                 | DATA_2
                 | EXEC
                 | I
                 | INF
                 | INST_ATC
                 | K
                 | M0
                 | NAN
                 | OFFSET0
                 | OFFSET1
                 | OPCODE_SIZE_IN_BITS
                 | P0
                 | P10
                 | P20
                 | PC
                 | PI
                 | PRIV
                 | RETURN_DATA
                 | RETURN_DATA_0
                 | RETURN_DATA_1
                 | SCC
                 | SIMM16
                 | SIMM4
                 | SRC
                 | TBA
                 | THREADID
                 | TMP
                 | VCC
                 | VSKIP
        '''
        t[0] = t[1]

    # function rules
    def p_function_0(self, t):
        '''
        function : ABS
                 | APPROXIMATE2TOX
                 | APPROXIMATELOG2
                 | APPROXIMATERECIP
                 | APPROXIMATERECIPSQRT
                 | APPROXIMATESQRT
                 | COUNTONEBITS
                 | COUNTZEROBITS
                 | CORRUPTED
                 | COS
                 | EXPONENT
                 | FINDFIRSTONE
                 | FINDFIRSTZERO
                 | FIRSTOPPOSITESIGNBIT
                 | FLOOR
                 | FLOOR_IS_EVEN
                 | FLT16_TO_FLT32
                 | FLT16_TO_INT16
                 | FLT16_TO_UINT16
                 | FLT32_TO_FLT16
                 | FLT32_TO_FLT64
                 | FLT32_TO_INT32
                 | FLT32_TO_UINT32
                 | FLT32_TO_UINT8
                 | FLT64_TO_FLT32
                 | FLT64_TO_INT32
                 | FLT64_TO_UINT32
                 | FRACT
                 | INT16_TO_FLT16
                 | INT32_FLOOR
                 | INT32_TO_FLT32
                 | INT32_TO_FLT64
                 | ISNAN
                 | LOG2
                 | MANTISSA
                 | MAX
                 | MEDIAN
                 | MIN
                 | POWER2
                 | QUADMASK
                 | POW
                 | ROUND_NEAREST_EVEN
                 | SAD_U8
                 | SIGNEXT
                 | SIN
                 | SQRT
                 | TRUNC
                 | TWOSCOMPLEMENT
                 | UINT16_TO_FLT16
                 | UINT32_TO_FLT32
                 | UINT32_TO_FLT64
                 | UINT8_TO_FLT32
                 | WHOLE_QUAD_MODE
                 | ZEROS
        '''
        t[0] = t[1]

    # data_type rules
    def p_data_type_0(self, t):
        '''
        data_type : empty
        '''
        t[0] = ['empty', -1]

    def p_data_type_1(self, t):
        '''
        data_type : DOT I16
                  | DOT F16
                  | DOT U16
        '''
        t[0] = [t[2], 16]

    def p_data_type_2(self, t):
        '''
        data_type : DOT I
                  | DOT F
                  | DOT U
                  | DOT I32
                  | DOT F32
                  | DOT U32
        '''
        t[0] = [t[2], 32]

    def p_data_type_3(self, t):
        '''
        data_type : DOT D
                  | DOT I64
                  | DOT U64
        '''
        t[0] = [t[2], 64]

    def p_data_type_4(self, t):
        '''
        data_type : DOT U96
        '''
        t[0] = [t[2], 96]

    def p_data_type_5(self, t):
        '''
        data_type : DOT U128
        '''
        t[0] = [t[2], 128]

    def p_data_type_6(self, t):
        '''
        data_type : DOT U256
        '''
        t[0] = [t[2], 256]

    def p_data_type_7(self, t):
        '''
        data_type : DOT U512
        '''
        t[0] = [t[2], 512]

    # data_range rules
    def p_data_range_0(self, t):
        '''
        data_range : empty
        '''
        t[0] = None

    def p_data_range_1(self, t):
        '''
        data_range : LBRACKET expression RBRACKET
        '''
        t[0] = t[2]

    def p_data_range_2(self, t):
        '''
        data_range : LBRACKET expression COLON expression RBRACKET
        '''
        t[0] = [t[2], t[4]]

    # reg_desc rules
    def p_reg_desc_0(self, t):
        '''
        reg_desc : data_reg data_type data_range
        '''
        clause = DataRegClause()
        clause.reg = t[1]
        clause.typ = t[2]
        clause.rng = t[3]
        t[0] = clause

    def p_reg_desc_1(self, t):
        '''
        reg_desc : spec_reg data_range data_type data_range
        '''
        clause = DataRegClause()
        clause.reg = t[1]
        clause.typ = t[3]
        if t[1] == 'SGPR' or t[1] == 'VGPR' or t[1] == 'MEM':
            clause.idx = t[2]
            clause.rng = t[4]
        elif t[4]:
            clause.idx = t[2]
            clause.rng = t[4]
        else:
            clause.idx = None
            clause.rng = t[2]
        t[0] = clause

    def p_reg_desc_2(self, t):
        '''
        reg_desc : LBRACE reg_desc COMMA reg_desc RBRACE
        '''
        clause = GroupClause()
        clause.group = [t[2], t[4]]
        t[0] = clause

    def p_reg_desc_3(self, t):
        '''
        reg_desc : LBRACE reg_desc COMMA reg_desc COMMA reg_desc RBRACE
        '''
        clause = GroupClause()
        clause.group = [t[2], t[4], t[6]]
        t[0] = clause

    def p_reg_desc_4(self, t):
        '''
        reg_desc : LBRACE reg_desc COMMA reg_desc COMMA reg_desc \
                   COMMA reg_desc RBRACE
        '''
        clause = GroupClause()
        clause.group = [t[2], t[4], t[6], t[8]]
        t[0] = clause

    def p_mem_desc_0(self, t):
        '''
        mem_desc : MEM LBRACKET clist RBRACKET data_range
        '''
        clause = MemClause()
        clause.mem = t[1]
        clause.addr = t[3]
        clause.rng = t[5]
        t[0] = clause

    def p_mem_desc_1(self, t):
        '''
        mem_desc : LBRACE mem_desc COMMA mem_desc RBRACE
        '''
        clause = GroupClause()
        clause.group = [t[2], t[4]]
        t[0] = clause

    def p_mem_desc_2(self, t):
        '''
        mem_desc : LBRACE mem_desc COMMA mem_desc COMMA mem_desc RBRACE
        '''
        clause = GroupClause()
        clause.group = [t[2], t[4], t[6]]
        t[0] = clause

    def p_mem_desc_3(self, t):
        '''
        mem_desc : LBRACE mem_desc COMMA mem_desc COMMA mem_desc \
                   COMMA mem_desc RBRACE
        '''
        clause = GroupClause()
        clause.group = [t[2], t[4], t[6], t[8]]
        t[0] = clause

    def p_gpr_desc_0(self, t):
        '''
        gpr_desc : SGPR LBRACKET expression RBRACKET data_type
                 | VGPR LBRACKET expression RBRACKET data_type
        '''
        clause = GprClause()
        clause.gpr = t[1]
        clause.idx = t[3]
        clause.typ = t[5]
        t[0] = clause

    # clist rules
    def p_clist_0(self, t):
        '''
        clist : empty
        '''
        pass

    def p_clist_1(self, t):
        '''
        clist : expression
        '''
        t[0] = t[1]

    def p_clist_2(self, t):
        '''
        clist : clist COMMA expression
        '''
        clause = CommaClause()
        clause.left = t[1]
        clause.right = t[3]
        t[0] = clause

    # operand rules
    def p_operand_0(self, t):
        '''
        operand : NUMBER
        '''
        clause = ConstantClause()
        clause.value = t[1]
        t[0] = clause

    def p_operand_1(self, t):
        '''
        operand : reg_desc
                | mem_desc
                | gpr_desc
        '''
        t[0] = t[1]

    def p_operand_2(self, t):
        '''
        operand : function LPAREN clist RPAREN
        '''
        clause = FunctionClause()
        clause.func = t[1]
        clause.args = t[3]
        t[0] = clause


    # unary rules
    def p_unary_0(self, t):
        '''
        unary : operand
        '''
        t[0] = t[1]

    def p_unary_1(self, t):
        '''
        unary : ADD expression
              | INV expression
              | NOT expression
              | SUB expression
              | ADDADD expression
              | SUBSUB expression
        '''
        clause = UnaryClause()
        clause.op = t[1]
        clause.oprnd = t[2]
        t[0] = clause

    def p_unary_2(self, t):
        '''
        unary : LPAREN expression RPAREN
        '''
        clause = ParenClause()
        clause.parexp = t[2]
        t[0] = clause

    def p_unary_3(self, t):
        '''
        unary : NOP
        '''
        clause = FunctionClause()
        clause.func = t[1]
        clause.arg = None
        t[0] = clause

    def p_type_name_0(self, t):
        '''
        type_name : DOUBLE
                  | FLOAT
                  | INT
                  | UNSIGNED
                  | SNORM
                  | UNORM
        '''
        t[0] = t[1]

    def p_cast_0(self, t):
        '''
        cast : unary
        '''
        t[0] = t[1]

    def p_cast_1(self, t):
        '''
        cast : LPAREN type_name RPAREN cast
        '''
        clause = CastClause()
        clause.typ = t[2]
        clause.var = t[4]
        t[0] = clause

    def p_binary_0(self, t):
        '''
        binary : cast
        '''
        t[0] = t[1]

    def p_binary_1(self, t):
        '''
        binary : binary MUL binary
               | binary DIV binary
               | binary MULMUL binary
               | binary MOD binary
               | binary ADD binary
               | binary SUB binary
               | binary LSH binary
               | binary RSH binary
               | binary LE binary
               | binary LG binary
               | binary LT binary
               | binary GE binary
               | binary GT binary
               | binary EQ binary
               | binary NE binary
               | binary OR binary
               | binary XOR binary
               | binary AND binary
               | binary ANDAND binary
               | binary OROR binary
        '''
        clause = BinaryClause()
        clause.left = t[1]
        clause.op = t[2]
        clause.right = t[3]
        t[0] = clause

    # conditional rules
    def p_conditional_0(self, t):
        '''
        conditional : binary
        '''
        t[0] = t[1]

    def p_conditional_1(self, t):
        '''
        conditional : binary QUESTION expression COLON conditional
        '''
        clause = ConditionalClause()
        clause.cond = t[1]
        clause.true = t[3]
        clause.false = t[5]
        t[0] = clause

    # assignment rules
    def p_assignment_0(self, t):
        '''
        assignment : conditional
        '''
        t[0] = t[1]

    def p_assignment_1(self, t):
        '''
        assignment : operand EQUALS assignment
                   | operand ADDEQ assignment
                   | operand ANDEQ assignment
                   | operand DIVEQ assignment
                   | operand LSHEQ assignment
                   | operand MODEQ assignment
                   | operand MULEQ assignment
                   | operand OREQ assignment
                   | operand RSHEQ assignment
                   | operand SUBEQ assignment
                   | operand XOREQ assignment
        '''
        clause = AssignmentClause()
        clause.dst = t[1]
        clause.op = t[2]
        clause.src = t[3]
        t[0] = clause

    # ifthenelse rules
    def p_then_stmt_0(self, t):
        '''
        then_stmt : empty
        '''
        t[0] = []

    def p_then_stmt_1(self, t):
        '''
        then_stmt : statement
        '''
        t[0] = t[1]

    def p_then_stmt_2(self, t):
        '''
        then_stmt : THEN statement
        '''
        t[0] = t[2]

    def p_then_stmt_3(self, t):
        '''
        then_stmt : THEN statement terminator
        '''
        t[0] = t[2]

    def p_else_stmt_0(self, t):
        '''
        else_stmt : empty
        '''
        t[0] = []

    def p_else_stmt_1(self, t):
        '''
        else_stmt : ELSE statement
        '''
        t[0] = t[2]

    def p_else_stmt_2(self, t):
        '''
        else_stmt : ELSE statement terminator
        '''
        t[0] = t[2]

    # expression rules
    def p_expression_0(self, t):
        '''
        expression : assignment
        '''
        t[0] = t[1]

    # terminator rules
    def p_terminator_0(self, t):
        '''
        terminator : NEWLINE
                   | SEMI
                   | DOT
                   | comment
        '''
        if type(t[1]) is CommentClause:
            clause = t[1]
        else:
            clause = CommentClause()
            clause.content = [ t[1] ]
        t[0] = clause

    # statement rules
    def p_statement_0(self, t):
        '''
        statement : assignment terminator
        '''
        t[0] = [t[1], t[2]]

    def p_statement_2(self, t):
        '''
        statement : assignment COMMA assignment
        '''
        clause = ChainClause()
        clause.left = t[1]
        clause.right = t[3]
        t[0] = [clause]

    def p_statement_3(self, t):
        '''
        statement : IF LPAREN conditional RPAREN then_stmt else_stmt
        '''
        clause = IfThenElseClause()
        clause.cond = t[3]
        clause.then_stmt = t[5]
        clause.else_stmt = t[6]
        t[0] = [clause]

    def p_statement_4(self, t):
        '''
        statement : IF LPAREN conditional RPAREN terminator
        '''
        clause = IfClause()
        clause.cond = t[3]
        t[0] = [clause]

    def p_statement_5(self, t):
        '''
        statement : ELSE terminator
        '''
        clause = ElseClause()
        t[0] = [clause]

    def p_statement_6(self, t):
        '''
        statement : ELSIF LPAREN conditional RPAREN terminator
        '''
        clause = ElseIfClause()
        clause.cond = t[3]
        t[0] = [clause]

    def p_statement_7(self, t):
        '''
        statement : FOR I IN expression DOTDOTDOT expression DO terminator
        '''
        clause = ForClause()
        clause.variable = t[2]
        clause.start = t[4]
        clause.end = t[6]
        t[0] = [clause]

    def p_statement_8(self, t):
        '''
        statement : END terminator
        '''
        clause = EndClause()
        t[0] = [clause]

    def p_statement_9(self, t):
        '''
        statement : TAB statement terminator
        '''
        clause = TabClause()
        clause.stmt = t[2]
        t[0] = [clause]

    def p_statement_10(self, t):
        '''
        statement : NUMBER B COLON
                  | NUMBER B DOT
        '''
        clause = SizeClause()
        clause.size = t[1]
        t[0] = [clause]

    def p_statement_11(self, t):
        '''
        statement : terminator
        '''
        t[0] = [t[1]]

    # statements rules
    def p_statements_0(self, t):
        '''
        statements : statement
        '''
        t[0] = t[1]

    def p_statements_1(self, t):
        '''
        statements : statements statement
        '''
        t[1].extend(t[2])
        t[0] = t[1]

    # specification rule
    def p_specification(self, t):
        '''
        specification : statements
        '''
        t[0] = t[1]


    # error rule
    def p_error(self, t):
        if t:
            # sys.exit('%d: syntax error at "%s"' % (t.lexer.lineno, t.value))
            pass
        else:
            # sys.exit('unknown syntax error')
            pass
        self.yacc.errok()
        raise ParseError('p_error')

    # end rules
    def cleanup_substitute(self, desc):
        for sub in self.repeat_subs:
            if sub[0] in desc:
                desc = desc.replace(sub[0], sub[1])
        for sub in self.single_subs:
            if sub[0] in desc:
                return desc.replace(sub[0], sub[1])
        return desc

    def cleanup_description(self, desc_list):
        edited = []
        for d in desc_list:
            if d:
                lines = d.split('\\n')
                for l in lines:
                    if l:
                        e = self.cleanup_substitute(l + '\n')
                        edited.append(e)
        return ''.join(edited)

    def parse_description(self, desc_list):
        '''
        Parse the description strings
        '''
        assert(type(desc_list) is list)
        self.in_text = False
        desc_string = self.cleanup_description(desc_list)
        # import pdb; pdb.set_trace()
        # print 'desc_string=%s' % repr(desc_string)
        return self.parse_string(desc_string, '<string>', debug=False)
