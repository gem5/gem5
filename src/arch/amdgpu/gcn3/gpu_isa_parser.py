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

import re
import sys

from ast_objects import *
from m5.util.grammar import Grammar

class GpuIsaParser(Grammar):
    def __init__(self, input_file, output_dir):
        super(GpuIsaParser, self).__init__()
        self.input_file = input_file
        self.output_dir = output_dir

    # lexer

    states = (
        ( 'qstring', 'exclusive' ),
    )

    reserved = (
        'BITS',
        'CONST',
        'DESC',
        'DP_ONLY',
        'DST_0',
        'DST_1',
        'ENC',
        'ENCODING',
        'ENUM',
        'FIELDS',
        'FLAG',
        'FLAGS',
        'FMT',
        'GROUP',
        'IMPORT',
        'INOUT',
        'INST',
        'NAME',
        'NUM_DST',
        'NUM_SRC',
        'OP_TYPE',
        'OPERANDS',
        'PARENT_ENC',
        'PRIVATE',
        'RANGE',
        'REG',
        'SIZE',
        'SIZE_BITS',
        'SP3_DESC',
        'SP3_NAME',
        'SP3_NCOMP',
        'SP3_NUM',
        'SRC_0',
        'SRC_1',
        'SRC_2',
        'SRC_3',
        'SRC_FLAGS',
        'SUB_ENC',
        'TYPE',
        'WHEN'
    )

    # list token names
    tokens = reserved + (
        'ID',
        'STRING',
        'QSTRING',
        'NUMBER',
        'COLON',
        'SEMI',
        'DOLLAR',
        'EQUAL',
        'LBRACE',
        'RBRACE',
        'LPAREN',
        'RPAREN',
        'PLUS',
    )

    # regular expressions for simple tokens
    t_COLON     = r':'
    t_SEMI      = r';'
    t_DOLLAR    = r'\$'
    t_EQUAL     = r'='
    t_LBRACE    = r'\{'
    t_RBRACE    = r'\}'
    t_LPAREN    = r'\('
    t_RPAREN    = r'\)'
    t_PLUS      = r'\+'

    reserved_map = { }
    for r in reserved:
        reserved_map[r.lower()] = r

    def t_QSTRING(self, t):
        r'q{'
        nesting = 0
        t.lexer.begin('qstring')
        while True:
            tok = t.lexer.token()
            if not tok:
                break
            t.value += tok.value
            if tok.type == 'LBRACE':
                nesting += 1
            if tok.type == 'RBRACE':
                if nesting > 0:
                    nesting -= 1
                else:
                    break
        t.lexer.begin('INITIAL')
        t.lexer.lineno += t.value.count('\n')
        return t

    t_qstring_LBRACE = r'{'
    t_qstring_RBRACE = r'}'
    t_qstring_STRING = r'[^}{]+'

    t_qstring_ignore = ' \t\x0c'

    def t_qstring_error(self, t):
        sys.exit('%d: illegal character "%s"' % (t.lexer.lineno, t.value[0]))

    def t_ID(self, t):
        r'[a-zA-Z_][a-zA-Z0-9_]*'
        t.type = self.reserved_map.get(t.value, 'ID')
        return t

    def t_STRING(self, t):
        r'(?m)"([^"])+"'
        t.value = t.value[1:-1]
        t.lexer.lineno += t.value.count('\n')
        return t

    def t_NEWLINE(self, t):
        r'\n+'
        t.lexer.lineno += t.value.count('\n')

    def t_BINARY(self, t):
        r'0[bB][0-1]+'
        t.value = int(t.value, 2)
        t.type = 'NUMBER'
        return t

    def t_OCTAL(self, t):
        r'0[oO]?[0-7]+'
        t.value = int(t.value, 8)
        t.type = 'NUMBER'
        return t

    def t_HEXADECIMAL(self, t):
        r'0[xX][0-9a-fA-F]+'
        t.value = int(t.value, 16)
        t.type = 'NUMBER'
        return t

    def t_DECIMAL(self, t):
        r'[0-9][0-9]*'
        t.value = int(t.value, 10)
        t.type = 'NUMBER'
        return t

    def t_COMMENT(self, t):
        r'\#[^\n]*\n'
        t.lexer.lineno += 1

    t_ignore = ' \t\x0c'

    def t_error(self, t):
        sys.exit('%d: illegal character "%s"' % (t.lexer.lineno, t.value[0]))

    ## parser

    start = 'specification'

    # enum_or_reg rules

    def p_enum_or_reg_0(self, t):
        '''
        enum_or_reg : ENUM
        '''
        t[0] = t[1]

    def p_enum_or_reg_1(self, t):
        '''
        enum_or_reg : REG
        '''
        t[0] = t[1]

    # statement(import_statement) rules
    def p_import_statement(self, t):
        '''
        statement : IMPORT enum_or_reg ID SEMI
        '''
        stmnt = ImportStatement()
        stmnt.what = t[2]
        stmnt.name = t[3]
        t[0] = stmnt

    # flags_clause rules
    def p_flags_field_0(self, t):
        '''
        flags_field : DESC COLON STRING
        '''
        info = FlagsField()
        info.tag = 'desc'
        info.desc = [t[3]]
        t[0] = info

    def p_flags_field_1(self, t):
        '''
        flags_field : DESC PLUS COLON STRING
        '''
        info = FlagsField()
        info.tag = 'desc+'
        info.desc = [t[4]]
        t[0] = info

    def p_flags_field_2(self, t):
        '''
        flags_field : PRIVATE COLON NUMBER
        '''
        info = FlagsField()
        info.tag = 'private'
        info.private = t[3]
        t[0] = info

    def p_flags_field_3(self, t):
        '''
        flags_field : GROUP COLON ID
        '''
        info = FlagsField()
        info.tag = 'group'
        info.group = t[3]
        t[0] = info

    # flags_clauses rules
    def p_flags_fields_0(self, t):
        '''
        flags_fields : flags_field
        '''
        t[0] = t[1]

    def p_flags_fields_1(self, t):
        '''
        flags_fields : flags_fields flags_field
        '''
        t[1].update(t[2])
        t[0] = t[1]

    # flags_clause rules
    def p_flags_clause(self, t):
        '''
        flags_clause : ID COLON flags_fields SEMI
        '''
        field = FlagsField()
        field.tag = 'name'
        field.name = t[1]
        t[3].update(field)
        t[0] = t[3]

    # flags_clauses rules
    def p_flags_clauses_0(self, t):
        '''
        flags_clauses : flags_clause
        '''
        t[0] = [t[1]]

    def p_flags_clauses_1(self, t):
        '''
        flags_clauses : flags_clauses flags_clause
        '''
        t[1].append(t[2])
        t[0] = t[1]

    # flag_clause rules
    # FlagBlock:
    def p_flag_clause(self, t):
        '''
        flag_clause : DESC EQUAL STRING SEMI
        '''
        block = FlagBlock()
        block.tag = 'desc'
        block.desc = [t[3]]
        t[0] = block

    # flag_clauses rules
    # FlagBlock:
    def p_flag_clauses_0(self, t):
        '''
        flag_clauses : flag_clause
        '''
        t[0] = t[1]

    def p_flag_clauses_1(self, t):
        '''
        flag_clauses : flag_clauses flag_clause
        '''
        t[1].update(t[2])
        t[0] = t[1]

    # statement(flag_block) rules
    # FlagBlock:
    def p_flag_block(self, t):
        '''
        statement : FLAG ID LBRACE flag_clauses RBRACE
        '''
        block = FlagBlock()
        block.tag = 'name'
        block.name = t[2]
        t[4].update(block)
        t[0] = t[4]

    # statement(flags_block) rules
    # FlagsBlock:
    #     clauses: [ FlagsFields, ... ]
    def p_flags_block(self, t):
        '''
        statement : FLAGS LBRACE flags_clauses RBRACE
        '''
        block = FlagsBlock()
        block.clauses = t[3]
        t[0] = block

    # dst_X rules
    def p_dst_0(self, t):
        '''
        dst_X : DST_0
        '''
        t[0] = 0

    def p_dst_1(self, t):
        '''
        dst_X : DST_1
        '''
        t[0] = 1

    # src_X rules
    def p_src_0(self, t):
        '''
        src_X : SRC_0
        '''
        t[0] = 0

    def p_src_1(self, t):
        '''
        src_X : SRC_1
        '''
        t[0] = 1

    def p_src_2(self, t):
        '''
        src_X : SRC_2
        '''
        t[0] = 2

    def p_src_3(self, t):
        '''
        src_X : SRC_3
        '''
        t[0] = 3

    # operands_phrase rules
    # OpInfo
    def p_operands_phrase_0(self, t):
        '''
        operands_phrase : dst_X EQUAL ID
        '''
        phrase = OpInfo()
        phrase.tag = 'dst'
        phrase.opr = 'dst'
        phrase.index = t[1]
        phrase.iseq = t[3]
        t[0] = phrase

    def p_operands_phrase_1(self, t):
        '''
        operands_phrase : dst_X COLON
        '''
        phrase = OpInfo()
        phrase.tag = 'dst'
        phrase.opr = 'dst'
        phrase.index = t[1]
        phrase.iseq = None
        t[0] = phrase

    def p_operands_phrase_2(self, t):
        '''
        operands_phrase : src_X EQUAL ID
        '''
        phrase = OpInfo()
        phrase.tag = 'src'
        phrase.opr = 'src'
        phrase.index = t[1]
        phrase.iseq = t[3]
        t[0] = phrase

    def p_operands_phrase_3(self, t):
        '''
        operands_phrase : src_X COLON
        '''
        phrase = OpInfo()
        phrase.tag = 'src'
        phrase.opr = 'src'
        phrase.index = t[1]
        phrase.iseq = None
        t[0] = phrase

    def p_operands_phrase_4(self, t):
        '''
        operands_phrase : NAME COLON STRING
        '''
        phrase = OpInfo()
        phrase.tag = 'name'
        phrase.name = t[3]
        t[0] = phrase

    def p_operands_phrase_5(self, t):
        '''
        operands_phrase : FMT COLON ID
        '''
        phrase = OpInfo()
        phrase.tag = 'fmt'
        phrase.fmt = t[3]
        t[0] = phrase

    def p_operands_phrase_6(self, t):
        '''
        operands_phrase : SIZE COLON NUMBER
        '''
        phrase = OpInfo()
        phrase.tag = 'size'
        phrase.size = t[3]
        t[0] = phrase

    def p_operands_phrase_7(self, t):
        '''
        operands_phrase : INOUT COLON NUMBER
        '''
        phrase = OpInfo()
        phrase.tag = 'inout'
        phrase.inout = t[3]
        t[0] = phrase

    # operands_phrases rules
    # OpInfo
    def p_operands_phrases_0(self, t):
        '''
        operands_phrases : operands_phrase
        '''
        t[0] = t[1]

    def p_operands_phrases_1(self, t):
        '''
        operands_phrases : operands_phrases operands_phrase
        '''
        t[1].update(t[2])
        t[0] = t[1]

    # operands_clause rules
    # Operand:
    #     dst: [ OpInfo, ... ]
    #     src: [ OpInfo, ... ]
    #     when: [ WhenBlock, ... ]
    #     operands: [ Operand, ... ]
    def p_operands_clause_0(self, t):
        '''
        operands_clause : NUM_DST EQUAL NUMBER SEMI
        '''
        clause = Operand()
        clause.tag = 'num_dst'
        clause.num_dst = t[3]
        t[0] = clause

    def p_operands_clause_1(self, t):
        '''
        operands_clause : NUM_SRC EQUAL NUMBER SEMI
        '''
        clause = Operand()
        clause.tag = 'num_src'
        clause.num_src = t[3]
        t[0] = clause

    def p_operands_clause_2(self, t):
        '''
        operands_clause : PARENT_ENC EQUAL ID SEMI
        '''
        clause = Operand()
        clause.tag = 'parent_enc'
        clause.parent_enc = t[3]
        t[0] = clause

    def p_operands_clause_3(self, t):
        '''
        operands_clause : SUB_ENC EQUAL ID SEMI
        '''
        clause = Operand()
        clause.tag = 'sub_enc'
        clause.sub_enc = t[3]
        t[0] = clause

    def p_operands_clause_4(self, t):
        '''
        operands_clause : FLAGS EQUAL ID SEMI
        '''
        clause = Operand()
        clause.tag = 'flags'
        clause.flags = [t[3]]
        t[0] = clause

    def p_operands_clause_5(self, t):
        '''
        operands_clause : FLAGS EQUAL STRING SEMI
        '''
        clause = Operand()
        clause.tag = 'flags'
        # handle flags = 'flag1 flag2'
        regexp = re.compile('\w+')
        clause.flags = regexp.findall(t[3])
        t[0] = clause

    def p_operands_clause_6(self, t):
        '''
        operands_clause : operands_phrases SEMI
        '''
        clause = Operand()
        clause.tag = t[1].opr
        if clause.tag == 'dst':
            clause.dst = [t[1]]
        elif clause.tag == 'src':
            clause.src = [t[1]]
        else:
            assert (False), 'p_operands_clause_6: unexpected tag ' + clause.tag
        t[0] = clause

    def p_operands_clause_7(self, t):
        '''
        operands_clause : when_block
        '''
        clause = Operand()
        clause.tag = 'when'
        clause.when = [t[1]]
        t[0] = clause

    def p_operands_clause_8(self, t):
        '''
        operands_clause : operands_block
        '''
        clause = Operand()
        clause.tag = 'operands'
        clause.operands = [t[1]]
        t[0] = clause

    # operands_clauses rules
    # Operand
    def p_operands_clauses_0(self, t):
        '''
        operands_clauses : operands_clause
        '''
        t[0] = t[1]

    def p_operands_clauses_1(self, t):
        '''
        operands_clauses : operands_clauses operands_clause
        '''
        t[1].update(t[2])
        t[0] = t[1]

    # when_block rules
    # WhenBlock:
    #     operands: [ Operand, ... ]
    def p_when_block_0(self, t):
        '''
        when_block : WHEN FLAGS EQUAL ID LBRACE operands_clauses RBRACE
        '''
        block = WhenBlock()
        block.left = 'flags'
        block.right = [t[4]]
        block.operand = t[6]
        t[0] = block

    def p_when_block_1(self, t):
        '''
        when_block : WHEN FLAGS EQUAL STRING LBRACE operands_clauses RBRACE
        '''
        block = WhenBlock()
        block.left = 'flags'
        # handle when flags = 'flag1 flag2'
        regexp = re.compile('\w+')
        block.right = regexp.findall(t[4])
        block.operand = t[6]
        t[0] = block

    def p_when_block_2(self, t):
        '''
        when_block : WHEN SUB_ENC EQUAL STRING LBRACE operands_clauses RBRACE
        '''
        block = WhenBlock()
        block.left = 'sub_enc'
        block.right = [t[4]]
        block.operand = t[6]
        t[0] = block

    # operands_block rules
    # Operand
    def p_operands_block(self, t):
        '''
        operands_block : OPERANDS LBRACE operands_clauses RBRACE
        '''
        t[0] = t[3]

    # encoding_clause rules
    # EncodingBlock:
    #     operands: [ Operand, ... ]
    def p_encoding_clause_0(self, t):
        '''
        encoding_clause : BITS EQUAL STRING SEMI
        '''
        clause = EncodingBlock()
        clause.tag = 'bits'
        clause.bits = t[3]
        t[0] = clause

    def p_encoding_clause_1(self, t):
        '''
        encoding_clause : SIZE EQUAL NUMBER SEMI
        '''
        clause = EncodingBlock()
        clause.tag = 'size'
        clause.size = t[3]
        t[0] = clause

    def p_encoding_clause_2(self, t):
        '''
        encoding_clause : DESC EQUAL STRING SEMI
        '''
        clause = EncodingBlock()
        clause.tag = 'desc'
        clause.desc = [t[3]]
        t[0] = clause

    def p_encoding_clause_3(self, t):
        '''
        encoding_clause : DESC PLUS EQUAL STRING SEMI
        '''
        clause = EncodingBlock()
        clause.tag = 'desc+'
        clause.desc = [t[4]]
        t[0] = clause

    def p_encoding_clause_4(self, t):
        '''
        encoding_clause : operands_block
        '''
        clause = EncodingBlock()
        clause.tag = 'operands'
        clause.operands = [t[1]]
        t[0] = clause

    # encoding clauses rules
    # EncodingBlock:
    #     operands: [ Operand, ... ]
    def p_encoding_clauses_0(self, t):
        '''
        encoding_clauses : encoding_clause
        '''
        t[0] = t[1]

    def p_encoding_clauses_1(self, t):
        '''
        encoding_clauses : encoding_clauses encoding_clause
        '''
        t[1].update(t[2])
        t[0] = t[1]

    # encoding_block rules
    # EncodingBlock:
    #     operands: [ Operand, ... ]

    def p_encoding_block(self, t):
        '''
        statement : ENCODING ID LBRACE encoding_clauses RBRACE
        '''
        clause = EncodingBlock()
        clause.tag = 'name'
        clause.name = t[2]
        t[4].update(clause)
        t[0] = t[4]

    # const_clauses rules
    def p_const_clause(self, t):
        '''
        const_clause : ID EQUAL NUMBER SEMI
        '''
        clause = ConstClause()
        clause.name = t[1]
        clause.value = t[3]
        t[0] = clause

    # const_clauses rules
    def p_const_clauses_0(self, t):
        '''
        const_clauses : const_clause
        '''
        t[0] = [t[1]]

    def p_const_clauses_1(self, t):
        '''
        const_clauses : const_clauses const_clause
        '''
        t[1].append(t[2])
        t[0] = t[1]

    # statement(const_block) rules
    def p_const_block(self, t):
        '''
        statement : CONST LBRACE const_clauses RBRACE
        '''
        stmnt = ConstBlock()
        stmnt.clauses = t[3]
        t[0] = stmnt

    # type_phrase rules
    def p_type_phrase_0(self, t):
        '''
        type_phrase : ID EQUAL NUMBER COLON NUMBER
        '''
        field = TypeClause()
        field.tag = 'id_range'
        field.name = t[1]
        field.value = t[3]
        field.v_max = t[5]
        t[0] = field

    def p_type_phrase_1(self, t):
        '''
        type_phrase : ID EQUAL NUMBER
        '''
        field = TypeClause()
        field.tag = 'id_number'
        field.name = t[1]
        field.value = t[3]
        t[0] = field

    def p_type_phrase_2(self, t):
        '''
        type_phrase : ID DOLLAR LBRACE ID RBRACE ID EQUAL NUMBER
        '''
        field = TypeClause()
        field.tag = 'id_var_number'
        field.name = t[1] + '${' + t[4] + '}' + t[6]
        field.value = t[8]
        field.var = True
        t[0] = field

    def p_type_phrase_3(self, t):
        '''
        type_phrase : ID DOLLAR LBRACE ID RBRACE EQUAL NUMBER
        '''
        field = TypeClause()
        field.tag = 'id_var_number'
        field.name = t[1] + '${' + t[4] + '}'
        field.value = t[7]
        field.var = True
        t[0] = field

    def p_type_phrase_4(self, t):
        '''
        type_phrase : DESC COLON STRING
        '''
        field = TypeClause()
        field.tag = 'desc'
        field.desc = [t[3]]
        t[0] = field

    def p_type_phrase_5(self, t):
        '''
        type_phrase : DESC COLON ID LPAREN ID RPAREN
        '''
        field = TypeClause()
        field.tag = 'desc'
        field.desc = [ t[3] + '(' + t[5] + ')' ]
        t[0] = field

    def p_type_phrase_6(self, t):
        '''
        type_phrase : DESC PLUS COLON ID LPAREN ID RPAREN
        '''
        field = TypeClause()
        field.tag = 'desc+'
        field.desc = [ t[4] + '(' + t[6] + ')' ]
        t[0] = field

    def p_type_phrase_7(self, t):
        '''
        type_phrase : DESC PLUS COLON STRING
        '''
        field = TypeClause()
        field.tag = 'desc+'
        field.desc = [t[4]]
        t[0] = field

    def p_type_phrase_8a(self, t):
        '''
        type_phrase : DESC COLON QSTRING
        '''
        field = TypeClause()
        field.tag = 'desc'
        field.desc = t[3][2:-1].strip().split('\n')
        t[0] = field

    def p_type_phrase_8b(self, t):
        '''
        type_phrase : DESC PLUS COLON QSTRING
        '''
        field = TypeClause()
        field.tag = 'desc+'
        field.desc = t[4][2:-1].strip().split('\n')
        t[0] = field

    def p_type_phrase_9(self, t):
        '''
        type_phrase : FLAGS COLON ID
        '''
        field = TypeClause()
        field.tag = 'flags'
        field.flags = [t[3]]
        t[0] = field

    def p_type_phrase_10(self, t):
        '''
        type_phrase : FLAGS COLON STRING
        '''
        field = TypeClause()
        field.tag = 'flags'
        # handle flags: 'flag1 flag2'
        regexp = re.compile('\w+')
        field.flags = regexp.findall(t[3])
        t[0] = field

    def p_type_phrase_11(self, t):
        '''
        type_phrase : FLAGS PLUS COLON ID
        '''
        field = TypeClause()
        field.tag = 'flags+'
        field.flags = [t[4]]
        t[0] = field

    def p_type_phrase_12(self, t):
        '''
        type_phrase : FLAGS PLUS COLON STRING
        '''
        field = TypeClause()
        field.tag = 'flags+'
        # handle flags+: 'flag1 flag2'
        regexp = re.compile('\w+')
        field.flags = regexp.findall(t[4])
        t[0] = field

    def p_type_phrase_13(self, t):
        '''
        type_phrase : SRC_FLAGS COLON ID
        '''
        field = TypeClause()
        field.tag = 'src_flags'
        field.src_flags = t[3]
        t[0] = field

    def p_type_phrase_14(self, t):
        '''
        type_phrase : SRC_FLAGS COLON STRING
        '''
        field = TypeClause()
        field.tag = 'src_flags'
        field.src_flags = t[3]
        t[0] = field

    def p_type_phrase_15(self, t):
        '''
        type_phrase : SP3_DESC COLON STRING
        '''
        field = TypeClause()
        field.tag = 'sp3_desc'
        field.sp3_desc = [t[3]]
        t[0] = field

    def p_type_phrase_16(self, t):
        '''
        type_phrase : SP3_DESC PLUS COLON STRING
        '''
        field = TypeClause()
        field.tag = 'sp3_desc+'
        field.sp3_desc = [t[4]]
        t[0] = field

    def p_type_phrase_17(self, t):
        '''
        type_phrase : SP3_NAME COLON STRING
        '''
        field = TypeClause()
        field.tag = 'sp3_name'
        field.sp3_name = t[3]
        t[0] = field

    def p_type_phrase_18(self, t):
        '''
        type_phrase : SP3_NCOMP COLON NUMBER
        '''
        field = TypeClause()
        field.tag = 'sp3_ncomp'
        field.sp3_ncomp = t[3]
        t[0] = field

    def p_type_phrase_19(self, t):
        '''
        type_phrase : SP3_NUM COLON STRING
        '''
        field = TypeClause()
        field.tag = 'sp3_num'
        field.sp3_num = t[3]
        t[0] = field

    def p_type_phrase_20(self, t):
        '''
        type_phrase : SUB_ENC COLON ID
        '''
        field = TypeClause()
        field.tag = 'sub_enc'
        field.sub_enc = t[3]
        t[0] = field

    def p_type_phrase_21(self, t):
        '''
        type_phrase : OP_TYPE COLON ID
        '''
        field = TypeClause()
        field.tag = 'op_type'
        field.op_type = t[3]
        t[0] = field

    def p_type_phrase_22(self, t):
        '''
        type_phrase : DP_ONLY COLON NUMBER
        '''
        field = TypeClause()
        field.tag = 'dp_only'
        field.sub_enc = t[3]
        t[0] = field

    def p_type_phrase_23(self, t):
        '''
        type_phrase : SIZE COLON NUMBER
        '''
        field = TypeClause()
        field.tag = 'size'
        field.size = t[3]
        t[0] = field

    def p_type_phrase_24(self, t):
        '''
        type_phrase : FMT COLON ID
        '''
        field = TypeClause()
        field.tag = 'fmt'
        field.fmt = t[3]
        t[0] = field

    def p_type_phrase_25(self, t):
        '''
        type_phrase : TYPE ID
        '''
        field = TypeClause()
        field.tag = 'type'
        field.type = t[2]
        t[0] = field

    def p_type_phrase_26(self, t):
        '''
        type_phrase : RANGE NUMBER COLON NUMBER
        '''
        field = TypeClause()
        field.tag = 'range'
        field.range = [ t[2], t[4] ]
        t[0] = field

    def p_type_phrase_27(self, t):
        '''
        type_phrase : SIZE_BITS NUMBER
        '''
        field = TypeClause()
        field.tag = 'size_bits'
        field.size_bits = t[2]
        t[0] = field

    # type_phrases rules
    # [ TypeClause, ... ]
    def p_type_phrases_0(self, t):
        '''
        type_phrases : type_phrase
        '''
        t[0] = t[1]

    def p_type_phrases_1(self, t):
        '''
        type_phrases : type_phrases type_phrase
        '''
        t[1].update(t[2])
        t[0] = t[1]

    # type_clause rules
    # [ TypeClause, ... ]
    def p_type_clause(self, t):
        '''
        type_clause : type_phrases SEMI
        '''
        t[0] = t[1]

    # type_clauses rules
    def p_type_clauses_0(self, t):
        '''
        type_clauses : type_clause
        '''
        t[0] = [t[1]]

    def p_type_clauses_1(self, t):
        '''
        type_clauses : type_clauses type_clause
        '''
        t[1].append(t[2])
        t[0] = t[1]

    # statement(type_block) rules
    # TypeBlock:
    #     clauses: [ TypeClause, ... ]
    def p_type_block(self, t):
        '''
        statement : TYPE ID LBRACE type_clauses RBRACE
        '''
        stmnt = TypeBlock()
        stmnt.name = t[2]
        stmnt.clauses = t[4]
        t[0] = stmnt

    # inst_field rules
    def p_inst_field_0(self, t):
        '''
        inst_field : ID EQUAL NUMBER COLON NUMBER
        '''
        field = InstField()
        field.tag = 'id_range'
        field.name = t[1]
        field.v_max = t[3]
        field.value = t[5]
        t[0] = field

    def p_inst_field_1(self, t):
        '''
        inst_field : ID EQUAL NUMBER
        '''
        field = InstField()
        field.tag = 'id_number'
        field.name = t[1]
        field.v_max = t[3]
        field.value = t[3]
        t[0] = field

    def p_inst_field_2(self, t):
        '''
        inst_field : DESC COLON STRING
        '''
        field = InstField()
        field.tag = 'desc'
        field.desc = t[3]
        t[0] = field

    def p_inst_field_3(self, t):
        '''
        inst_field : TYPE COLON ID
        '''
        field = InstField()
        field.tag = 'type'
        field.type = t[3]
        t[0] = field

    def p_inst_field_4(self, t):
        '''
        inst_field : ENC COLON ID
        '''
        field = InstField()
        field.tag = 'enc'
        field.enc = t[3]
        t[0] = field

    # fields_phrases rules
    # [ InstFields, ... ]
    def p_fields_phrases_0(self, t):
        '''
        fields_phrases : inst_field
        '''
        t[0] = t[1]

    def p_fields_phrases_1(self, t):
        '''
        fields_phrases : fields_phrases inst_field
        '''
        t[1].update(t[2])
        t[0] = t[1]

    # inst_fields rules
    # [ InstFields, ... ]
    def p_fields_clause(self, t):
        '''
        fields_clause : fields_phrases SEMI
        '''
        t[0] = t[1]

    # fields_clauses rules
    def p_fields_clauses_0(self, t):
        '''
        fields_clauses : fields_clause
        '''
        t[0] = [t[1]]

    def p_fields_clauses_1(self, t):
        '''
        fields_clauses : fields_clauses fields_clause
        '''
        t[1].append(t[2])
        t[0] = t[1]

    # fields_block rule
    # [ InstFields, ... ]
    def p_fields_block(self, t):
        '''
        fields_block : FIELDS LBRACE fields_clauses RBRACE
        '''
        t[0] = t[3]

    # inst_clause rules
    # InstBlock:
    #     fields: [ InstFields, ... ]
    def p_inst_clause_0(self, t):
        '''
        inst_clause : DESC EQUAL STRING SEMI
        '''
        clause = InstBlock()
        clause.tag = 'desc'
        clause.desc = t[3]
        t[0] = clause

    def p_inst_clause_1(self, t):
        '''
        inst_clause : fields_block
        '''
        clause = InstBlock()
        clause.tag = 'fields'
        clause.fields = t[1]
        t[0] = clause

    # inst_clauses rules
    # InstBlock:
    #     fields: [ InstField, ... ]
    def p_inst_clauses_0(self, t):
        '''
        inst_clauses : inst_clause
        '''
        t[0] = t[1]

    def p_inst_clauses_1(self, t):
        '''
        inst_clauses : inst_clauses inst_clause
        '''
        t[1].update(t[2])
        t[0] = t[1]

    # statement(inst_block) rules
    # InstBlock:
    #     fields: [ InstFields, ... ]
    def p_inst_block(self, t):
        '''
        statement : INST ID LBRACE inst_clauses RBRACE
        '''
        block = InstBlock()
        block.tag = 'name'
        # drop '_0' to simplify matches with encodings
        n = t[2]
        if '_0' in n:
            n = n.replace('_0', '')
        block.name = n
        t[4].update(block)
        t[0] = t[4]

    # statements rules
    def p_statements_0(self, t):
        '''
        statements : statement
        '''
        t[0] = [t[1]]

    def p_statements_1(self, t):
        '''
        statements : statements statement
        '''
        t[1].append(t[2])
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
            print '%d: syntax error at "%s"' % (t.lexer.lineno, t.value)
        else:
            print 'unknown syntax error'
        import pdb; pdb.set_trace()

    # end rules
    def parse_isa_desc(self):
        '''
        Read in and parse the ISA description
        '''
        try:
            contents = open(self.input_file).read()
        except IOError:
            error('Error with file "%s"' % self.input_file)

        return self.parse_string(contents, '<string>', debug=False)
