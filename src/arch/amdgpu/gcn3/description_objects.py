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

class Clause(object):
    def __init__(self):
        self.keyword = ''

class CommentClause(Clause):
    def __init__(self):
        self.keyword = 'comment'
        self.content = []

    def __repr__(self):
        text  = '[ kw:      %s,' % self.keyword
        text += ' content: %s ]' % repr(self.content)
        return text

class AssignmentClause(Clause):
    def __init__(self):
        self.keyword = 'assignment'
        self.dst = None
        self.op = None
        self.src = None

    def __repr__(self):
        text  = '[ kw: %s,' % self.keyword
        text += ' dst: %s,' % repr(self.dst)
        text += ' op: %s,' % repr(self.op)
        text += ' src: %s ]' % repr(self.src)
        return text

class BinaryClause(Clause):
    def __init__(self):
        self.keyword = 'binary'
        self.left = None
        self.op = None
        self.right = None

    def __repr__(self):
        text  = '[ kw: %s,' % self.keyword
        text += ' left: %s,' % repr(self.left)
        text += ' op: %s,' % repr(self.op)
        text += ' right: %s ]' % repr(self.right)
        return text

class UnaryClause(Clause):
    def __init__(self):
        self.keyword = 'unary'
        self.op = None
        self.oprnd = None

    def __repr__(self):
        text  = '[ kw: %s,' % self.keyword
        text += ' op: %s,' % repr(self.op)
        text += ' oprnd: %s ]' % repr(self.oprnd)
        return text

class ConditionalClause(Clause):
    def __init__(self):
        self.keyword = 'conditional'
        self.cond = None
        self.true = None
        self.false = None

    def __repr__(self):
        text  = '[ kw: %s,' % self.keyword
        text += ' cond: %s,' % repr(self.cond)
        text += ' true: %s,' % repr(self.true)
        text += ' false: %s ]' % repr(self.false)
        return text

class VariableClause(Clause):
    def __init__(self):
        self.keyword = 'variable'
        self.name = ''

    def __repr__(self):
        text  = '[ kw: %s,' % self.keyword
        text += ' name: %s ]' % repr(self.name)
        return text

class ConstantClause(Clause):
    def __init__(self):
        self.keyword = 'constant'
        self.value = 0

    def __repr__(self):
        text  = '[ kw: %s,' % self.keyword
        if type(self.value) is 'float':
            text += ' value: %f ]' % (self.value)
        else:
            text += ' value: 0x%x ]' % (self.value)

        return text

class DataRegClause(Clause):
    def __init__(self):
        self.keyword = 'data_reg'
        self.reg = None
        self.idx = None
        self.typ = [ 'default', -1 ]
        self.rng = None

    def __repr__(self):
        text  = '[ kw: %s,' % self.keyword
        text += ' reg: %s,' % repr(self.reg)
        if self.typ:
            text += ' typ: %s,' % repr(self.typ)
        else:
            text += ' typ: None,'
        if self.idx:
            text += ' idx: %s,' % repr(self.idx)
        else:
            text += ' idx: None,'
        if self.rng:
            text += ' rng: %s ]' % repr(self.rng)
        else:
            text += ' rng: None ]'
        return text

class FunctionClause(Clause):
    def __init__(self):
        self.keyword = 'function'
        self.func = ''
        self.args = []

    def __repr__(self):
        text  = '[ kw: %s,' % self.keyword
        text += ' func: %s,' % repr(self.func)
        text += ' args: %s ]' % repr(self.args)
        return text

class IfThenElseClause(Clause):
    def __init__(self):
        self.keyword = 'ifthenelse'
        self.cond = None
        self.then_stmt = None
        self.else_stmt = None

    def __repr__(self):
        text  = '[ kw: %s,' % self.keyword
        if self.cond:
            text += ' cond: %s,' % repr(self.cond)
        else:
            text += ' cond: None,'
        if self.then_stmt:
            text += ' then: %s,' % repr(self.then_stmt)
        else:
            text += ' then: None,'
        if self.else_stmt:
            text += ' else: %s ]' % repr(self.else_stmt)
        else:
            text += ' else: None ]'
        return text

class IfClause(Clause):
    def __init__(self):
        self.keyword = 'if'
        self.cond = None

    def __repr__(self):
        text  = '[ kw: %s,' % self.keyword
        if self.cond:
            text += ' cond: %s ]' % repr(self.cond)
        else:
            text += ' cond: None ]'
        return text

class ForClause(Clause):
    def __init__(self):
        self.keyword = 'for'
        self.variable = None
        self.start = None
        self.end = None

    def __repr__(self):
        text  = '[ kw: %s,' % self.keyword
        text += ' variable: %s,' % repr(self.variable)
        text += ' start: %s,' % repr(self.start)
        text += ' end: %s ]' % repr(self.end)
        return text

class ElseClause(Clause):
    def __init__(self):
        self.keyword = 'else'

    def __repr__(self):
        text  = '[ kw: %s ]' % self.keyword
        return text

class EndClause(Clause):
    def __init__(self):
        self.keyword = 'end'

    def __repr__(self):
        text  = '[ kw: %s ]' % self.keyword
        return text

class ElseIfClause(Clause):
    def __init__(self):
        self.keyword = 'elseif'
        self.cond = None

    def __repr__(self):
        text  = '[ kw: %s,' % self.keyword
        if self.cond:
            text += ' cond: %s ]' % repr(self.cond)
        else:
            text += ' cond: None ]'
        return text

class TabClause(Clause):
    def __init__(self):
        self.keyword = 'tab'
        self.stmt = None

    def __repr__(self):
        text  = '[ kw: %s,' % self.keyword
        if self.stmt:
            text += ' stmt: %s ]' % repr(self.stmt)
        else:
            text += ' stmt: None ]'
        return text

class GroupClause(Clause):
    def __init__(self):
        self.keyword = 'group'
        self.group = []

    def __repr__(self):
        text  = '[ kw: %s,' % self.keyword
        text += ' group: %s ]' % repr(self.group)
        return text

class CastClause(Clause):
    def __init__(self):
        self.keyword = 'cast'
        self.typ = ''
        self.var = ''

    def __repr__(self):
        text  = '[ kw: %s,' % self.keyword
        text += ' typ: %s,' % repr(self.typ)
        text += ' var: %s ]' % repr(self.var)
        return text

class CommaClause(Clause):
    def __init__(self):
        self.keyword = 'comma'
        self.left = None
        self.right = None

    def __repr__(self):
        text  = '[ kw: %s,' % self.keyword
        text += ' left: %s,' % repr(self.left)
        text += ' right: %s ]' % repr(self.right)
        return text

class ChainClause(Clause):
    def __init__(self):
        self.keyword = 'chain'
        self.left = None
        self.right = None

    def __repr__(self):
        text  = '[ kw: %s,' % self.keyword
        text += ' left: %s,' % repr(self.left)
        text += ' right: %s ]' % repr(self.right)
        return text

class MemClause(Clause):
    def __init__(self):
        self.keyword = 'mem'
        self.mem = None
        self.addr = None
        self.rng = None

    def __repr__(self):
        text  = '[ kw: %s,' % self.keyword
        text += ' mem: %s,' % repr(self.mem)
        text += ' addr: %s,' % repr(self.addr)
        if self.rng:
            text += ' rng: %s ]' % repr(self.rng)
        else:
            text += ' rng: None ]'
        return text

class GprClause(Clause):
    def __init__(self):
        self.keyword = 'gpr'
        self.gpr = None
        self.idx = None
        self.typ = None

    def __repr__(self):
        text  = '[ kw: %s,' % self.keyword
        text += ' gpr: %s,' % repr(self.gpr)
        text += ' idx: %s,' % repr(self.idx)
        text += ' typ: %s ]' % repr(self.typ)
        return text

class ParenClause(Clause):
    def __init__(self):
        self.keyword = 'paren'
        self.parexp = None

    def __repr__(self):
        text  = '[ kw: %s,' % self.keyword
        text += ' parexp: %s ]' % repr(self.parexp)
        return text

class SizeClause(Clause):
    def __init__(self):
        self.keyword = 'size'
        self.size = 0

    def __repr__(self):
        text  = '[ kw: %s,' % self.keyword
        text += ' size: %d ]' % self.size
        return text

TypeToDetails = {
    'f16'  : ( 'SregF16',  16 ),
    'i16'  : ( 'SregU16',  16 ),
    'u16'  : ( 'SregU16',  16 ),
    'f'    : ( 'SregF32',  32 ),
    'f32'  : ( 'SregF32',  32 ),
    'i'    : ( 'SregI32',  32 ),
    'i32'  : ( 'SregI32',  32 ),
    'u'    : ( 'SregU32',  32 ),
    'u32'  : ( 'SregU32',  32 ),
    'd'    : ( 'SregF64',  64 ),
    'i64'  : ( 'SregI64',  64 ),
    'u64'  : ( 'SregU64',  64 ),
    'u96'  : ( 'SregU96',  96 ),
    'u128' : ( 'SregU128', 128 ),
    'u256' : ( 'SregU256', 256 ),
    'u512' : ( 'SregU512', 512 )
}
