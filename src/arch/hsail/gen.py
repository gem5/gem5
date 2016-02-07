#! /usr/bin/python

#
#  Copyright (c) 2015 Advanced Micro Devices, Inc.
#  All rights reserved.
#
#  For use for simulation and test purposes only
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation
#  and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its contributors
#  may be used to endorse or promote products derived from this software
#  without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  Author: Steve Reinhardt
#

import sys, re

from m5.util import code_formatter

if len(sys.argv) != 4:
    print "Error: need 3 args (file names)"
    sys.exit(0)

header_code = code_formatter()
decoder_code = code_formatter()
exec_code = code_formatter()

###############
#
# Generate file prologs (includes etc.)
#
###############

header_code('''
#include "arch/hsail/insts/decl.hh"
#include "base/bitfield.hh"
#include "gpu-compute/hsail_code.hh"
#include "gpu-compute/wavefront.hh"

namespace HsailISA
{
''')
header_code.indent()

decoder_code('''
#include "arch/hsail/gpu_decoder.hh"
#include "arch/hsail/insts/branch.hh"
#include "arch/hsail/insts/decl.hh"
#include "arch/hsail/insts/gen_decl.hh"
#include "arch/hsail/insts/mem.hh"
#include "arch/hsail/insts/mem_impl.hh"
#include "gpu-compute/brig_object.hh"

namespace HsailISA
{
    std::vector<GPUStaticInst*> Decoder::decodedInsts;

    GPUStaticInst*
    Decoder::decode(MachInst machInst)
    {
        using namespace Brig;

        const BrigInstBase *ib = machInst.brigInstBase;
        const BrigObject *obj = machInst.brigObj;

        switch(ib->opcode) {
''')
decoder_code.indent()
decoder_code.indent()

exec_code('''
#include "arch/hsail/insts/gen_decl.hh"
#include "base/intmath.hh"

namespace HsailISA
{
''')
exec_code.indent()

###############
#
# Define code templates for class declarations (for header file)
#
###############

# Basic header template for an instruction with no template parameters.
header_template_nodt = '''
class $class_name : public $base_class
{
  public:
    typedef $base_class Base;

    $class_name(const Brig::BrigInstBase *ib, const BrigObject *obj)
       : Base(ib, obj, "$opcode")
    {
    }

    void execute(GPUDynInstPtr gpuDynInst);
};

'''

# Basic header template for an instruction with a single DataType
# template parameter.
header_template_1dt = '''
template<typename DataType>
class $class_name : public $base_class<DataType>
{
  public:
    typedef $base_class<DataType> Base;
    typedef typename DataType::CType CType;

    $class_name(const Brig::BrigInstBase *ib, const BrigObject *obj)
       : Base(ib, obj, "$opcode")
    {
    }

    void execute(GPUDynInstPtr gpuDynInst);
};

'''

header_template_1dt_noexec = '''
template<typename DataType>
class $class_name : public $base_class<DataType>
{
  public:
    typedef $base_class<DataType> Base;
    typedef typename DataType::CType CType;

    $class_name(const Brig::BrigInstBase *ib, const BrigObject *obj)
       : Base(ib, obj, "$opcode")
    {
    }
};

'''

# Same as header_template_1dt, except the base class has a second
# template parameter NumSrcOperands to allow a variable number of
# source operands.  Note that since this is implemented with an array,
# it only works for instructions where all sources are of the same
# type (like most arithmetics).
header_template_1dt_varsrcs = '''
template<typename DataType>
class $class_name : public $base_class<DataType, $num_srcs>
{
  public:
    typedef $base_class<DataType, $num_srcs> Base;
    typedef typename DataType::CType CType;

    $class_name(const Brig::BrigInstBase *ib, const BrigObject *obj)
       : Base(ib, obj, "$opcode")
    {
    }

    void execute(GPUDynInstPtr gpuDynInst);
};

'''

# Header template for instruction with two DataType template
# parameters, one for the dest and one for the source.  This is used
# by compare and convert.
header_template_2dt = '''
template<typename DestDataType, class SrcDataType>
class $class_name : public $base_class<DestDataType, SrcDataType>
{
  public:
    typedef $base_class<DestDataType, SrcDataType> Base;
    typedef typename DestDataType::CType DestCType;
    typedef typename SrcDataType::CType SrcCType;

    $class_name(const Brig::BrigInstBase *ib, const BrigObject *obj)
       : Base(ib, obj, "$opcode")
    {
    }

    void execute(GPUDynInstPtr gpuDynInst);
};

'''

header_templates = {
    'ArithInst': header_template_1dt_varsrcs,
    'CmovInst': header_template_1dt,
    'ClassInst': header_template_1dt,
    'ShiftInst': header_template_1dt,
    'ExtractInsertInst': header_template_1dt,
    'CmpInst': header_template_2dt,
    'CvtInst': header_template_2dt,
    'LdInst': '',
    'StInst': '',
    'SpecialInstNoSrc': header_template_nodt,
    'SpecialInst1Src': header_template_nodt,
    'SpecialInstNoSrcNoDest': '',
}

###############
#
# Define code templates for exec functions
#
###############

# exec function body
exec_template_nodt_nosrc = '''
void
$class_name::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *w = gpuDynInst->wavefront();

    typedef Base::DestCType DestCType;

    const VectorMask &mask = w->get_pred();

    for (int lane = 0; lane < VSZ; ++lane) {
        if (mask[lane]) {
            DestCType dest_val = $expr;
            this->dest.set(w, lane, dest_val);
        }
    }
}

'''

exec_template_nodt_1src = '''
void
$class_name::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *w = gpuDynInst->wavefront();

    typedef Base::DestCType DestCType;
    typedef Base::SrcCType  SrcCType;

    const VectorMask &mask = w->get_pred();

    for (int lane = 0; lane < VSZ; ++lane) {
        if (mask[lane]) {
            SrcCType src_val0 = this->src0.get<SrcCType>(w, lane);
            DestCType dest_val = $expr;

            this->dest.set(w, lane, dest_val);
        }
    }
}

'''

exec_template_1dt_varsrcs = '''
template<typename DataType>
void
$class_name<DataType>::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *w = gpuDynInst->wavefront();

    const VectorMask &mask = w->get_pred();

    for (int lane = 0; lane < VSZ; ++lane) {
        if (mask[lane]) {
            CType dest_val;
            if ($dest_is_src_flag) {
                dest_val = this->dest.template get<CType>(w, lane);
            }

            CType src_val[$num_srcs];

            for (int i = 0; i < $num_srcs; ++i) {
                src_val[i] = this->src[i].template get<CType>(w, lane);
            }

            dest_val = (CType)($expr);

            this->dest.set(w, lane, dest_val);
        }
    }
}

'''

exec_template_1dt_3srcs = '''
template<typename DataType>
void
$class_name<DataType>::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *w = gpuDynInst->wavefront();

    typedef typename Base::Src0CType Src0T;
    typedef typename Base::Src1CType Src1T;
    typedef typename Base::Src2CType Src2T;

    const VectorMask &mask = w->get_pred();

    for (int lane = 0; lane < VSZ; ++lane) {
        if (mask[lane]) {
            CType dest_val;

            if ($dest_is_src_flag) {
                dest_val = this->dest.template get<CType>(w, lane);
            }

            Src0T src_val0 = this->src0.template get<Src0T>(w, lane);
            Src1T src_val1 = this->src1.template get<Src1T>(w, lane);
            Src2T src_val2 = this->src2.template get<Src2T>(w, lane);

            dest_val = $expr;

            this->dest.set(w, lane, dest_val);
        }
    }
}

'''

exec_template_1dt_2src_1dest = '''
template<typename DataType>
void
$class_name<DataType>::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *w = gpuDynInst->wavefront();

    typedef typename Base::DestCType DestT;
    typedef CType Src0T;
    typedef typename Base::Src1CType Src1T;

    const VectorMask &mask = w->get_pred();

    for (int lane = 0; lane < VSZ; ++lane) {
        if (mask[lane]) {
            DestT dest_val;
            if ($dest_is_src_flag) {
                dest_val = this->dest.template get<DestT>(w, lane);
            }
            Src0T src_val0 = this->src0.template get<Src0T>(w, lane);
            Src1T src_val1 = this->src1.template get<Src1T>(w, lane);

            dest_val = $expr;

            this->dest.set(w, lane, dest_val);
        }
    }
}

'''

exec_template_shift = '''
template<typename DataType>
void
$class_name<DataType>::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *w = gpuDynInst->wavefront();

    const VectorMask &mask = w->get_pred();
    for (int lane = 0; lane < VSZ; ++lane) {
        if (mask[lane]) {
            CType dest_val;

            if ($dest_is_src_flag) {
                dest_val = this->dest.template get<CType>(w, lane);
            }

            CType src_val0 = this->src0.template get<CType>(w, lane);
            uint32_t src_val1 = this->src1.template get<uint32_t>(w, lane);

            dest_val = $expr;

            this->dest.set(w, lane, dest_val);
        }
    }
}

'''

exec_template_2dt = '''
template<typename DestDataType, class SrcDataType>
void
$class_name<DestDataType, SrcDataType>::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *w = gpuDynInst->wavefront();

    const VectorMask &mask = w->get_pred();

    for (int lane = 0; lane < VSZ; ++lane) {
        if (mask[lane]) {
            DestCType dest_val;
            SrcCType src_val[$num_srcs];

            for (int i = 0; i < $num_srcs; ++i) {
                src_val[i] = this->src[i].template get<SrcCType>(w, lane);
            }

            dest_val = $expr;

            this->dest.set(w, lane, dest_val);
        }
    }
}

'''

exec_templates = {
    'ArithInst': exec_template_1dt_varsrcs,
    'CmovInst': exec_template_1dt_3srcs,
    'ExtractInsertInst': exec_template_1dt_3srcs,
    'ClassInst': exec_template_1dt_2src_1dest,
    'CmpInst': exec_template_2dt,
    'CvtInst': exec_template_2dt,
    'LdInst': '',
    'StInst': '',
    'SpecialInstNoSrc': exec_template_nodt_nosrc,
    'SpecialInst1Src': exec_template_nodt_1src,
    'SpecialInstNoSrcNoDest': '',
}

###############
#
# Define code templates for the decoder cases
#
###############

# decode template for nodt-opcode case
decode_nodt_template = '''
  case BRIG_OPCODE_$brig_opcode_upper: return $constructor(ib, obj);'''

decode_case_prolog_class_inst = '''
  case BRIG_OPCODE_$brig_opcode_upper:
    {
        //const BrigOperandBase *baseOp = obj->getOperand(ib->operands[1]);
        BrigType16_t type = ((BrigInstSourceType*)ib)->sourceType;
        //switch (baseOp->kind) {
        //    case BRIG_OPERAND_REG:
        //        type = ((const BrigOperandReg*)baseOp)->type;
        //        break;
        //    case BRIG_OPERAND_IMMED:
        //        type = ((const BrigOperandImmed*)baseOp)->type;
        //        break;
        //    default:
        //        fatal("CLASS unrecognized kind of operand %d\\n",
        //               baseOp->kind);
        //}
        switch (type) {'''

# common prolog for 1dt- or 2dt-opcode case: switch on data type
decode_case_prolog = '''
  case BRIG_OPCODE_$brig_opcode_upper:
    {
        switch (ib->type) {'''

# single-level decode case entry (for 1dt opcodes)
decode_case_entry = \
'      case BRIG_TYPE_$type_name: return $constructor(ib, obj);'

decode_store_prolog = \
'      case BRIG_TYPE_$type_name: {'

decode_store_case_epilog = '''
    }'''

decode_store_case_entry = \
'          return $constructor(ib, obj);'

# common epilog for type switch
decode_case_epilog = '''
          default: fatal("$brig_opcode_upper: unrecognized type %d\\n",
              ib->type);
        }
    }
    break;'''

# Additional templates for nested decode on a second type field (for
# compare and convert).  These are used in place of the
# decode_case_entry template to create a second-level switch on on the
# second type field inside each case of the first-level type switch.
# Because the name and location of the second type can vary, the Brig
# instruction type must be provided in $brig_type, and the name of the
# second type field must be provided in $type_field.
decode_case2_prolog = '''
        case BRIG_TYPE_$type_name:
          switch (((Brig$brig_type*)ib)->$type2_field) {'''

decode_case2_entry = \
'          case BRIG_TYPE_$type2_name: return $constructor(ib, obj);'

decode_case2_epilog = '''
          default: fatal("$brig_opcode_upper: unrecognized $type2_field %d\\n",
                         ((Brig$brig_type*)ib)->$type2_field);
        }
        break;'''

# Figure out how many source operands an expr needs by looking for the
# highest-numbered srcN value referenced.  Since sources are numbered
# starting at 0, the return value is N+1.
def num_src_operands(expr):
    if expr.find('src2') != -1:
        return 3
    elif expr.find('src1') != -1:
        return 2
    elif expr.find('src0') != -1:
        return 1
    else:
        return 0

###############
#
# Define final code generation methods
#
# The gen_nodt, and gen_1dt, and gen_2dt methods are the interface for
# generating actual instructions.
#
###############

# Generate class declaration, exec function, and decode switch case
# for an brig_opcode with a single-level type switch.  The 'types'
# parameter is a list or tuple of types for which the instruction
# should be instantiated.
def gen(brig_opcode, types=None, expr=None, base_class='ArithInst',
        type2_info=None, constructor_prefix='new ', is_store=False):
    brig_opcode_upper = brig_opcode.upper()
    class_name = brig_opcode
    opcode = class_name.lower()

    if base_class == 'ArithInst':
        # note that expr must be provided with ArithInst so we can
        # derive num_srcs for the template
        assert expr

    if expr:
        # Derive several bits of info from expr.  If expr is not used,
        # this info will be irrelevant.
        num_srcs = num_src_operands(expr)
        # if the RHS expression includes 'dest', then we're doing an RMW
        # on the reg and we need to treat it like a source
        dest_is_src = expr.find('dest') != -1
        dest_is_src_flag = str(dest_is_src).lower() # for C++
        if base_class in ['ShiftInst']:
            expr = re.sub(r'\bsrc(\d)\b', r'src_val\1', expr)
        elif base_class in ['ArithInst', 'CmpInst', 'CvtInst']:
            expr = re.sub(r'\bsrc(\d)\b', r'src_val[\1]', expr)
        else:
            expr = re.sub(r'\bsrc(\d)\b', r'src_val\1', expr)
        expr = re.sub(r'\bdest\b', r'dest_val', expr)

    # Strip template arguments off of base class before looking up
    # appropriate templates
    base_class_base = re.sub(r'<.*>$', '', base_class)
    header_code(header_templates[base_class_base])

    if base_class.startswith('SpecialInst'):
        exec_code(exec_templates[base_class_base])
    elif base_class.startswith('ShiftInst'):
        header_code(exec_template_shift)
    else:
        header_code(exec_templates[base_class_base])

    if not types or isinstance(types, str):
        # Just a single type
        constructor = constructor_prefix + class_name
        decoder_code(decode_nodt_template)
    else:
        # multiple types, need at least one level of decode
        if brig_opcode == 'Class':
            decoder_code(decode_case_prolog_class_inst)
        else:
            decoder_code(decode_case_prolog)
        if not type2_info:
            if not is_store:
                # single list of types, to basic one-level decode
                for type_name in types:
                    full_class_name = '%s<%s>' % (class_name, type_name.upper())
                    constructor = constructor_prefix + full_class_name
                    decoder_code(decode_case_entry)
            else:
                # single list of types, to basic one-level decode
                for type_name in types:
                    decoder_code(decode_store_prolog)
                    type_size = int(re.findall(r'[0-9]+', type_name)[0])
                    src_size = 32
                    type_type = type_name[0]
                    full_class_name = '%s<%s,%s>' % (class_name, \
                                                     type_name.upper(), \
                                                     '%s%d' % \
                                                     (type_type.upper(), \
                                                     type_size))
                    constructor = constructor_prefix + full_class_name
                    decoder_code(decode_store_case_entry)
                    decoder_code(decode_store_case_epilog)
        else:
            # need secondary type switch (convert, compare)
            # unpack extra info on second switch
            (type2_field, types2) = type2_info
            brig_type = 'Inst%s' % brig_opcode
            for type_name in types:
                decoder_code(decode_case2_prolog)
                fmt = '%s<%s,%%s>' % (class_name, type_name.upper())
                for type2_name in types2:
                    full_class_name = fmt % type2_name.upper()
                    constructor = constructor_prefix + full_class_name
                    decoder_code(decode_case2_entry)

                decoder_code(decode_case2_epilog)

        decoder_code(decode_case_epilog)

###############
#
# Generate instructions
#
###############

# handy abbreviations for common sets of types

# arithmetic ops are typically defined only on 32- and 64-bit sizes
arith_int_types = ('S32', 'U32', 'S64', 'U64')
arith_float_types = ('F32', 'F64')
arith_types = arith_int_types + arith_float_types

bit_types = ('B1', 'B32', 'B64')

all_int_types = ('S8', 'U8', 'S16', 'U16') + arith_int_types

# I think you might be able to do 'f16' memory ops too, but we'll
# ignore them for now.
mem_types = all_int_types + arith_float_types
mem_atom_types = all_int_types + ('B32', 'B64')

##### Arithmetic & logical operations
gen('Add', arith_types, 'src0 + src1')
gen('Sub', arith_types, 'src0 - src1')
gen('Mul', arith_types, 'src0 * src1')
gen('Div', arith_types, 'src0 / src1')
gen('Min', arith_types, 'std::min(src0, src1)')
gen('Max', arith_types, 'std::max(src0, src1)')
gen('Gcnmin', arith_types, 'std::min(src0, src1)')

gen('CopySign', arith_float_types,
    'src1 < 0 ? -std::abs(src0) : std::abs(src0)')
gen('Sqrt', arith_float_types, 'sqrt(src0)')
gen('Floor', arith_float_types, 'floor(src0)')

# "fast" sqrt... same as slow for us
gen('Nsqrt', arith_float_types, 'sqrt(src0)')
gen('Nrsqrt', arith_float_types, '1.0/sqrt(src0)')
gen('Nrcp', arith_float_types, '1.0/src0')
gen('Fract', arith_float_types,
    '(src0 >= 0.0)?(src0-floor(src0)):(floor(src0)-src0)')

gen('Ncos', arith_float_types, 'cos(src0)');
gen('Nsin', arith_float_types, 'sin(src0)');

gen('And', bit_types, 'src0 & src1')
gen('Or', bit_types,  'src0 | src1')
gen('Xor', bit_types, 'src0 ^ src1')

gen('Bitselect', bit_types, '(src1 & src0) | (src2 & ~src0)')
gen('Firstbit',bit_types, 'firstbit(src0)')
gen('Popcount', ('B32', 'B64'), '__builtin_popcount(src0)')

gen('Shl', arith_int_types, 'src0 << (unsigned)src1', 'ShiftInst')
gen('Shr', arith_int_types, 'src0 >> (unsigned)src1', 'ShiftInst')

# gen('Mul_hi', types=('s32','u32', '??'))
# gen('Mul24', types=('s32','u32', '??'))
gen('Rem', arith_int_types, 'src0 - ((src0 / src1) * src1)')

gen('Abs', arith_types, 'std::abs(src0)')
gen('Neg', arith_types, '-src0')

gen('Mov', bit_types, 'src0')
gen('Not', bit_types, 'heynot(src0)')

# mad and fma differ only in rounding behavior, which we don't emulate
# also there's an integer form of mad, but not of fma
gen('Mad', arith_types, 'src0 * src1 + src2')
gen('Fma', arith_float_types, 'src0 * src1 + src2')

#native floating point operations
gen('Nfma', arith_float_types, 'src0 * src1 + src2')

gen('Cmov', bit_types, 'src0 ? src1 : src2', 'CmovInst')
gen('BitAlign', bit_types, '(src0 << src2)|(src1 >> (32 - src2))')
gen('ByteAlign', bit_types, '(src0 << 8 * src2)|(src1 >> (32 - 8 * src2))')

# see base/bitfield.hh
gen('BitExtract', arith_int_types, 'bits(src0, src1, src1 + src2 - 1)',
    'ExtractInsertInst')

gen('BitInsert', arith_int_types, 'insertBits(dest, src1, src2, src0)',
    'ExtractInsertInst')

##### Compare
gen('Cmp', ('B1', 'S32', 'U32', 'F32'), 'compare(src0, src1, this->cmpOp)',
    'CmpInst', ('sourceType', arith_types + bit_types))
gen('Class', arith_float_types, 'fpclassify(src0,src1)','ClassInst')

##### Conversion

# Conversion operations are only defined on B1, not B32 or B64
cvt_types = ('B1',) + mem_types

gen('Cvt', cvt_types, 'src0', 'CvtInst', ('sourceType', cvt_types))


##### Load & Store
gen('Lda', mem_types, base_class = 'LdInst', constructor_prefix='decode')
gen('Ld', mem_types, base_class = 'LdInst', constructor_prefix='decode')
gen('St', mem_types, base_class = 'StInst', constructor_prefix='decode',
    is_store=True)
gen('Atomic', mem_atom_types, base_class='StInst', constructor_prefix='decode')
gen('AtomicNoRet', mem_atom_types, base_class='StInst',
    constructor_prefix='decode')

gen('Cbr', base_class = 'LdInst', constructor_prefix='decode')
gen('Br', base_class = 'LdInst', constructor_prefix='decode')

##### Special operations
def gen_special(brig_opcode, expr, dest_type='U32'):
    num_srcs = num_src_operands(expr)
    if num_srcs == 0:
        base_class = 'SpecialInstNoSrc<%s>' % dest_type
    elif num_srcs == 1:
        base_class = 'SpecialInst1Src<%s>' % dest_type
    else:
        assert false

    gen(brig_opcode, None, expr, base_class)

gen_special('WorkItemId', 'w->workitemid[src0][lane]')
gen_special('WorkItemAbsId',
    'w->workitemid[src0][lane] + (w->workgroupid[src0] * w->workgroupsz[src0])')
gen_special('WorkGroupId', 'w->workgroupid[src0]')
gen_special('WorkGroupSize', 'w->workgroupsz[src0]')
gen_special('CurrentWorkGroupSize', 'w->workgroupsz[src0]')
gen_special('GridSize', 'w->gridsz[src0]')
gen_special('GridGroups',
    'divCeil(w->gridsz[src0],w->workgroupsz[src0])')
gen_special('LaneId', 'lane')
gen_special('WaveId', 'w->dynwaveid')
gen_special('Clock', 'w->computeUnit->shader->tick_cnt', 'U64')

# gen_special('CU'', ')

gen('Ret', base_class='SpecialInstNoSrcNoDest')
gen('Barrier', base_class='SpecialInstNoSrcNoDest')
gen('MemFence', base_class='SpecialInstNoSrcNoDest')

# Map magic instructions to the BrigSyscall opcode
# Magic instructions are defined in magic.hh
#
# In the future, real HSA kernel system calls can be implemented and coexist
# with magic instructions.
gen('Call', base_class='SpecialInstNoSrcNoDest')

###############
#
# Generate file epilogs
#
###############
header_code.dedent()
header_code('''
} // namespace HsailISA
''')

# close off main decode switch
decoder_code.dedent()
decoder_code.dedent()
decoder_code('''
          default: fatal("unrecognized Brig opcode %d\\n", ib->opcode);
        } // end switch(ib->opcode)
    } // end decode()
} // namespace HsailISA
''')

exec_code.dedent()
exec_code('''
} // namespace HsailISA
''')

###############
#
# Output accumulated code to files
#
###############
header_code.write(sys.argv[1])
decoder_code.write(sys.argv[2])
exec_code.write(sys.argv[3])
