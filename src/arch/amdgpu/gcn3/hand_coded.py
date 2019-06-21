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

HandCodedExecMethods = {
    'Inst_SOPP__S_NOP' : [
        'ExecNOP(gpuDynInst, instData.SIMM16);'
    ],
    'Inst_VOP1__V_NOP' : [
        'ExecNOP(gpuDynInst, 1);'
    ],
    'Inst_VOP3__V_NOP' : [
        'ExecNOP(gpuDynInst, 1);'
    ],
    'Inst_DS__DS_NOP' : [
        'ExecNOP(gpuDynInst, 1);'
    ],
    'Inst_SOPP__S_ENDPGM' : [
        'ExecEndPgm(gpuDynInst);'
    ],
    'Inst_SOPP__S_ENDPGM_SAVED' : [
        'ExecEndPgmSaved(gpuDynInst);'
    ],
    'Inst_VOP2__V_MUL_HI_I32_I24' : [
        'exec = readSpecialReg<SregU64>(gpuDynInst, REG_EXEC);',
        'vdst = readVectorReg<VregI32>(gpuDynInst, instData.VDST);',
        'src_0 = readSrcReg<VregI32>(gpuDynInst, instData.SRC0);',
        'src_1 = readVectorReg<VregI32>(gpuDynInst, instData.VSRC1);',
        'for (unsigned t = 0; exec != 0; t++, exec >>= 1) {',
        '    if ((exec & 1) != 0) {',
        '        int64_t s0 = (int64_t)src_0[t](23, 0);',
        '        int64_t s1 = (int64_t)src_1[t](23, 0);',
        '        vdst[t] = (int32_t)((s0 * s1) >> 32);',
        '    }',
        '}',
        'writeVectorReg<VregI32>(gpuDynInst, instData.VDST, vdst);'
    ],
    'Inst_VOP2__V_MUL_HI_U32_U24' : [
        'exec = readSpecialReg<SregU64>(gpuDynInst, REG_EXEC);',
        'vdst = readVectorReg<VregI32>(gpuDynInst, instData.VDST);',
        'src_0 = readSrcReg<VregU32>(gpuDynInst, instData.SRC0);',
        'src_1 = readVectorReg<VregU32>(gpuDynInst, instData.VSRC1);',
        'for (unsigned t = 0; exec != 0; t++, exec >>= 1) {',
        '    if ((exec & 1) != 0) {',
        '        uint64_t s0 = (uint64_t)src_0[t](23, 0);',
        '        uint64_t s1 = (uint64_t)src_1[t](23, 0);',
        '        vdst[t] = (uint32_t)((s0 * s1) >> 32);',
        '    }',
        '}',
        'writeVectorReg<VregI32>(gpuDynInst, instData.VDST, vdst);'
    ],

    # stuff below here should eventually get fixed in the parser
    'Inst_SOP1__S_MOVRELS_B64' : [
        'm0 = readSpecialReg<SregU32>(gpuDynInst, REG_M0);',
        'ssrc = readScalarReg<SregU64>(gpuDynInst, instData.SSRC0 + m0);',
        'sdst = ssrc;',
        'writeScalarReg<SregU64>(gpuDynInst, instData.SDST, sdst);'
    ],
    'Inst_SOP1__S_MOVRELD_B64' : [
        'm0 = readSpecialReg<SregU32>(gpuDynInst, REG_M0);',
        'ssrc = readScalarReg<SregU64>(gpuDynInst, instData.SSRC0);',
        'sdst = ssrc;',
        'writeScalarReg<SregU64>(gpuDynInst, instData.SDST + m0, sdst);'
    ],
    'Inst_SOPC__S_SET_GPR_IDX_ON' : [
        'ssrc_0 = readScalarReg<SregU32>(gpuDynInst, instData.SSRC0);',
        'simm4 = instData.SSRC1;',
        'm0(7, 0) = ssrc_0(7, 0);',
        'm0(15, 12) = (uint32_t)simm4;',
        'writeSpecialReg<SregU32>(gpuDynInst, REG_M0, m0);'
    ],
    'Inst_VOP2__V_MADMK_F32' : [
        'exec = readSpecialReg<SregU64>(gpuDynInst, REG_EXEC);',
        'vdst = readVectorReg<VregF32>(gpuDynInst, instData.VDST);',
        'src_0 = readSrcReg<VregF32>(gpuDynInst, instData.SRC0);',
        'k = extData.imm_f32;',
        'src_2 = readVectorReg<VregF32>(gpuDynInst, instData.VSRC1);',
        'for (unsigned t = 0; exec != 0; t++, exec >>= 1) {',
        '    if ((exec & 1) != 0) {',
        '        vdst[t] = src_0[t] * k + src_2[t];',
        '    }',
        '}',
        'writeVectorReg<VregF32>(gpuDynInst, instData.VDST, vdst);'
    ],
    'Inst_VOP2__V_MADAK_F32' : [
        'exec = readSpecialReg<SregU64>(gpuDynInst, REG_EXEC);',
        'vdst = readVectorReg<VregF32>(gpuDynInst, instData.VDST);',
        'src_0 = readSrcReg<VregF32>(gpuDynInst, instData.SRC0);',
        'src_1 = readVectorReg<VregF32>(gpuDynInst, instData.VSRC1);',
        'k = extData.imm_f32;',
        'for (unsigned t = 0; exec != 0; t++, exec >>= 1) {',
        '    if ((exec & 1) != 0) {',
        '        vdst[t] = src_0[t] * src_1[t] + k;',
        '    }',
        '}',
        'writeVectorReg<VregF32>(gpuDynInst, instData.VDST, vdst);'
    ],
    'Inst_VOP2__V_MADMK_F16' : [
        'exec = readSpecialReg<SregU64>(gpuDynInst, REG_EXEC);',
        'vdst = readVectorReg<VregF16>(gpuDynInst, instData.VDST);',
        'src_0 = readSrcReg<VregF16>(gpuDynInst, instData.SRC0);',
        'k = extData.imm_f32;',
        'src_2 = readVectorReg<VregF16>(gpuDynInst, instData.VSRC1);',
        'for (unsigned t = 0; exec != 0; t++, exec >>= 1) {',
        '    if ((exec & 1) != 0) {',
        '        vdst[t] = src_0[t] * k + src_2[t];',
        '    }',
        '}',
        'writeVectorReg<VregF16>(gpuDynInst, instData.VDST, vdst);'
    ],
    'Inst_VOP2__V_MADAK_F16' : [
        'exec = readSpecialReg<SregU64>(gpuDynInst, REG_EXEC);',
        'vdst = readVectorReg<VregF16>(gpuDynInst, instData.VDST);',
        'src_0 = readSrcReg<VregF16>(gpuDynInst, instData.SRC0);',
        'src_1 = readVectorReg<VregF16>(gpuDynInst, instData.VSRC1);',
        'k = extData.imm_f32;',
        'for (unsigned t = 0; exec != 0; t++, exec >>= 1) {',
        '    if ((exec & 1) != 0) {',
        '        vdst[t] = src_0[t] * src_1[t] + k;',
        '    }',
        '}',
        'writeVectorReg<VregF16>(gpuDynInst, instData.VDST, vdst);'
    ],
    'Inst_VOP3__V_MUL_HI_I32_I24' : [
        'exec = readSpecialReg<SregU64>(gpuDynInst, REG_EXEC);',
        'vdst = readVectorReg<VregI32>(gpuDynInst, instData.VDST);',
        'src_0 = readSrcReg<VregI32>(gpuDynInst, extData.SRC0);',
        'src_1 = readSrcReg<VregI32>(gpuDynInst, extData.SRC1);',
        'for (unsigned t = 0; exec != 0; t++, exec >>= 1) {',
        '    if ((exec & 1) != 0) {',
        '        int64_t s0 = (int64_t)(int32_t)src_0[t](23, 0);',
        '        int64_t s1 = (int64_t)(int32_t)src_1[t](23, 0);',
        '        vdst[t] = (int32_t)((s0 * s1) >> 32);',
        '    }',
        '}',
        'writeVectorReg<VregI32>(gpuDynInst, instData.VDST, vdst);'
    ],
    'Inst_VOP3__V_MUL_HI_U32_U24' : [
        'exec = readSpecialReg<SregU64>(gpuDynInst, REG_EXEC);',
        'vdst = readVectorReg<VregI32>(gpuDynInst, instData.VDST);',
        'src_0 = readSrcReg<VregU32>(gpuDynInst, extData.SRC0);',
        'src_1 = readSrcReg<VregU32>(gpuDynInst, extData.SRC1);',
        'for (unsigned t = 0; exec != 0; t++, exec >>= 1) {',
        '    if ((exec & 1) != 0) {',
        '        uint64_t s0 = (uint64_t)(uint32_t)src_0[t](23, 0);',
        '        uint64_t s1 = (uint64_t)(uint32_t)src_1[t](23, 0);',
        '        vdst[t] = (int32_t)((s0 * s1) >> 32);',
        '    }',
        '}',
        'writeVectorReg<VregI32>(gpuDynInst, instData.VDST, vdst);'
    ],
    'Inst_VOP3__V_MAD_U64_U32' : [
        'exec = readSpecialReg<SregU64>(gpuDynInst, REG_EXEC);',
        'vdst = readVectorReg<VregU64>(gpuDynInst, instData.VDST);',
        'src_0 = readSrcReg<VregU32>(gpuDynInst, extData.SRC0);',
        'src_1 = readSrcReg<VregU32>(gpuDynInst, extData.SRC1);',
        'src_2 = readSrcReg<VregU64>(gpuDynInst, extData.SRC2);',
        'for (unsigned t = 0; exec != 0; t++, exec >>= 1) {',
        '    if ((exec & 1) != 0) {',
        '        vcc(t) = muladd(vdst[t], src_0[t], src_1[t], src_2[t]);',
        '    }',
        '}',
        'writeSpecialReg<SregU64>(gpuDynInst, REG_VCC, vcc);',
        'writeVectorReg<VregU64>(gpuDynInst, instData.VDST, vdst);'
    ],
    'Inst_VOP3__V_MAD_I64_I32' : [
        'exec = readSpecialReg<SregU64>(gpuDynInst, REG_EXEC);',
        'vdst = readVectorReg<VregI64>(gpuDynInst, instData.VDST);',
        'src_0 = readSrcReg<VregI32>(gpuDynInst, extData.SRC0);',
        'src_1 = readSrcReg<VregI32>(gpuDynInst, extData.SRC1);',
        'src_2 = readSrcReg<VregI64>(gpuDynInst, extData.SRC2);',
        'for (unsigned t = 0; exec != 0; t++, exec >>= 1) {',
        '    if ((exec & 1) != 0) {',
        '        vcc(t) = muladd(vdst[t], src_0[t], src_1[t], src_2[t]);',
        '    }',
        '}',
        'writeSpecialReg<SregU64>(gpuDynInst, REG_VCC, vcc);',
        'writeVectorReg<VregI64>(gpuDynInst, instData.VDST, vdst);'
    ],
    'Inst_DS__DS_WRITE_B96' : [
        'exec = readSpecialReg<SregU64>(gpuDynInst, REG_EXEC);',
        'vgpr_a = readVectorReg<VregU32>(gpuDynInst, extData.ADDR);',
        'calculateAddr<VregU32>(gpuDynInst, vgpr_a, 8, 0);',
        'calculateAddr<VregU32>(gpuDynInst, vgpr_a, 4, 0);',
        'calculateAddr<VregU32>(gpuDynInst, vgpr_a, 0, 0);',
        'vgpr_d0 = readVectorReg<VregU96>(gpuDynInst, extData.DATA0);',
        'for (unsigned t = 0; exec != 0; t++, exec >>= 1) {',
        '    if ((exec & 1) != 0) {',
        '        vmem_0[t] = vgpr_d0[t].getDword(2);',
        '        vmem_1[t] = vgpr_d0[t].getDword(1);',
        '        vmem_2[t] = vgpr_d0[t].getDword(0);',
        '    }',
        '}',
        'writeMem<VregU32, VregU32>(gpuDynInst, vgpr_a, 0, vmem_0);',
        'writeMem<VregU32, VregU32>(gpuDynInst, vgpr_a, 0, vmem_1);',
        'writeMem<VregU32, VregU32>(gpuDynInst, vgpr_a, 0, vmem_2);'
    ],
    'Inst_DS__DS_WRITE_B128' : [
        'exec = readSpecialReg<SregU64>(gpuDynInst, REG_EXEC);',
        'vgpr_a = readVectorReg<VregU32>(gpuDynInst, extData.ADDR);',
        'calculateAddr<VregU32>(gpuDynInst, vgpr_a, 12, 0);',
        'calculateAddr<VregU32>(gpuDynInst, vgpr_a, 8, 0);',
        'calculateAddr<VregU32>(gpuDynInst, vgpr_a, 4, 0);',
        'calculateAddr<VregU32>(gpuDynInst, vgpr_a, 0, 0);',
        'vgpr_d0 = readVectorReg<VregU128>(gpuDynInst, extData.DATA0);',
        'for (unsigned t = 0; exec != 0; t++, exec >>= 1) {',
        '    if ((exec & 1) != 0) {',
        '        vmem_0[t] = vgpr_d0[t].getDword(3);',
        '        vmem_1[t] = vgpr_d0[t].getDword(2);',
        '        vmem_2[t] = vgpr_d0[t].getDword(1);',
        '        vmem_3[t] = vgpr_d0[t].getDword(0);',
        '    }',
        '}',
        'writeMem<VregU32, VregU32>(gpuDynInst, vgpr_a, 0, vmem_0);',
        'writeMem<VregU32, VregU32>(gpuDynInst, vgpr_a, 0, vmem_1);',
        'writeMem<VregU32, VregU32>(gpuDynInst, vgpr_a, 0, vmem_2);',
        'writeMem<VregU32, VregU32>(gpuDynInst, vgpr_a, 0, vmem_3);'
    ],
    'Inst_SOPP__S_WAITCNT' : [
        'int vm_cnt = 0;',
        'int lgkm_cnt = 0;',
        'vm_cnt = bits<uint16_t>(instData.SIMM16, 3, 0);',
        'lgkm_cnt = bits<uint16_t>(instData.SIMM16, 11, 8);',
        'gpuDynInst->wavefront()->setWaitCnts(vm_cnt, lgkm_cnt);'
    ]
}

HandCodedDecl = {
    'Inst_VOP2__V_MUL_HI_I32_I24' : [
        'SregU64 exec;',
        'VregI32 vdst;',
        'VregI32 src_0;',
        'VregI32 src_1;'
    ],
    'Inst_VOP2__V_MUL_HI_U32_U24' : [
        'SregU64 exec;',
        'VregI32 vdst;',
        'VregU32 src_0;',
        'VregU32 src_1;'
    ],
    'Inst_VOP3__V_MUL_HI_I32_I24' : [
        'SregU64 exec;',
        'VregI32 vdst;',
        'VregI32 src_0;',
        'VregI32 src_1;'
    ],
    'Inst_VOP3__V_MUL_HI_U32_U24' : [
        'SregU64 exec;',
        'VregI32 vdst;',
        'VregU32 src_0;',
        'VregU32 src_1;'
    ],
    'Inst_SOPC__S_SET_GPR_IDX_ON' : [
        'SregU32 m0;',
        'SregU32 ssrc_0;',
        'SregU16 simm4;'
    ],
    'Inst_SOP1__S_MOVRELS_B64' : [
        'SregU64 sdst;',
        'SregU32 m0;',
        'SregU64 ssrc;'
    ],
    'Inst_SOP1__S_MOVRELD_B64' : [
        'SregU64 sdst;',
        'SregU32 m0;',
        'SregU64 ssrc;'
    ],
    'Inst_VOP2__V_MADMK_F32' : [
        'SregU64 exec;',
        'VregF32 vdst;',
        'VregF32 src_0;',
        'SregF32 k;',
        'VregF32 src_2;'
    ],
    'Inst_VOP2__V_MADAK_F32' : [
        'SregU64 exec;',
        'VregF32 vdst;',
        'VregF32 src_0;',
        'VregF32 src_1;',
        'SregF32 k;'
    ],
    'Inst_VOP2__V_MADMK_F16' : [
        'SregU64 exec;',
        'VregF16 vdst;',
        'VregF16 src_0;',
        'SregF16 k;',
        'VregF16 src_2;'
    ],
    'Inst_VOP2__V_MADAK_F16' : [
        'SregU64 exec;',
        'VregF16 vdst;',
        'VregF16 src_0;',
        'VregF16 src_1;',
        'SregF16 k;'
    ],
    'Inst_VOP3__V_MAD_U64_U32' : [
        'SregU64 exec;',
        'SregU64 vcc;',
        'VregU64 vdst;',
        'VregU32 src_0;',
        'VregU32 src_1;',
        'VregU64 src_2;'
    ],
    'Inst_VOP3__V_MAD_I64_I32' : [
        'SregU64 exec;',
        'SregU64 vcc;',
        'VregI64 vdst;',
        'VregI32 src_0;',
        'VregI32 src_1;',
        'VregI64 src_2;'
    ],
    'Inst_DS__DS_WRITE_B96' : [
        'SregU64 exec;',
        'VregU32 vmem_0;',
        'VregU32 vgpr_a;',
        'VregU32 vmem_1;',
        'VregU32 vmem_2;',
        'VregU96 vgpr_d0;'
    ],
    'Inst_DS__DS_WRITE_B128' : [
        'SregU64 exec;',
        'VregU32 vmem_0;',
        'VregU32 vgpr_a;',
        'VregU32 vmem_1;',
        'VregU32 vmem_2;',
        'VregU32 vmem_3;',
        'VregU128 vgpr_d0;'
    ]
}

HandCodedPrototypes = [
    ['void', 'ExecNOP', 'GPUDynInstPtr', 'uint32_t'],
    ['void', 'ExecEndPgm', 'GPUDynInstPtr'],
    ['void', 'ExecEndPgmSaved', 'GPUDynInstPtr'],
    ['VregI64&', 'getSRC_SIMPLE_I64', 'GPUDynInstPtr', 'uint32_t']
]

HandCodedStaticInstMethods = [
    ['SregI32&', 'ViGPUStaticInst', 'getSSrcLiteral_I32', [], [],
      [
          'static SregI32 sreg;',
          'sreg = 0;',
          'return sreg;'
      ]
    ],
    [ 'SregU32 &', 'ViGPUStaticInst', 'getSSrcLiteral_U32', [], [],
      [
          'static SregU32 sreg;',
          'sreg = 0;',
          'return sreg;'
      ]
    ],
    [ 'VregI32 &', 'ViGPUStaticInst', 'getVSrcLiteral_I32', [], [],
      [
          'static VregI32 vreg;',
          'vreg = getSSrcLiteral_I32();',
          'return vreg;'
      ]
    ],
    [ 'VregU32 &', 'ViGPUStaticInst', 'getVSrcLiteral_U32', [], [],
      [
          'static VregU32 vreg;',
          'vreg = getSSrcLiteral_U32();',
          'return vreg;'
      ]
    ]
]

HandCodedInstProlog = {
    'getSSRC_I32' : [
        'if (arg2 == REG_SRC_LITERAL)',
        '    return getSSrcLiteral_I32();'
    ],
    'getSSRC_U32' : [
        'if (arg2 == REG_SRC_LITERAL)',
        '    return getSSrcLiteral_U32();'
    ],
    'getSRC_I32' : [
        'if (arg2 == REG_SRC_LITERAL)',
        '    return getVSrcLiteral_I32();'
    ],
    'getSRC_U32' : [
        'if (arg2 == REG_SRC_LITERAL)',
        '    return getVSrcLiteral_U32();'
    ]
}
