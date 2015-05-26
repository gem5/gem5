/*
 * Copyright (c) 2010-2015 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2009 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Gabe Black
 *          Giacomo Gabrielli
 */
#ifndef __ARCH_ARM_MISCREGS_HH__
#define __ARCH_ARM_MISCREGS_HH__

#include <bitset>

#include "base/bitunion.hh"
#include "base/compiler.hh"

class ThreadContext;


namespace ArmISA
{
    enum MiscRegIndex {
        MISCREG_CPSR = 0,               //   0
        MISCREG_SPSR,                   //   1
        MISCREG_SPSR_FIQ,               //   2
        MISCREG_SPSR_IRQ,               //   3
        MISCREG_SPSR_SVC,               //   4
        MISCREG_SPSR_MON,               //   5
        MISCREG_SPSR_ABT,               //   6
        MISCREG_SPSR_HYP,               //   7
        MISCREG_SPSR_UND,               //   8
        MISCREG_ELR_HYP,                //   9
        MISCREG_FPSID,                  //  10
        MISCREG_FPSCR,                  //  11
        MISCREG_MVFR1,                  //  12
        MISCREG_MVFR0,                  //  13
        MISCREG_FPEXC,                  //  14

        // Helper registers
        MISCREG_CPSR_MODE,              //  15
        MISCREG_CPSR_Q,                 //  16
        MISCREG_FPSCR_EXC,              //  17
        MISCREG_FPSCR_QC,               //  18
        MISCREG_LOCKADDR,               //  19
        MISCREG_LOCKFLAG,               //  20
        MISCREG_PRRR_MAIR0,             //  21
        MISCREG_PRRR_MAIR0_NS,          //  22
        MISCREG_PRRR_MAIR0_S,           //  23
        MISCREG_NMRR_MAIR1,             //  24
        MISCREG_NMRR_MAIR1_NS,          //  25
        MISCREG_NMRR_MAIR1_S,           //  26
        MISCREG_PMXEVTYPER_PMCCFILTR,   //  27
        MISCREG_SCTLR_RST,              //  28
        MISCREG_SEV_MAILBOX,            //  29

        // AArch32 CP14 registers (debug/trace/ThumbEE/Jazelle control)
        MISCREG_DBGDIDR,                //  30
        MISCREG_DBGDSCRint,             //  31
        MISCREG_DBGDCCINT,              //  32
        MISCREG_DBGDTRTXint,            //  33
        MISCREG_DBGDTRRXint,            //  34
        MISCREG_DBGWFAR,                //  35
        MISCREG_DBGVCR,                 //  36
        MISCREG_DBGDTRRXext,            //  37
        MISCREG_DBGDSCRext,             //  38
        MISCREG_DBGDTRTXext,            //  39
        MISCREG_DBGOSECCR,              //  40
        MISCREG_DBGBVR0,                //  41
        MISCREG_DBGBVR1,                //  42
        MISCREG_DBGBVR2,                //  43
        MISCREG_DBGBVR3,                //  44
        MISCREG_DBGBVR4,                //  45
        MISCREG_DBGBVR5,                //  46
        MISCREG_DBGBCR0,                //  47
        MISCREG_DBGBCR1,                //  48
        MISCREG_DBGBCR2,                //  49
        MISCREG_DBGBCR3,                //  50
        MISCREG_DBGBCR4,                //  51
        MISCREG_DBGBCR5,                //  52
        MISCREG_DBGWVR0,                //  53
        MISCREG_DBGWVR1,                //  54
        MISCREG_DBGWVR2,                //  55
        MISCREG_DBGWVR3,                //  56
        MISCREG_DBGWCR0,                //  57
        MISCREG_DBGWCR1,                //  58
        MISCREG_DBGWCR2,                //  59
        MISCREG_DBGWCR3,                //  60
        MISCREG_DBGDRAR,                //  61
        MISCREG_DBGBXVR4,               //  62
        MISCREG_DBGBXVR5,               //  63
        MISCREG_DBGOSLAR,               //  64
        MISCREG_DBGOSLSR,               //  65
        MISCREG_DBGOSDLR,               //  66
        MISCREG_DBGPRCR,                //  67
        MISCREG_DBGDSAR,                //  68
        MISCREG_DBGCLAIMSET,            //  69
        MISCREG_DBGCLAIMCLR,            //  70
        MISCREG_DBGAUTHSTATUS,          //  71
        MISCREG_DBGDEVID2,              //  72
        MISCREG_DBGDEVID1,              //  73
        MISCREG_DBGDEVID0,              //  74
        MISCREG_TEECR,                  //  75
        MISCREG_JIDR,                   //  76
        MISCREG_TEEHBR,                 //  77
        MISCREG_JOSCR,                  //  78
        MISCREG_JMCR,                   //  79

        // AArch32 CP15 registers (system control)
        MISCREG_MIDR,                   //  80
        MISCREG_CTR,                    //  81
        MISCREG_TCMTR,                  //  82
        MISCREG_TLBTR,                  //  83
        MISCREG_MPIDR,                  //  84
        MISCREG_REVIDR,                 //  85
        MISCREG_ID_PFR0,                //  86
        MISCREG_ID_PFR1,                //  87
        MISCREG_ID_DFR0,                //  88
        MISCREG_ID_AFR0,                //  89
        MISCREG_ID_MMFR0,               //  90
        MISCREG_ID_MMFR1,               //  91
        MISCREG_ID_MMFR2,               //  92
        MISCREG_ID_MMFR3,               //  93
        MISCREG_ID_ISAR0,               //  94
        MISCREG_ID_ISAR1,               //  95
        MISCREG_ID_ISAR2,               //  96
        MISCREG_ID_ISAR3,               //  97
        MISCREG_ID_ISAR4,               //  98
        MISCREG_ID_ISAR5,               //  99
        MISCREG_CCSIDR,                 // 100
        MISCREG_CLIDR,                  // 101
        MISCREG_AIDR,                   // 102
        MISCREG_CSSELR,                 // 103
        MISCREG_CSSELR_NS,              // 104
        MISCREG_CSSELR_S,               // 105
        MISCREG_VPIDR,                  // 106
        MISCREG_VMPIDR,                 // 107
        MISCREG_SCTLR,                  // 108
        MISCREG_SCTLR_NS,               // 109
        MISCREG_SCTLR_S,                // 110
        MISCREG_ACTLR,                  // 111
        MISCREG_ACTLR_NS,               // 112
        MISCREG_ACTLR_S,                // 113
        MISCREG_CPACR,                  // 114
        MISCREG_SCR,                    // 115
        MISCREG_SDER,                   // 116
        MISCREG_NSACR,                  // 117
        MISCREG_HSCTLR,                 // 118
        MISCREG_HACTLR,                 // 119
        MISCREG_HCR,                    // 120
        MISCREG_HDCR,                   // 121
        MISCREG_HCPTR,                  // 122
        MISCREG_HSTR,                   // 123
        MISCREG_HACR,                   // 124
        MISCREG_TTBR0,                  // 125
        MISCREG_TTBR0_NS,               // 126
        MISCREG_TTBR0_S,                // 127
        MISCREG_TTBR1,                  // 128
        MISCREG_TTBR1_NS,               // 129
        MISCREG_TTBR1_S,                // 130
        MISCREG_TTBCR,                  // 131
        MISCREG_TTBCR_NS,               // 132
        MISCREG_TTBCR_S,                // 133
        MISCREG_HTCR,                   // 134
        MISCREG_VTCR,                   // 135
        MISCREG_DACR,                   // 136
        MISCREG_DACR_NS,                // 137
        MISCREG_DACR_S,                 // 138
        MISCREG_DFSR,                   // 139
        MISCREG_DFSR_NS,                // 140
        MISCREG_DFSR_S,                 // 141
        MISCREG_IFSR,                   // 142
        MISCREG_IFSR_NS,                // 143
        MISCREG_IFSR_S,                 // 144
        MISCREG_ADFSR,                  // 145
        MISCREG_ADFSR_NS,               // 146
        MISCREG_ADFSR_S,                // 147
        MISCREG_AIFSR,                  // 148
        MISCREG_AIFSR_NS,               // 149
        MISCREG_AIFSR_S,                // 150
        MISCREG_HADFSR,                 // 151
        MISCREG_HAIFSR,                 // 152
        MISCREG_HSR,                    // 153
        MISCREG_DFAR,                   // 154
        MISCREG_DFAR_NS,                // 155
        MISCREG_DFAR_S,                 // 156
        MISCREG_IFAR,                   // 157
        MISCREG_IFAR_NS,                // 158
        MISCREG_IFAR_S,                 // 159
        MISCREG_HDFAR,                  // 160
        MISCREG_HIFAR,                  // 161
        MISCREG_HPFAR,                  // 162
        MISCREG_ICIALLUIS,              // 163
        MISCREG_BPIALLIS,               // 164
        MISCREG_PAR,                    // 165
        MISCREG_PAR_NS,                 // 166
        MISCREG_PAR_S,                  // 167
        MISCREG_ICIALLU,                // 168
        MISCREG_ICIMVAU,                // 169
        MISCREG_CP15ISB,                // 170
        MISCREG_BPIALL,                 // 171
        MISCREG_BPIMVA,                 // 172
        MISCREG_DCIMVAC,                // 173
        MISCREG_DCISW,                  // 174
        MISCREG_ATS1CPR,                // 175
        MISCREG_ATS1CPW,                // 176
        MISCREG_ATS1CUR,                // 177
        MISCREG_ATS1CUW,                // 178
        MISCREG_ATS12NSOPR,             // 179
        MISCREG_ATS12NSOPW,             // 180
        MISCREG_ATS12NSOUR,             // 181
        MISCREG_ATS12NSOUW,             // 182
        MISCREG_DCCMVAC,                // 183
        MISCREG_DCCSW,                  // 184
        MISCREG_CP15DSB,                // 185
        MISCREG_CP15DMB,                // 186
        MISCREG_DCCMVAU,                // 187
        MISCREG_DCCIMVAC,               // 188
        MISCREG_DCCISW,                 // 189
        MISCREG_ATS1HR,                 // 190
        MISCREG_ATS1HW,                 // 191
        MISCREG_TLBIALLIS,              // 192
        MISCREG_TLBIMVAIS,              // 193
        MISCREG_TLBIASIDIS,             // 194
        MISCREG_TLBIMVAAIS,             // 195
        MISCREG_TLBIMVALIS,             // 196
        MISCREG_TLBIMVAALIS,            // 197
        MISCREG_ITLBIALL,               // 198
        MISCREG_ITLBIMVA,               // 199
        MISCREG_ITLBIASID,              // 200
        MISCREG_DTLBIALL,               // 201
        MISCREG_DTLBIMVA,               // 202
        MISCREG_DTLBIASID,              // 203
        MISCREG_TLBIALL,                // 204
        MISCREG_TLBIMVA,                // 205
        MISCREG_TLBIASID,               // 206
        MISCREG_TLBIMVAA,               // 207
        MISCREG_TLBIMVAL,               // 208
        MISCREG_TLBIMVAAL,              // 209
        MISCREG_TLBIIPAS2IS,            // 210
        MISCREG_TLBIIPAS2LIS,           // 211
        MISCREG_TLBIALLHIS,             // 212
        MISCREG_TLBIMVAHIS,             // 213
        MISCREG_TLBIALLNSNHIS,          // 214
        MISCREG_TLBIMVALHIS,            // 215
        MISCREG_TLBIIPAS2,              // 216
        MISCREG_TLBIIPAS2L,             // 217
        MISCREG_TLBIALLH,               // 218
        MISCREG_TLBIMVAH,               // 219
        MISCREG_TLBIALLNSNH,            // 220
        MISCREG_TLBIMVALH,              // 221
        MISCREG_PMCR,                   // 222
        MISCREG_PMCNTENSET,             // 223
        MISCREG_PMCNTENCLR,             // 224
        MISCREG_PMOVSR,                 // 225
        MISCREG_PMSWINC,                // 226
        MISCREG_PMSELR,                 // 227
        MISCREG_PMCEID0,                // 228
        MISCREG_PMCEID1,                // 229
        MISCREG_PMCCNTR,                // 230
        MISCREG_PMXEVTYPER,             // 231
        MISCREG_PMCCFILTR,              // 232
        MISCREG_PMXEVCNTR,              // 233
        MISCREG_PMUSERENR,              // 234
        MISCREG_PMINTENSET,             // 235
        MISCREG_PMINTENCLR,             // 236
        MISCREG_PMOVSSET,               // 237
        MISCREG_L2CTLR,                 // 238
        MISCREG_L2ECTLR,                // 239
        MISCREG_PRRR,                   // 240
        MISCREG_PRRR_NS,                // 241
        MISCREG_PRRR_S,                 // 242
        MISCREG_MAIR0,                  // 243
        MISCREG_MAIR0_NS,               // 244
        MISCREG_MAIR0_S,                // 245
        MISCREG_NMRR,                   // 246
        MISCREG_NMRR_NS,                // 247
        MISCREG_NMRR_S,                 // 248
        MISCREG_MAIR1,                  // 249
        MISCREG_MAIR1_NS,               // 250
        MISCREG_MAIR1_S,                // 251
        MISCREG_AMAIR0,                 // 252
        MISCREG_AMAIR0_NS,              // 253
        MISCREG_AMAIR0_S,               // 254
        MISCREG_AMAIR1,                 // 255
        MISCREG_AMAIR1_NS,              // 256
        MISCREG_AMAIR1_S,               // 257
        MISCREG_HMAIR0,                 // 258
        MISCREG_HMAIR1,                 // 259
        MISCREG_HAMAIR0,                // 260
        MISCREG_HAMAIR1,                // 261
        MISCREG_VBAR,                   // 262
        MISCREG_VBAR_NS,                // 263
        MISCREG_VBAR_S,                 // 264
        MISCREG_MVBAR,                  // 265
        MISCREG_RMR,                    // 266
        MISCREG_ISR,                    // 267
        MISCREG_HVBAR,                  // 268
        MISCREG_FCSEIDR,                // 269
        MISCREG_CONTEXTIDR,             // 270
        MISCREG_CONTEXTIDR_NS,          // 271
        MISCREG_CONTEXTIDR_S,           // 272
        MISCREG_TPIDRURW,               // 273
        MISCREG_TPIDRURW_NS,            // 274
        MISCREG_TPIDRURW_S,             // 275
        MISCREG_TPIDRURO,               // 276
        MISCREG_TPIDRURO_NS,            // 277
        MISCREG_TPIDRURO_S,             // 278
        MISCREG_TPIDRPRW,               // 279
        MISCREG_TPIDRPRW_NS,            // 280
        MISCREG_TPIDRPRW_S,             // 281
        MISCREG_HTPIDR,                 // 282
        MISCREG_CNTFRQ,                 // 283
        MISCREG_CNTKCTL,                // 284
        MISCREG_CNTP_TVAL,              // 285
        MISCREG_CNTP_TVAL_NS,           // 286
        MISCREG_CNTP_TVAL_S,            // 287
        MISCREG_CNTP_CTL,               // 288
        MISCREG_CNTP_CTL_NS,            // 289
        MISCREG_CNTP_CTL_S,             // 290
        MISCREG_CNTV_TVAL,              // 291
        MISCREG_CNTV_CTL,               // 292
        MISCREG_CNTHCTL,                // 293
        MISCREG_CNTHP_TVAL,             // 294
        MISCREG_CNTHP_CTL,              // 295
        MISCREG_IL1DATA0,               // 296
        MISCREG_IL1DATA1,               // 297
        MISCREG_IL1DATA2,               // 298
        MISCREG_IL1DATA3,               // 299
        MISCREG_DL1DATA0,               // 300
        MISCREG_DL1DATA1,               // 301
        MISCREG_DL1DATA2,               // 302
        MISCREG_DL1DATA3,               // 303
        MISCREG_DL1DATA4,               // 304
        MISCREG_RAMINDEX,               // 305
        MISCREG_L2ACTLR,                // 306
        MISCREG_CBAR,                   // 307
        MISCREG_HTTBR,                  // 308
        MISCREG_VTTBR,                  // 309
        MISCREG_CNTPCT,                 // 310
        MISCREG_CNTVCT,                 // 311
        MISCREG_CNTP_CVAL,              // 312
        MISCREG_CNTP_CVAL_NS,           // 313
        MISCREG_CNTP_CVAL_S,            // 314
        MISCREG_CNTV_CVAL,              // 315
        MISCREG_CNTVOFF,                // 316
        MISCREG_CNTHP_CVAL,             // 317
        MISCREG_CPUMERRSR,              // 318
        MISCREG_L2MERRSR,               // 319

        // AArch64 registers (Op0=2)
        MISCREG_MDCCINT_EL1,            // 320
        MISCREG_OSDTRRX_EL1,            // 321
        MISCREG_MDSCR_EL1,              // 322
        MISCREG_OSDTRTX_EL1,            // 323
        MISCREG_OSECCR_EL1,             // 324
        MISCREG_DBGBVR0_EL1,            // 325
        MISCREG_DBGBVR1_EL1,            // 326
        MISCREG_DBGBVR2_EL1,            // 327
        MISCREG_DBGBVR3_EL1,            // 328
        MISCREG_DBGBVR4_EL1,            // 329
        MISCREG_DBGBVR5_EL1,            // 330
        MISCREG_DBGBCR0_EL1,            // 331
        MISCREG_DBGBCR1_EL1,            // 332
        MISCREG_DBGBCR2_EL1,            // 333
        MISCREG_DBGBCR3_EL1,            // 334
        MISCREG_DBGBCR4_EL1,            // 335
        MISCREG_DBGBCR5_EL1,            // 336
        MISCREG_DBGWVR0_EL1,            // 337
        MISCREG_DBGWVR1_EL1,            // 338
        MISCREG_DBGWVR2_EL1,            // 339
        MISCREG_DBGWVR3_EL1,            // 340
        MISCREG_DBGWCR0_EL1,            // 341
        MISCREG_DBGWCR1_EL1,            // 342
        MISCREG_DBGWCR2_EL1,            // 343
        MISCREG_DBGWCR3_EL1,            // 344
        MISCREG_MDCCSR_EL0,             // 345
        MISCREG_MDDTR_EL0,              // 346
        MISCREG_MDDTRTX_EL0,            // 347
        MISCREG_MDDTRRX_EL0,            // 348
        MISCREG_DBGVCR32_EL2,           // 349
        MISCREG_MDRAR_EL1,              // 350
        MISCREG_OSLAR_EL1,              // 351
        MISCREG_OSLSR_EL1,              // 352
        MISCREG_OSDLR_EL1,              // 353
        MISCREG_DBGPRCR_EL1,            // 354
        MISCREG_DBGCLAIMSET_EL1,        // 355
        MISCREG_DBGCLAIMCLR_EL1,        // 356
        MISCREG_DBGAUTHSTATUS_EL1,      // 357
        MISCREG_TEECR32_EL1,            // 358
        MISCREG_TEEHBR32_EL1,           // 359

        // AArch64 registers (Op0=1,3)
        MISCREG_MIDR_EL1,               // 360
        MISCREG_MPIDR_EL1,              // 361
        MISCREG_REVIDR_EL1,             // 362
        MISCREG_ID_PFR0_EL1,            // 363
        MISCREG_ID_PFR1_EL1,            // 364
        MISCREG_ID_DFR0_EL1,            // 365
        MISCREG_ID_AFR0_EL1,            // 366
        MISCREG_ID_MMFR0_EL1,           // 367
        MISCREG_ID_MMFR1_EL1,           // 368
        MISCREG_ID_MMFR2_EL1,           // 369
        MISCREG_ID_MMFR3_EL1,           // 370
        MISCREG_ID_ISAR0_EL1,           // 371
        MISCREG_ID_ISAR1_EL1,           // 372
        MISCREG_ID_ISAR2_EL1,           // 373
        MISCREG_ID_ISAR3_EL1,           // 374
        MISCREG_ID_ISAR4_EL1,           // 375
        MISCREG_ID_ISAR5_EL1,           // 376
        MISCREG_MVFR0_EL1,              // 377
        MISCREG_MVFR1_EL1,              // 378
        MISCREG_MVFR2_EL1,              // 379
        MISCREG_ID_AA64PFR0_EL1,        // 380
        MISCREG_ID_AA64PFR1_EL1,        // 381
        MISCREG_ID_AA64DFR0_EL1,        // 382
        MISCREG_ID_AA64DFR1_EL1,        // 383
        MISCREG_ID_AA64AFR0_EL1,        // 384
        MISCREG_ID_AA64AFR1_EL1,        // 385
        MISCREG_ID_AA64ISAR0_EL1,       // 386
        MISCREG_ID_AA64ISAR1_EL1,       // 387
        MISCREG_ID_AA64MMFR0_EL1,       // 388
        MISCREG_ID_AA64MMFR1_EL1,       // 389
        MISCREG_CCSIDR_EL1,             // 390
        MISCREG_CLIDR_EL1,              // 391
        MISCREG_AIDR_EL1,               // 392
        MISCREG_CSSELR_EL1,             // 393
        MISCREG_CTR_EL0,                // 394
        MISCREG_DCZID_EL0,              // 395
        MISCREG_VPIDR_EL2,              // 396
        MISCREG_VMPIDR_EL2,             // 397
        MISCREG_SCTLR_EL1,              // 398
        MISCREG_ACTLR_EL1,              // 399
        MISCREG_CPACR_EL1,              // 400
        MISCREG_SCTLR_EL2,              // 401
        MISCREG_ACTLR_EL2,              // 402
        MISCREG_HCR_EL2,                // 403
        MISCREG_MDCR_EL2,               // 404
        MISCREG_CPTR_EL2,               // 405
        MISCREG_HSTR_EL2,               // 406
        MISCREG_HACR_EL2,               // 407
        MISCREG_SCTLR_EL3,              // 408
        MISCREG_ACTLR_EL3,              // 409
        MISCREG_SCR_EL3,                // 410
        MISCREG_SDER32_EL3,             // 411
        MISCREG_CPTR_EL3,               // 412
        MISCREG_MDCR_EL3,               // 413
        MISCREG_TTBR0_EL1,              // 414
        MISCREG_TTBR1_EL1,              // 415
        MISCREG_TCR_EL1,                // 416
        MISCREG_TTBR0_EL2,              // 417
        MISCREG_TCR_EL2,                // 418
        MISCREG_VTTBR_EL2,              // 419
        MISCREG_VTCR_EL2,               // 420
        MISCREG_TTBR0_EL3,              // 421
        MISCREG_TCR_EL3,                // 422
        MISCREG_DACR32_EL2,             // 423
        MISCREG_SPSR_EL1,               // 424
        MISCREG_ELR_EL1,                // 425
        MISCREG_SP_EL0,                 // 426
        MISCREG_SPSEL,                  // 427
        MISCREG_CURRENTEL,              // 428
        MISCREG_NZCV,                   // 429
        MISCREG_DAIF,                   // 430
        MISCREG_FPCR,                   // 431
        MISCREG_FPSR,                   // 432
        MISCREG_DSPSR_EL0,              // 433
        MISCREG_DLR_EL0,                // 434
        MISCREG_SPSR_EL2,               // 435
        MISCREG_ELR_EL2,                // 436
        MISCREG_SP_EL1,                 // 437
        MISCREG_SPSR_IRQ_AA64,          // 438
        MISCREG_SPSR_ABT_AA64,          // 439
        MISCREG_SPSR_UND_AA64,          // 440
        MISCREG_SPSR_FIQ_AA64,          // 441
        MISCREG_SPSR_EL3,               // 442
        MISCREG_ELR_EL3,                // 443
        MISCREG_SP_EL2,                 // 444
        MISCREG_AFSR0_EL1,              // 445
        MISCREG_AFSR1_EL1,              // 446
        MISCREG_ESR_EL1,                // 447
        MISCREG_IFSR32_EL2,             // 448
        MISCREG_AFSR0_EL2,              // 449
        MISCREG_AFSR1_EL2,              // 450
        MISCREG_ESR_EL2,                // 451
        MISCREG_FPEXC32_EL2,            // 452
        MISCREG_AFSR0_EL3,              // 453
        MISCREG_AFSR1_EL3,              // 454
        MISCREG_ESR_EL3,                // 455
        MISCREG_FAR_EL1,                // 456
        MISCREG_FAR_EL2,                // 457
        MISCREG_HPFAR_EL2,              // 458
        MISCREG_FAR_EL3,                // 459
        MISCREG_IC_IALLUIS,             // 460
        MISCREG_PAR_EL1,                // 461
        MISCREG_IC_IALLU,               // 462
        MISCREG_DC_IVAC_Xt,             // 463
        MISCREG_DC_ISW_Xt,              // 464
        MISCREG_AT_S1E1R_Xt,            // 465
        MISCREG_AT_S1E1W_Xt,            // 466
        MISCREG_AT_S1E0R_Xt,            // 467
        MISCREG_AT_S1E0W_Xt,            // 468
        MISCREG_DC_CSW_Xt,              // 469
        MISCREG_DC_CISW_Xt,             // 470
        MISCREG_DC_ZVA_Xt,              // 471
        MISCREG_IC_IVAU_Xt,             // 472
        MISCREG_DC_CVAC_Xt,             // 473
        MISCREG_DC_CVAU_Xt,             // 474
        MISCREG_DC_CIVAC_Xt,            // 475
        MISCREG_AT_S1E2R_Xt,            // 476
        MISCREG_AT_S1E2W_Xt,            // 477
        MISCREG_AT_S12E1R_Xt,           // 478
        MISCREG_AT_S12E1W_Xt,           // 479
        MISCREG_AT_S12E0R_Xt,           // 480
        MISCREG_AT_S12E0W_Xt,           // 481
        MISCREG_AT_S1E3R_Xt,            // 482
        MISCREG_AT_S1E3W_Xt,            // 483
        MISCREG_TLBI_VMALLE1IS,         // 484
        MISCREG_TLBI_VAE1IS_Xt,         // 485
        MISCREG_TLBI_ASIDE1IS_Xt,       // 486
        MISCREG_TLBI_VAAE1IS_Xt,        // 487
        MISCREG_TLBI_VALE1IS_Xt,        // 488
        MISCREG_TLBI_VAALE1IS_Xt,       // 489
        MISCREG_TLBI_VMALLE1,           // 490
        MISCREG_TLBI_VAE1_Xt,           // 491
        MISCREG_TLBI_ASIDE1_Xt,         // 492
        MISCREG_TLBI_VAAE1_Xt,          // 493
        MISCREG_TLBI_VALE1_Xt,          // 494
        MISCREG_TLBI_VAALE1_Xt,         // 495
        MISCREG_TLBI_IPAS2E1IS_Xt,      // 496
        MISCREG_TLBI_IPAS2LE1IS_Xt,     // 497
        MISCREG_TLBI_ALLE2IS,           // 498
        MISCREG_TLBI_VAE2IS_Xt,         // 499
        MISCREG_TLBI_ALLE1IS,           // 500
        MISCREG_TLBI_VALE2IS_Xt,        // 501
        MISCREG_TLBI_VMALLS12E1IS,      // 502
        MISCREG_TLBI_IPAS2E1_Xt,        // 503
        MISCREG_TLBI_IPAS2LE1_Xt,       // 504
        MISCREG_TLBI_ALLE2,             // 505
        MISCREG_TLBI_VAE2_Xt,           // 506
        MISCREG_TLBI_ALLE1,             // 507
        MISCREG_TLBI_VALE2_Xt,          // 508
        MISCREG_TLBI_VMALLS12E1,        // 509
        MISCREG_TLBI_ALLE3IS,           // 510
        MISCREG_TLBI_VAE3IS_Xt,         // 511
        MISCREG_TLBI_VALE3IS_Xt,        // 512
        MISCREG_TLBI_ALLE3,             // 513
        MISCREG_TLBI_VAE3_Xt,           // 514
        MISCREG_TLBI_VALE3_Xt,          // 515
        MISCREG_PMINTENSET_EL1,         // 516
        MISCREG_PMINTENCLR_EL1,         // 517
        MISCREG_PMCR_EL0,               // 518
        MISCREG_PMCNTENSET_EL0,         // 519
        MISCREG_PMCNTENCLR_EL0,         // 520
        MISCREG_PMOVSCLR_EL0,           // 521
        MISCREG_PMSWINC_EL0,            // 522
        MISCREG_PMSELR_EL0,             // 523
        MISCREG_PMCEID0_EL0,            // 524
        MISCREG_PMCEID1_EL0,            // 525
        MISCREG_PMCCNTR_EL0,            // 526
        MISCREG_PMXEVTYPER_EL0,         // 527
        MISCREG_PMCCFILTR_EL0,          // 528
        MISCREG_PMXEVCNTR_EL0,          // 529
        MISCREG_PMUSERENR_EL0,          // 530
        MISCREG_PMOVSSET_EL0,           // 531
        MISCREG_MAIR_EL1,               // 532
        MISCREG_AMAIR_EL1,              // 533
        MISCREG_MAIR_EL2,               // 534
        MISCREG_AMAIR_EL2,              // 535
        MISCREG_MAIR_EL3,               // 536
        MISCREG_AMAIR_EL3,              // 537
        MISCREG_L2CTLR_EL1,             // 538
        MISCREG_L2ECTLR_EL1,            // 539
        MISCREG_VBAR_EL1,               // 540
        MISCREG_RVBAR_EL1,              // 541
        MISCREG_ISR_EL1,                // 542
        MISCREG_VBAR_EL2,               // 543
        MISCREG_RVBAR_EL2,              // 544
        MISCREG_VBAR_EL3,               // 545
        MISCREG_RVBAR_EL3,              // 546
        MISCREG_RMR_EL3,                // 547
        MISCREG_CONTEXTIDR_EL1,         // 548
        MISCREG_TPIDR_EL1,              // 549
        MISCREG_TPIDR_EL0,              // 550
        MISCREG_TPIDRRO_EL0,            // 551
        MISCREG_TPIDR_EL2,              // 552
        MISCREG_TPIDR_EL3,              // 553
        MISCREG_CNTKCTL_EL1,            // 554
        MISCREG_CNTFRQ_EL0,             // 555
        MISCREG_CNTPCT_EL0,             // 556
        MISCREG_CNTVCT_EL0,             // 557
        MISCREG_CNTP_TVAL_EL0,          // 558
        MISCREG_CNTP_CTL_EL0,           // 559
        MISCREG_CNTP_CVAL_EL0,          // 560
        MISCREG_CNTV_TVAL_EL0,          // 561
        MISCREG_CNTV_CTL_EL0,           // 562
        MISCREG_CNTV_CVAL_EL0,          // 563
        MISCREG_PMEVCNTR0_EL0,          // 564
        MISCREG_PMEVCNTR1_EL0,          // 565
        MISCREG_PMEVCNTR2_EL0,          // 566
        MISCREG_PMEVCNTR3_EL0,          // 567
        MISCREG_PMEVCNTR4_EL0,          // 568
        MISCREG_PMEVCNTR5_EL0,          // 569
        MISCREG_PMEVTYPER0_EL0,         // 570
        MISCREG_PMEVTYPER1_EL0,         // 571
        MISCREG_PMEVTYPER2_EL0,         // 572
        MISCREG_PMEVTYPER3_EL0,         // 573
        MISCREG_PMEVTYPER4_EL0,         // 574
        MISCREG_PMEVTYPER5_EL0,         // 575
        MISCREG_CNTVOFF_EL2,            // 576
        MISCREG_CNTHCTL_EL2,            // 577
        MISCREG_CNTHP_TVAL_EL2,         // 578
        MISCREG_CNTHP_CTL_EL2,          // 579
        MISCREG_CNTHP_CVAL_EL2,         // 580
        MISCREG_CNTPS_TVAL_EL1,         // 581
        MISCREG_CNTPS_CTL_EL1,          // 582
        MISCREG_CNTPS_CVAL_EL1,         // 583
        MISCREG_IL1DATA0_EL1,           // 584
        MISCREG_IL1DATA1_EL1,           // 585
        MISCREG_IL1DATA2_EL1,           // 586
        MISCREG_IL1DATA3_EL1,           // 587
        MISCREG_DL1DATA0_EL1,           // 588
        MISCREG_DL1DATA1_EL1,           // 589
        MISCREG_DL1DATA2_EL1,           // 590
        MISCREG_DL1DATA3_EL1,           // 591
        MISCREG_DL1DATA4_EL1,           // 592
        MISCREG_L2ACTLR_EL1,            // 593
        MISCREG_CPUACTLR_EL1,           // 594
        MISCREG_CPUECTLR_EL1,           // 595
        MISCREG_CPUMERRSR_EL1,          // 596
        MISCREG_L2MERRSR_EL1,           // 597
        MISCREG_CBAR_EL1,               // 598
        MISCREG_CONTEXTIDR_EL2,         // 599

        // Dummy registers
        MISCREG_NOP,                    // 600
        MISCREG_RAZ,                    // 601
        MISCREG_CP14_UNIMPL,            // 602
        MISCREG_CP15_UNIMPL,            // 603
        MISCREG_A64_UNIMPL,             // 604
        MISCREG_UNKNOWN,                // 605

        NUM_MISCREGS                    // 606
    };

    enum MiscRegInfo {
        MISCREG_IMPLEMENTED,
        MISCREG_UNVERIFIABLE,   // Does the value change on every read (e.g. a
                                // arch generic counter)
        MISCREG_WARN_NOT_FAIL,  // If MISCREG_IMPLEMENTED is deasserted, it
                                // tells whether the instruction should raise a
                                // warning or fail
        MISCREG_MUTEX,  // True if the register corresponds to a pair of
                        // mutually exclusive registers
        MISCREG_BANKED,  // True if the register is banked between the two
                         // security states, and this is the parent node of the
                         // two banked registers
        MISCREG_BANKED_CHILD, // The entry is one of the child registers that
                              // forms a banked set of regs (along with the
                              // other child regs)

        // Access permissions
        // User mode
        MISCREG_USR_NS_RD,
        MISCREG_USR_NS_WR,
        MISCREG_USR_S_RD,
        MISCREG_USR_S_WR,
        // Privileged modes other than hypervisor or monitor
        MISCREG_PRI_NS_RD,
        MISCREG_PRI_NS_WR,
        MISCREG_PRI_S_RD,
        MISCREG_PRI_S_WR,
        // Hypervisor mode
        MISCREG_HYP_RD,
        MISCREG_HYP_WR,
        // Monitor mode, SCR.NS == 0
        MISCREG_MON_NS0_RD,
        MISCREG_MON_NS0_WR,
        // Monitor mode, SCR.NS == 1
        MISCREG_MON_NS1_RD,
        MISCREG_MON_NS1_WR,

        NUM_MISCREG_INFOS
    };

    extern std::bitset<NUM_MISCREG_INFOS> miscRegInfo[NUM_MISCREGS];

    // Decodes 32-bit CP14 registers accessible through MCR/MRC instructions
    MiscRegIndex decodeCP14Reg(unsigned crn, unsigned opc1,
                               unsigned crm, unsigned opc2);
    MiscRegIndex decodeAArch64SysReg(unsigned op0, unsigned op1,
                                     unsigned crn, unsigned crm,
                                     unsigned op2);
    // Whether a particular AArch64 system register is -always- read only.
    bool aarch64SysRegReadOnly(MiscRegIndex miscReg);

    // Decodes 32-bit CP15 registers accessible through MCR/MRC instructions
    MiscRegIndex decodeCP15Reg(unsigned crn, unsigned opc1,
                               unsigned crm, unsigned opc2);

    // Decodes 64-bit CP15 registers accessible through MCRR/MRRC instructions
    MiscRegIndex decodeCP15Reg64(unsigned crm, unsigned opc1);


    const char * const miscRegName[] = {
        "cpsr",
        "spsr",
        "spsr_fiq",
        "spsr_irq",
        "spsr_svc",
        "spsr_mon",
        "spsr_abt",
        "spsr_hyp",
        "spsr_und",
        "elr_hyp",
        "fpsid",
        "fpscr",
        "mvfr1",
        "mvfr0",
        "fpexc",

        // Helper registers
        "cpsr_mode",
        "cpsr_q",
        "fpscr_exc",
        "fpscr_qc",
        "lockaddr",
        "lockflag",
        "prrr_mair0",
        "prrr_mair0_ns",
        "prrr_mair0_s",
        "nmrr_mair1",
        "nmrr_mair1_ns",
        "nmrr_mair1_s",
        "pmxevtyper_pmccfiltr",
        "sctlr_rst",
        "sev_mailbox",

        // AArch32 CP14 registers
        "dbgdidr",
        "dbgdscrint",
        "dbgdccint",
        "dbgdtrtxint",
        "dbgdtrrxint",
        "dbgwfar",
        "dbgvcr",
        "dbgdtrrxext",
        "dbgdscrext",
        "dbgdtrtxext",
        "dbgoseccr",
        "dbgbvr0",
        "dbgbvr1",
        "dbgbvr2",
        "dbgbvr3",
        "dbgbvr4",
        "dbgbvr5",
        "dbgbcr0",
        "dbgbcr1",
        "dbgbcr2",
        "dbgbcr3",
        "dbgbcr4",
        "dbgbcr5",
        "dbgwvr0",
        "dbgwvr1",
        "dbgwvr2",
        "dbgwvr3",
        "dbgwcr0",
        "dbgwcr1",
        "dbgwcr2",
        "dbgwcr3",
        "dbgdrar",
        "dbgbxvr4",
        "dbgbxvr5",
        "dbgoslar",
        "dbgoslsr",
        "dbgosdlr",
        "dbgprcr",
        "dbgdsar",
        "dbgclaimset",
        "dbgclaimclr",
        "dbgauthstatus",
        "dbgdevid2",
        "dbgdevid1",
        "dbgdevid0",
        "teecr",
        "jidr",
        "teehbr",
        "joscr",
        "jmcr",

        // AArch32 CP15 registers
        "midr",
        "ctr",
        "tcmtr",
        "tlbtr",
        "mpidr",
        "revidr",
        "id_pfr0",
        "id_pfr1",
        "id_dfr0",
        "id_afr0",
        "id_mmfr0",
        "id_mmfr1",
        "id_mmfr2",
        "id_mmfr3",
        "id_isar0",
        "id_isar1",
        "id_isar2",
        "id_isar3",
        "id_isar4",
        "id_isar5",
        "ccsidr",
        "clidr",
        "aidr",
        "csselr",
        "csselr_ns",
        "csselr_s",
        "vpidr",
        "vmpidr",
        "sctlr",
        "sctlr_ns",
        "sctlr_s",
        "actlr",
        "actlr_ns",
        "actlr_s",
        "cpacr",
        "scr",
        "sder",
        "nsacr",
        "hsctlr",
        "hactlr",
        "hcr",
        "hdcr",
        "hcptr",
        "hstr",
        "hacr",
        "ttbr0",
        "ttbr0_ns",
        "ttbr0_s",
        "ttbr1",
        "ttbr1_ns",
        "ttbr1_s",
        "ttbcr",
        "ttbcr_ns",
        "ttbcr_s",
        "htcr",
        "vtcr",
        "dacr",
        "dacr_ns",
        "dacr_s",
        "dfsr",
        "dfsr_ns",
        "dfsr_s",
        "ifsr",
        "ifsr_ns",
        "ifsr_s",
        "adfsr",
        "adfsr_ns",
        "adfsr_s",
        "aifsr",
        "aifsr_ns",
        "aifsr_s",
        "hadfsr",
        "haifsr",
        "hsr",
        "dfar",
        "dfar_ns",
        "dfar_s",
        "ifar",
        "ifar_ns",
        "ifar_s",
        "hdfar",
        "hifar",
        "hpfar",
        "icialluis",
        "bpiallis",
        "par",
        "par_ns",
        "par_s",
        "iciallu",
        "icimvau",
        "cp15isb",
        "bpiall",
        "bpimva",
        "dcimvac",
        "dcisw",
        "ats1cpr",
        "ats1cpw",
        "ats1cur",
        "ats1cuw",
        "ats12nsopr",
        "ats12nsopw",
        "ats12nsour",
        "ats12nsouw",
        "dccmvac",
        "dccsw",
        "cp15dsb",
        "cp15dmb",
        "dccmvau",
        "dccimvac",
        "dccisw",
        "ats1hr",
        "ats1hw",
        "tlbiallis",
        "tlbimvais",
        "tlbiasidis",
        "tlbimvaais",
        "tlbimvalis",
        "tlbimvaalis",
        "itlbiall",
        "itlbimva",
        "itlbiasid",
        "dtlbiall",
        "dtlbimva",
        "dtlbiasid",
        "tlbiall",
        "tlbimva",
        "tlbiasid",
        "tlbimvaa",
        "tlbimval",
        "tlbimvaal",
        "tlbiipas2is",
        "tlbiipas2lis",
        "tlbiallhis",
        "tlbimvahis",
        "tlbiallnsnhis",
        "tlbimvalhis",
        "tlbiipas2",
        "tlbiipas2l",
        "tlbiallh",
        "tlbimvah",
        "tlbiallnsnh",
        "tlbimvalh",
        "pmcr",
        "pmcntenset",
        "pmcntenclr",
        "pmovsr",
        "pmswinc",
        "pmselr",
        "pmceid0",
        "pmceid1",
        "pmccntr",
        "pmxevtyper",
        "pmccfiltr",
        "pmxevcntr",
        "pmuserenr",
        "pmintenset",
        "pmintenclr",
        "pmovsset",
        "l2ctlr",
        "l2ectlr",
        "prrr",
        "prrr_ns",
        "prrr_s",
        "mair0",
        "mair0_ns",
        "mair0_s",
        "nmrr",
        "nmrr_ns",
        "nmrr_s",
        "mair1",
        "mair1_ns",
        "mair1_s",
        "amair0",
        "amair0_ns",
        "amair0_s",
        "amair1",
        "amair1_ns",
        "amair1_s",
        "hmair0",
        "hmair1",
        "hamair0",
        "hamair1",
        "vbar",
        "vbar_ns",
        "vbar_s",
        "mvbar",
        "rmr",
        "isr",
        "hvbar",
        "fcseidr",
        "contextidr",
        "contextidr_ns",
        "contextidr_s",
        "tpidrurw",
        "tpidrurw_ns",
        "tpidrurw_s",
        "tpidruro",
        "tpidruro_ns",
        "tpidruro_s",
        "tpidrprw",
        "tpidrprw_ns",
        "tpidrprw_s",
        "htpidr",
        "cntfrq",
        "cntkctl",
        "cntp_tval",
        "cntp_tval_ns",
        "cntp_tval_s",
        "cntp_ctl",
        "cntp_ctl_ns",
        "cntp_ctl_s",
        "cntv_tval",
        "cntv_ctl",
        "cnthctl",
        "cnthp_tval",
        "cnthp_ctl",
        "il1data0",
        "il1data1",
        "il1data2",
        "il1data3",
        "dl1data0",
        "dl1data1",
        "dl1data2",
        "dl1data3",
        "dl1data4",
        "ramindex",
        "l2actlr",
        "cbar",
        "httbr",
        "vttbr",
        "cntpct",
        "cntvct",
        "cntp_cval",
        "cntp_cval_ns",
        "cntp_cval_s",
        "cntv_cval",
        "cntvoff",
        "cnthp_cval",
        "cpumerrsr",
        "l2merrsr",

        // AArch64 registers (Op0=2)
        "mdccint_el1",
        "osdtrrx_el1",
        "mdscr_el1",
        "osdtrtx_el1",
        "oseccr_el1",
        "dbgbvr0_el1",
        "dbgbvr1_el1",
        "dbgbvr2_el1",
        "dbgbvr3_el1",
        "dbgbvr4_el1",
        "dbgbvr5_el1",
        "dbgbcr0_el1",
        "dbgbcr1_el1",
        "dbgbcr2_el1",
        "dbgbcr3_el1",
        "dbgbcr4_el1",
        "dbgbcr5_el1",
        "dbgwvr0_el1",
        "dbgwvr1_el1",
        "dbgwvr2_el1",
        "dbgwvr3_el1",
        "dbgwcr0_el1",
        "dbgwcr1_el1",
        "dbgwcr2_el1",
        "dbgwcr3_el1",
        "mdccsr_el0",
        "mddtr_el0",
        "mddtrtx_el0",
        "mddtrrx_el0",
        "dbgvcr32_el2",
        "mdrar_el1",
        "oslar_el1",
        "oslsr_el1",
        "osdlr_el1",
        "dbgprcr_el1",
        "dbgclaimset_el1",
        "dbgclaimclr_el1",
        "dbgauthstatus_el1",
        "teecr32_el1",
        "teehbr32_el1",

        // AArch64 registers (Op0=1,3)
        "midr_el1",
        "mpidr_el1",
        "revidr_el1",
        "id_pfr0_el1",
        "id_pfr1_el1",
        "id_dfr0_el1",
        "id_afr0_el1",
        "id_mmfr0_el1",
        "id_mmfr1_el1",
        "id_mmfr2_el1",
        "id_mmfr3_el1",
        "id_isar0_el1",
        "id_isar1_el1",
        "id_isar2_el1",
        "id_isar3_el1",
        "id_isar4_el1",
        "id_isar5_el1",
        "mvfr0_el1",
        "mvfr1_el1",
        "mvfr2_el1",
        "id_aa64pfr0_el1",
        "id_aa64pfr1_el1",
        "id_aa64dfr0_el1",
        "id_aa64dfr1_el1",
        "id_aa64afr0_el1",
        "id_aa64afr1_el1",
        "id_aa64isar0_el1",
        "id_aa64isar1_el1",
        "id_aa64mmfr0_el1",
        "id_aa64mmfr1_el1",
        "ccsidr_el1",
        "clidr_el1",
        "aidr_el1",
        "csselr_el1",
        "ctr_el0",
        "dczid_el0",
        "vpidr_el2",
        "vmpidr_el2",
        "sctlr_el1",
        "actlr_el1",
        "cpacr_el1",
        "sctlr_el2",
        "actlr_el2",
        "hcr_el2",
        "mdcr_el2",
        "cptr_el2",
        "hstr_el2",
        "hacr_el2",
        "sctlr_el3",
        "actlr_el3",
        "scr_el3",
        "sder32_el3",
        "cptr_el3",
        "mdcr_el3",
        "ttbr0_el1",
        "ttbr1_el1",
        "tcr_el1",
        "ttbr0_el2",
        "tcr_el2",
        "vttbr_el2",
        "vtcr_el2",
        "ttbr0_el3",
        "tcr_el3",
        "dacr32_el2",
        "spsr_el1",
        "elr_el1",
        "sp_el0",
        "spsel",
        "currentel",
        "nzcv",
        "daif",
        "fpcr",
        "fpsr",
        "dspsr_el0",
        "dlr_el0",
        "spsr_el2",
        "elr_el2",
        "sp_el1",
        "spsr_irq_aa64",
        "spsr_abt_aa64",
        "spsr_und_aa64",
        "spsr_fiq_aa64",
        "spsr_el3",
        "elr_el3",
        "sp_el2",
        "afsr0_el1",
        "afsr1_el1",
        "esr_el1",
        "ifsr32_el2",
        "afsr0_el2",
        "afsr1_el2",
        "esr_el2",
        "fpexc32_el2",
        "afsr0_el3",
        "afsr1_el3",
        "esr_el3",
        "far_el1",
        "far_el2",
        "hpfar_el2",
        "far_el3",
        "ic_ialluis",
        "par_el1",
        "ic_iallu",
        "dc_ivac_xt",
        "dc_isw_xt",
        "at_s1e1r_xt",
        "at_s1e1w_xt",
        "at_s1e0r_xt",
        "at_s1e0w_xt",
        "dc_csw_xt",
        "dc_cisw_xt",
        "dc_zva_xt",
        "ic_ivau_xt",
        "dc_cvac_xt",
        "dc_cvau_xt",
        "dc_civac_xt",
        "at_s1e2r_xt",
        "at_s1e2w_xt",
        "at_s12e1r_xt",
        "at_s12e1w_xt",
        "at_s12e0r_xt",
        "at_s12e0w_xt",
        "at_s1e3r_xt",
        "at_s1e3w_xt",
        "tlbi_vmalle1is",
        "tlbi_vae1is_xt",
        "tlbi_aside1is_xt",
        "tlbi_vaae1is_xt",
        "tlbi_vale1is_xt",
        "tlbi_vaale1is_xt",
        "tlbi_vmalle1",
        "tlbi_vae1_xt",
        "tlbi_aside1_xt",
        "tlbi_vaae1_xt",
        "tlbi_vale1_xt",
        "tlbi_vaale1_xt",
        "tlbi_ipas2e1is_xt",
        "tlbi_ipas2le1is_xt",
        "tlbi_alle2is",
        "tlbi_vae2is_xt",
        "tlbi_alle1is",
        "tlbi_vale2is_xt",
        "tlbi_vmalls12e1is",
        "tlbi_ipas2e1_xt",
        "tlbi_ipas2le1_xt",
        "tlbi_alle2",
        "tlbi_vae2_xt",
        "tlbi_alle1",
        "tlbi_vale2_xt",
        "tlbi_vmalls12e1",
        "tlbi_alle3is",
        "tlbi_vae3is_xt",
        "tlbi_vale3is_xt",
        "tlbi_alle3",
        "tlbi_vae3_xt",
        "tlbi_vale3_xt",
        "pmintenset_el1",
        "pmintenclr_el1",
        "pmcr_el0",
        "pmcntenset_el0",
        "pmcntenclr_el0",
        "pmovsclr_el0",
        "pmswinc_el0",
        "pmselr_el0",
        "pmceid0_el0",
        "pmceid1_el0",
        "pmccntr_el0",
        "pmxevtyper_el0",
        "pmccfiltr_el0",
        "pmxevcntr_el0",
        "pmuserenr_el0",
        "pmovsset_el0",
        "mair_el1",
        "amair_el1",
        "mair_el2",
        "amair_el2",
        "mair_el3",
        "amair_el3",
        "l2ctlr_el1",
        "l2ectlr_el1",
        "vbar_el1",
        "rvbar_el1",
        "isr_el1",
        "vbar_el2",
        "rvbar_el2",
        "vbar_el3",
        "rvbar_el3",
        "rmr_el3",
        "contextidr_el1",
        "tpidr_el1",
        "tpidr_el0",
        "tpidrro_el0",
        "tpidr_el2",
        "tpidr_el3",
        "cntkctl_el1",
        "cntfrq_el0",
        "cntpct_el0",
        "cntvct_el0",
        "cntp_tval_el0",
        "cntp_ctl_el0",
        "cntp_cval_el0",
        "cntv_tval_el0",
        "cntv_ctl_el0",
        "cntv_cval_el0",
        "pmevcntr0_el0",
        "pmevcntr1_el0",
        "pmevcntr2_el0",
        "pmevcntr3_el0",
        "pmevcntr4_el0",
        "pmevcntr5_el0",
        "pmevtyper0_el0",
        "pmevtyper1_el0",
        "pmevtyper2_el0",
        "pmevtyper3_el0",
        "pmevtyper4_el0",
        "pmevtyper5_el0",
        "cntvoff_el2",
        "cnthctl_el2",
        "cnthp_tval_el2",
        "cnthp_ctl_el2",
        "cnthp_cval_el2",
        "cntps_tval_el1",
        "cntps_ctl_el1",
        "cntps_cval_el1",
        "il1data0_el1",
        "il1data1_el1",
        "il1data2_el1",
        "il1data3_el1",
        "dl1data0_el1",
        "dl1data1_el1",
        "dl1data2_el1",
        "dl1data3_el1",
        "dl1data4_el1",
        "l2actlr_el1",
        "cpuactlr_el1",
        "cpuectlr_el1",
        "cpumerrsr_el1",
        "l2merrsr_el1",
        "cbar_el1",
        "contextidr_el2",

        // Dummy registers
        "nop",
        "raz",
        "cp14_unimpl",
        "cp15_unimpl",
        "a64_unimpl",
        "unknown"
    };

    static_assert(sizeof(miscRegName) / sizeof(*miscRegName) == NUM_MISCREGS,
                  "The miscRegName array and NUM_MISCREGS are inconsistent.");

    BitUnion32(CPSR)
        Bitfield<31, 30> nz;
        Bitfield<29> c;
        Bitfield<28> v;
        Bitfield<27> q;
        Bitfield<26, 25> it1;
        Bitfield<24> j;
        Bitfield<23, 22> res0_23_22;
        Bitfield<21> ss;        // AArch64
        Bitfield<20> il;        // AArch64
        Bitfield<19, 16> ge;
        Bitfield<15, 10> it2;
        Bitfield<9> d;          // AArch64
        Bitfield<9> e;
        Bitfield<8> a;
        Bitfield<7> i;
        Bitfield<6> f;
        Bitfield<9, 6> daif;    // AArch64
        Bitfield<5> t;
        Bitfield<4> width;      // AArch64
        Bitfield<3, 2> el;      // AArch64
        Bitfield<4, 0> mode;
        Bitfield<0> sp;         // AArch64
    EndBitUnion(CPSR)

    // This mask selects bits of the CPSR that actually go in the CondCodes
    // integer register to allow renaming.
    static const uint32_t CondCodesMask   = 0xF00F0000;
    static const uint32_t CpsrMaskQ       = 0x08000000;

    BitUnion32(HDCR)
        Bitfield<11>   tdra;
        Bitfield<10>   tdosa;
        Bitfield<9>    tda;
        Bitfield<8>    tde;
        Bitfield<7>    hpme;
        Bitfield<6>    tpm;
        Bitfield<5>    tpmcr;
        Bitfield<4, 0> hpmn;
    EndBitUnion(HDCR)

    BitUnion32(HCPTR)
        Bitfield<31> tcpac;
        Bitfield<20> tta;
        Bitfield<15> tase;
        Bitfield<13> tcp13;
        Bitfield<12> tcp12;
        Bitfield<11> tcp11;
        Bitfield<10> tcp10;
        Bitfield<10> tfp;  // AArch64
        Bitfield<9>  tcp9;
        Bitfield<8>  tcp8;
        Bitfield<7>  tcp7;
        Bitfield<6>  tcp6;
        Bitfield<5>  tcp5;
        Bitfield<4>  tcp4;
        Bitfield<3>  tcp3;
        Bitfield<2>  tcp2;
        Bitfield<1>  tcp1;
        Bitfield<0>  tcp0;
    EndBitUnion(HCPTR)

    BitUnion32(HSTR)
        Bitfield<17> tjdbx;
        Bitfield<16> ttee;
        Bitfield<15> t15;
        Bitfield<13> t13;
        Bitfield<12> t12;
        Bitfield<11> t11;
        Bitfield<10> t10;
        Bitfield<9>  t9;
        Bitfield<8>  t8;
        Bitfield<7>  t7;
        Bitfield<6>  t6;
        Bitfield<5>  t5;
        Bitfield<4>  t4;
        Bitfield<3>  t3;
        Bitfield<2>  t2;
        Bitfield<1>  t1;
        Bitfield<0>  t0;
    EndBitUnion(HSTR)

    BitUnion64(HCR)
        Bitfield<33>     id;    // AArch64
        Bitfield<32>     cd;    // AArch64
        Bitfield<31>     rw;    // AArch64
        Bitfield<30>     trvm;  // AArch64
        Bitfield<29>     hcd;   // AArch64
        Bitfield<28>     tdz;   // AArch64

        Bitfield<27>     tge;
        Bitfield<26>     tvm;
        Bitfield<25>     ttlb;
        Bitfield<24>     tpu;
        Bitfield<23>     tpc;
        Bitfield<22>     tsw;
        Bitfield<21>     tac;
        Bitfield<21>     tacr;  // AArch64
        Bitfield<20>     tidcp;
        Bitfield<19>     tsc;
        Bitfield<18>     tid3;
        Bitfield<17>     tid2;
        Bitfield<16>     tid1;
        Bitfield<15>     tid0;
        Bitfield<14>     twe;
        Bitfield<13>     twi;
        Bitfield<12>     dc;
        Bitfield<11, 10> bsu;
        Bitfield<9>      fb;
        Bitfield<8>      va;
        Bitfield<8>      vse;   // AArch64
        Bitfield<7>      vi;
        Bitfield<6>      vf;
        Bitfield<5>      amo;
        Bitfield<4>      imo;
        Bitfield<3>      fmo;
        Bitfield<2>      ptw;
        Bitfield<1>      swio;
        Bitfield<0>      vm;
    EndBitUnion(HCR)

    BitUnion32(NSACR)
        Bitfield<20> nstrcdis;
        Bitfield<19> rfr;
        Bitfield<15> nsasedis;
        Bitfield<14> nsd32dis;
        Bitfield<13> cp13;
        Bitfield<12> cp12;
        Bitfield<11> cp11;
        Bitfield<10> cp10;
        Bitfield<9>  cp9;
        Bitfield<8>  cp8;
        Bitfield<7>  cp7;
        Bitfield<6>  cp6;
        Bitfield<5>  cp5;
        Bitfield<4>  cp4;
        Bitfield<3>  cp3;
        Bitfield<2>  cp2;
        Bitfield<1>  cp1;
        Bitfield<0>  cp0;
    EndBitUnion(NSACR)

    BitUnion32(SCR)
        Bitfield<13> twe;
        Bitfield<12> twi;
        Bitfield<11> st;  // AArch64
        Bitfield<10> rw;  // AArch64
        Bitfield<9> sif;
        Bitfield<8> hce;
        Bitfield<7> scd;
        Bitfield<7> smd;  // AArch64
        Bitfield<6> nEt;
        Bitfield<5> aw;
        Bitfield<4> fw;
        Bitfield<3> ea;
        Bitfield<2> fiq;
        Bitfield<1> irq;
        Bitfield<0> ns;
    EndBitUnion(SCR)

    BitUnion32(SCTLR)
        Bitfield<30>   te;      // Thumb Exception Enable (AArch32 only)
        Bitfield<29>   afe;     // Access flag enable (AArch32 only)
        Bitfield<28>   tre;     // TEX remap enable (AArch32 only)
        Bitfield<27>   nmfi;    // Non-maskable FIQ support (ARMv7 only)
        Bitfield<26>   uci;     // Enable EL0 access to DC CVAU, DC CIVAC,
                                // DC CVAC and IC IVAU instructions
                                // (AArch64 SCTLR_EL1 only)
        Bitfield<25>   ee;      // Exception Endianness
        Bitfield<24>   ve;      // Interrupt Vectors Enable (ARMv7 only)
        Bitfield<24>   e0e;     // Endianness of explicit data accesses at EL0
                                // (AArch64 SCTLR_EL1 only)
        Bitfield<23>   xp;      // Extended page table enable (dropped in ARMv7)
        Bitfield<22>   u;       // Alignment (dropped in ARMv7)
        Bitfield<21>   fi;      // Fast interrupts configuration enable
                                // (ARMv7 only)
        Bitfield<20>   uwxn;    // Unprivileged write permission implies EL1 XN
                                // (AArch32 only)
        Bitfield<19>   dz;      // Divide by Zero fault enable
                                // (dropped in ARMv7)
        Bitfield<19>   wxn;     // Write permission implies XN
        Bitfield<18>   ntwe;    // Not trap WFE
                                // (ARMv8 AArch32 and AArch64 SCTLR_EL1 only)
        Bitfield<18>   rao2;    // Read as one
        Bitfield<16>   ntwi;    // Not trap WFI
                                // (ARMv8 AArch32 and AArch64 SCTLR_EL1 only)
        Bitfield<16>   rao3;    // Read as one
        Bitfield<15>   uct;     // Enable EL0 access to CTR_EL0
                                // (AArch64 SCTLR_EL1 only)
        Bitfield<14>   rr;      // Round Robin select (ARMv7 only)
        Bitfield<14>   dze;     // Enable EL0 access to DC ZVA
                                // (AArch64 SCTLR_EL1 only)
        Bitfield<13>   v;       // Vectors bit (AArch32 only)
        Bitfield<12>   i;       // Instruction cache enable
        Bitfield<11>   z;       // Branch prediction enable (ARMv7 only)
        Bitfield<10>   sw;      // SWP/SWPB enable (ARMv7 only)
        Bitfield<9, 8> rs;      // Deprecated protection bits (dropped in ARMv7)
        Bitfield<9>    uma;     // User mask access (AArch64 SCTLR_EL1 only)
        Bitfield<8>    sed;     // SETEND disable
                                // (ARMv8 AArch32 and AArch64 SCTLR_EL1 only)
        Bitfield<7>    b;       // Endianness support (dropped in ARMv7)
        Bitfield<7>    itd;     // IT disable
                                // (ARMv8 AArch32 and AArch64 SCTLR_EL1 only)
        Bitfield<6, 3> rao4;    // Read as one
        Bitfield<6>    thee;    // ThumbEE enable
                                // (ARMv8 AArch32 and AArch64 SCTLR_EL1 only)
        Bitfield<5>    cp15ben; // CP15 barrier enable
                                // (AArch32 and AArch64 SCTLR_EL1 only)
        Bitfield<4>    sa0;     // Stack Alignment Check Enable for EL0
                                // (AArch64 SCTLR_EL1 only)
        Bitfield<3>    sa;      // Stack Alignment Check Enable (AArch64 only)
        Bitfield<2>    c;       // Cache enable
        Bitfield<1>    a;       // Alignment check enable
        Bitfield<0>    m;       // MMU enable
    EndBitUnion(SCTLR)

    BitUnion32(CPACR)
        Bitfield<1, 0> cp0;
        Bitfield<3, 2> cp1;
        Bitfield<5, 4> cp2;
        Bitfield<7, 6> cp3;
        Bitfield<9, 8> cp4;
        Bitfield<11, 10> cp5;
        Bitfield<13, 12> cp6;
        Bitfield<15, 14> cp7;
        Bitfield<17, 16> cp8;
        Bitfield<19, 18> cp9;
        Bitfield<21, 20> cp10;
        Bitfield<21, 20> fpen;  // AArch64
        Bitfield<23, 22> cp11;
        Bitfield<25, 24> cp12;
        Bitfield<27, 26> cp13;
        Bitfield<29, 28> rsvd;
        Bitfield<28> tta;  // AArch64
        Bitfield<30> d32dis;
        Bitfield<31> asedis;
    EndBitUnion(CPACR)

    BitUnion32(FSR)
        Bitfield<3, 0> fsLow;
        Bitfield<5, 0> status;  // LPAE
        Bitfield<7, 4> domain;
        Bitfield<9> lpae;
        Bitfield<10> fsHigh;
        Bitfield<11> wnr;
        Bitfield<12> ext;
        Bitfield<13> cm;  // LPAE
    EndBitUnion(FSR)

    BitUnion32(FPSCR)
        Bitfield<0> ioc;
        Bitfield<1> dzc;
        Bitfield<2> ofc;
        Bitfield<3> ufc;
        Bitfield<4> ixc;
        Bitfield<7> idc;
        Bitfield<8> ioe;
        Bitfield<9> dze;
        Bitfield<10> ofe;
        Bitfield<11> ufe;
        Bitfield<12> ixe;
        Bitfield<15> ide;
        Bitfield<18, 16> len;
        Bitfield<21, 20> stride;
        Bitfield<23, 22> rMode;
        Bitfield<24> fz;
        Bitfield<25> dn;
        Bitfield<26> ahp;
        Bitfield<27> qc;
        Bitfield<28> v;
        Bitfield<29> c;
        Bitfield<30> z;
        Bitfield<31> n;
    EndBitUnion(FPSCR)

    // This mask selects bits of the FPSCR that actually go in the FpCondCodes
    // integer register to allow renaming.
    static const uint32_t FpCondCodesMask = 0xF0000000;
    // This mask selects the cumulative FP exception flags of the FPSCR.
    static const uint32_t FpscrExcMask = 0x0000009F;
    // This mask selects the cumulative saturation flag of the FPSCR.
    static const uint32_t FpscrQcMask = 0x08000000;

    BitUnion32(FPEXC)
        Bitfield<31> ex;
        Bitfield<30> en;
        Bitfield<29, 0> subArchDefined;
    EndBitUnion(FPEXC)

    BitUnion32(MVFR0)
        Bitfield<3, 0> advSimdRegisters;
        Bitfield<7, 4> singlePrecision;
        Bitfield<11, 8> doublePrecision;
        Bitfield<15, 12> vfpExceptionTrapping;
        Bitfield<19, 16> divide;
        Bitfield<23, 20> squareRoot;
        Bitfield<27, 24> shortVectors;
        Bitfield<31, 28> roundingModes;
    EndBitUnion(MVFR0)

    BitUnion32(MVFR1)
        Bitfield<3, 0> flushToZero;
        Bitfield<7, 4> defaultNaN;
        Bitfield<11, 8> advSimdLoadStore;
        Bitfield<15, 12> advSimdInteger;
        Bitfield<19, 16> advSimdSinglePrecision;
        Bitfield<23, 20> advSimdHalfPrecision;
        Bitfield<27, 24> vfpHalfPrecision;
        Bitfield<31, 28> raz;
    EndBitUnion(MVFR1)

    BitUnion64(TTBCR)
        // Short-descriptor translation table format
        Bitfield<2, 0> n;
        Bitfield<4> pd0;
        Bitfield<5> pd1;
        // Long-descriptor translation table format
        Bitfield<5, 0> t0sz;
        Bitfield<7> epd0;
        Bitfield<9, 8> irgn0;
        Bitfield<11, 10> orgn0;
        Bitfield<13, 12> sh0;
        Bitfield<14> tg0;
        Bitfield<21, 16> t1sz;
        Bitfield<22> a1;
        Bitfield<23> epd1;
        Bitfield<25, 24> irgn1;
        Bitfield<27, 26> orgn1;
        Bitfield<29, 28> sh1;
        Bitfield<30> tg1;
        Bitfield<34, 32> ips;
        Bitfield<36> as;
        Bitfield<37> tbi0;
        Bitfield<38> tbi1;
        // Common
        Bitfield<31> eae;
        // TCR_EL2/3 (AArch64)
        Bitfield<18, 16> ps;
        Bitfield<20> tbi;
    EndBitUnion(TTBCR)

    // Fields of TCR_EL{1,2,3} (mostly overlapping)
    // TCR_EL1 is natively 64 bits, the others are 32 bits
    BitUnion64(TCR)
        Bitfield<5, 0> t0sz;
        Bitfield<7> epd0; // EL1
        Bitfield<9, 8> irgn0;
        Bitfield<11, 10> orgn0;
        Bitfield<13, 12> sh0;
        Bitfield<15, 14> tg0;
        Bitfield<18, 16> ps;
        Bitfield<20> tbi; // EL2/EL3
        Bitfield<21, 16> t1sz; // EL1
        Bitfield<22> a1; // EL1
        Bitfield<23> epd1; // EL1
        Bitfield<25, 24> irgn1; // EL1
        Bitfield<27, 26> orgn1; // EL1
        Bitfield<29, 28> sh1; // EL1
        Bitfield<31, 30> tg1; // EL1
        Bitfield<34, 32> ips; // EL1
        Bitfield<36> as; // EL1
        Bitfield<37> tbi0; // EL1
        Bitfield<38> tbi1; // EL1
    EndBitUnion(TCR)

    BitUnion32(HTCR)
        Bitfield<2, 0> t0sz;
        Bitfield<9, 8> irgn0;
        Bitfield<11, 10> orgn0;
        Bitfield<13, 12> sh0;
    EndBitUnion(HTCR)

    BitUnion32(VTCR_t)
        Bitfield<3, 0> t0sz;
        Bitfield<4> s;
        Bitfield<7, 6> sl0;
        Bitfield<9, 8> irgn0;
        Bitfield<11, 10> orgn0;
        Bitfield<13, 12> sh0;
    EndBitUnion(VTCR_t)

    BitUnion32(PRRR)
       Bitfield<1,0> tr0;
       Bitfield<3,2> tr1;
       Bitfield<5,4> tr2;
       Bitfield<7,6> tr3;
       Bitfield<9,8> tr4;
       Bitfield<11,10> tr5;
       Bitfield<13,12> tr6;
       Bitfield<15,14> tr7;
       Bitfield<16> ds0;
       Bitfield<17> ds1;
       Bitfield<18> ns0;
       Bitfield<19> ns1;
       Bitfield<24> nos0;
       Bitfield<25> nos1;
       Bitfield<26> nos2;
       Bitfield<27> nos3;
       Bitfield<28> nos4;
       Bitfield<29> nos5;
       Bitfield<30> nos6;
       Bitfield<31> nos7;
   EndBitUnion(PRRR)

   BitUnion32(NMRR)
       Bitfield<1,0> ir0;
       Bitfield<3,2> ir1;
       Bitfield<5,4> ir2;
       Bitfield<7,6> ir3;
       Bitfield<9,8> ir4;
       Bitfield<11,10> ir5;
       Bitfield<13,12> ir6;
       Bitfield<15,14> ir7;
       Bitfield<17,16> or0;
       Bitfield<19,18> or1;
       Bitfield<21,20> or2;
       Bitfield<23,22> or3;
       Bitfield<25,24> or4;
       Bitfield<27,26> or5;
       Bitfield<29,28> or6;
       Bitfield<31,30> or7;
   EndBitUnion(NMRR)

   BitUnion32(CONTEXTIDR)
      Bitfield<7,0>  asid;
      Bitfield<31,8> procid;
   EndBitUnion(CONTEXTIDR)

   BitUnion32(L2CTLR)
      Bitfield<2,0>   sataRAMLatency;
      Bitfield<4,3>   reserved_4_3;
      Bitfield<5>     dataRAMSetup;
      Bitfield<8,6>   tagRAMLatency;
      Bitfield<9>     tagRAMSetup;
      Bitfield<11,10> dataRAMSlice;
      Bitfield<12>    tagRAMSlice;
      Bitfield<20,13> reserved_20_13;
      Bitfield<21>    eccandParityEnable;
      Bitfield<22>    reserved_22;
      Bitfield<23>    interptCtrlPresent;
      Bitfield<25,24> numCPUs;
      Bitfield<30,26> reserved_30_26;
      Bitfield<31>    l2rstDISABLE_monitor;
   EndBitUnion(L2CTLR)

   BitUnion32(CTR)
      Bitfield<3,0>   iCacheLineSize;
      Bitfield<13,4>  raz_13_4;
      Bitfield<15,14> l1IndexPolicy;
      Bitfield<19,16> dCacheLineSize;
      Bitfield<23,20> erg;
      Bitfield<27,24> cwg;
      Bitfield<28>    raz_28;
      Bitfield<31,29> format;
   EndBitUnion(CTR)

   BitUnion32(PMSELR)
      Bitfield<4, 0> sel;
   EndBitUnion(PMSELR)

    BitUnion64(PAR)
        // 64-bit format
        Bitfield<63, 56> attr;
        Bitfield<39, 12> pa;
        Bitfield<11>     lpae;
        Bitfield<9>      ns;
        Bitfield<8, 7>   sh;
        Bitfield<0>      f;
   EndBitUnion(PAR)

   BitUnion32(ESR)
        Bitfield<31, 26> ec;
        Bitfield<25> il;
        Bitfield<15, 0> imm16;
   EndBitUnion(ESR)

   BitUnion32(CPTR)
        Bitfield<31> tcpac;
        Bitfield<20> tta;
        Bitfield<13, 12> res1_13_12_el2;
        Bitfield<10> tfp;
        Bitfield<9, 0> res1_9_0_el2;
   EndBitUnion(CPTR)


    // Checks read access permissions to coproc. registers
    bool canReadCoprocReg(MiscRegIndex reg, SCR scr, CPSR cpsr,
                          ThreadContext *tc);

    // Checks write access permissions to coproc. registers
    bool canWriteCoprocReg(MiscRegIndex reg, SCR scr, CPSR cpsr,
                           ThreadContext *tc);

    // Checks read access permissions to AArch64 system registers
    bool canReadAArch64SysReg(MiscRegIndex reg, SCR scr, CPSR cpsr,
                              ThreadContext *tc);

    // Checks write access permissions to AArch64 system registers
    bool canWriteAArch64SysReg(MiscRegIndex reg, SCR scr, CPSR cpsr,
                               ThreadContext *tc);

    // Uses just the scr.ns bit to pre flatten the misc regs. This is useful
    // for MCR/MRC instructions
    int
    flattenMiscRegNsBanked(MiscRegIndex reg, ThreadContext *tc);

    // Flattens a misc reg index using the specified security state. This is
    // used for opperations (eg address translations) where the security
    // state of the register access may differ from the current state of the
    // processor
    int
    flattenMiscRegNsBanked(MiscRegIndex reg, ThreadContext *tc, bool ns);

    // Takes a misc reg index and returns the root reg if its one of a set of
    // banked registers
    void
    preUnflattenMiscReg();

    int
    unflattenMiscReg(int reg);

}

#endif // __ARCH_ARM_MISCREGS_HH__
