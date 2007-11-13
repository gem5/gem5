/*
 * Copyright N) 2007 MIPS Technologies, Inc.  All Rights Reserved
 *
 * This software is part of the M5 simulator.
 *
 * THIS IS A LEGAL AGREEMENT.  BY DOWNLOADING, USING, COPYING, CREATING
 * DERIVATIVE WORKS, AND/OR DISTRIBUTING THIS SOFTWARE YOU ARE AGREEING
 * TO THESE TERMS AND CONDITIONS.
 *
 * Permission is granted to use, copy, create derivative works and
 * distribute this software and such derivative works for any purpose,
 * so long as (1) the copyright notice above, this grant of permission,
 * and the disclaimer below appear in all copies and derivative works
 * made, (2) the copyright notice above is augmented as appropriate to
 * reflect the addition of any new copyrightable work in a derivative
 * work (e.g., Copyright N) <Publication Year> Copyright Owner), and (3)
 * the name of MIPS Technologies, Inc. ($(B!H(BMIPS$(B!I(B) is not used in any
 * advertising or publicity pertaining to the use or distribution of
 * this software without specific, written prior authorization.
 *
 * THIS SOFTWARE IS PROVIDED $(B!H(BAS IS.$(B!I(B  MIPS MAKES NO WARRANTIES AND
 * DISCLAIMS ALL WARRANTIES, WHETHER EXPRESS, STATUTORY, IMPLIED OR
 * OTHERWISE, INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 * NON-INFRINGEMENT OF THIRD PARTY RIGHTS, REGARDING THIS SOFTWARE.
 * IN NO EVENT SHALL MIPS BE LIABLE FOR ANY DAMAGES, INCLUDING DIRECT,
 * INDIRECT, INCIDENTAL, CONSEQUENTIAL, SPECIAL, OR PUNITIVE DAMAGES OF
 * ANY KIND OR NATURE, ARISING OUT OF OR IN CONNECTION WITH THIS AGREEMENT,
 * THIS SOFTWARE AND/OR THE USE OF THIS SOFTWARE, WHETHER SUCH LIABILITY
 * IS ASSERTED ON THE BASIS OF CONTRACT, TORT (INCLUDING NEGLIGENCE OR
 * STRICT LIABILITY), OR OTHERWISE, EVEN IF MIPS HAS BEEN WARNED OF THE
 * POSSIBILITY OF ANY SUCH LOSS OR DAMAGE IN ADVANCE.
 *
 * Authors: Brett Miller
 *
 */

#ifndef __ARCH_MIPS_DSP_HH__
#define __ARCH_MIPS_DSP_HH__

#include "arch/mips/types.hh"
#include "arch/mips/isa_traits.hh"
#include "base/misc.hh"
#include "config/full_system.hh"
#include "sim/host.hh"

class ThreadContext;

namespace MipsISA {

    // SIMD formats
    enum {
        SIMD_FMT_L,    // long word
        SIMD_FMT_W,    // word
        SIMD_FMT_PH,   // paired halfword
        SIMD_FMT_QB,   // quad byte
        SIMD_NUM_FMTS
    };

    // DSPControl Fields
    enum {
        DSP_POS,       // insertion bitfield position
        DSP_SCOUNT,    // insertion bitfield size
        DSP_C,         // carry bit
        DSP_OUFLAG,    // overflow-underflow flag
        DSP_CCOND,     // condition code
        DSP_EFI,       // extract fail indicator bit
        DSP_NUM_FIELDS
    };

    // compare instruction operations
    enum {
        CMP_EQ,        // equal
        CMP_LT,        // less than
        CMP_LE         // less than or equal
    };

    // SIMD operation order modes
    enum {
        MODE_L,        // left
        MODE_R,        // right
        MODE_LA,       // left-alternate
        MODE_RA,       // right-alternate
        MODE_X         // cross
    };

    // dsp operation parameters
    enum { UNSIGNED, SIGNED };
    enum { NOSATURATE, SATURATE };
    enum { NOROUND, ROUND };

    // DSPControl field positions and masks
    const uint32_t DSP_CTL_POS[DSP_NUM_FIELDS] = { 0, 7, 13, 16, 24, 14 };
    const uint32_t DSP_CTL_MASK[DSP_NUM_FIELDS] = { 0x0000003f, 0x00001f80, 0x00002000,
                                                    0x00ff0000, 0x0f000000, 0x00004000 };

    // SIMD format constants
    const uint32_t SIMD_MAX_VALS = 4; // maximum values per register
    const uint32_t SIMD_NVALS[SIMD_NUM_FMTS] = { 1, 1, 2, 4 }; // number of values in fmt
    const uint32_t SIMD_NBITS[SIMD_NUM_FMTS] = { 64, 32, 16, 8 }; // number of bits per value
    const uint32_t SIMD_LOG2N[SIMD_NUM_FMTS] = { 6, 5, 4, 3 }; // log2( bits per value )

    // DSP maximum values
    const uint64_t FIXED_L_SMAX = ULL(0x7fffffffffffffff);
    const uint64_t FIXED_W_SMAX = ULL(0x000000007fffffff);
    const uint64_t FIXED_H_SMAX = ULL(0x0000000000007fff);
    const uint64_t FIXED_B_SMAX = ULL(0x000000000000007f);
    const uint64_t FIXED_L_UMAX = ULL(0xffffffffffffffff);
    const uint64_t FIXED_W_UMAX = ULL(0x00000000ffffffff);
    const uint64_t FIXED_H_UMAX = ULL(0x000000000000ffff);
    const uint64_t FIXED_B_UMAX = ULL(0x00000000000000ff);
    const uint64_t FIXED_SMAX[SIMD_NUM_FMTS] = { FIXED_L_SMAX, FIXED_W_SMAX, FIXED_H_SMAX, FIXED_B_SMAX };
    const uint64_t FIXED_UMAX[SIMD_NUM_FMTS] = { FIXED_L_UMAX, FIXED_W_UMAX, FIXED_H_UMAX, FIXED_B_UMAX };

    // DSP minimum values
    const uint64_t FIXED_L_SMIN = ULL(0x8000000000000000);
    const uint64_t FIXED_W_SMIN = ULL(0xffffffff80000000);
    const uint64_t FIXED_H_SMIN = ULL(0xffffffffffff8000);
    const uint64_t FIXED_B_SMIN = ULL(0xffffffffffffff80);
    const uint64_t FIXED_L_UMIN = ULL(0x0000000000000000);
    const uint64_t FIXED_W_UMIN = ULL(0x0000000000000000);
    const uint64_t FIXED_H_UMIN = ULL(0x0000000000000000);
    const uint64_t FIXED_B_UMIN = ULL(0x0000000000000000);
    const uint64_t FIXED_SMIN[SIMD_NUM_FMTS] = { FIXED_L_SMIN, FIXED_W_SMIN, FIXED_H_SMIN, FIXED_B_SMIN };
    const uint64_t FIXED_UMIN[SIMD_NUM_FMTS] = { FIXED_L_UMIN, FIXED_W_UMIN, FIXED_H_UMIN, FIXED_B_UMIN };

    // DSP utility functions
    int32_t bitrev( int32_t value );
    uint64_t dspSaturate( uint64_t value, int32_t fmt, int32_t sign, uint32_t *overflow );
    uint64_t checkOverflow( uint64_t value, int32_t fmt, int32_t sign, uint32_t *overflow );
    uint64_t signExtend( uint64_t value, int32_t signpos );
    uint64_t addHalfLsb( uint64_t value, int32_t lsbpos );
    int32_t dspAbs( int32_t a, int32_t fmt, uint32_t *dspctl );
    int32_t dspAdd( int32_t a, int32_t b, int32_t fmt, int32_t saturate, int32_t sign, uint32_t *dspctl );
    int32_t dspAddh( int32_t a, int32_t b, int32_t fmt, int32_t round, int32_t sign );
    int32_t dspSub( int32_t a, int32_t b, int32_t fmt, int32_t saturate, int32_t sign, uint32_t *dspctl );
    int32_t dspSubh( int32_t a, int32_t b, int32_t fmt, int32_t round, int32_t sign );
    int32_t dspShll( int32_t a, uint32_t sa, int32_t fmt, int32_t saturate, int32_t sign, uint32_t *dspctl );
    int32_t dspShrl( int32_t a, uint32_t sa, int32_t fmt, int32_t sign );
    int32_t dspShra( int32_t a, uint32_t sa, int32_t fmt, int32_t round, int32_t sign, uint32_t *dspctl );
    int32_t dspMul( int32_t a, int32_t b, int32_t fmt, int32_t saturate, uint32_t *dspctl );
    int32_t dspMulq( int32_t a, int32_t b, int32_t fmt, int32_t saturate, int32_t round, uint32_t *dspctl );
    int32_t dspMuleu( int32_t a, int32_t b, int32_t mode, uint32_t *dspctl );
    int32_t dspMuleq( int32_t a, int32_t b, int32_t mode, uint32_t *dspctl );
    int64_t dspDpaq( int64_t dspac, int32_t a, int32_t b, int32_t ac, int32_t infmt,
                     int32_t outfmt, int32_t postsat, int32_t mode, uint32_t *dspctl );
    int64_t dspDpsq( int64_t dspac, int32_t a, int32_t b, int32_t ac, int32_t infmt,
                     int32_t outfmt, int32_t postsat, int32_t mode, uint32_t *dspctl );
    int64_t dspDpa( int64_t dspac, int32_t a, int32_t b, int32_t ac, int32_t fmt, int32_t sign, int32_t mode );
    int64_t dspDps( int64_t dspac, int32_t a, int32_t b, int32_t ac, int32_t fmt, int32_t sign, int32_t mode );
    int64_t dspMaq( int64_t dspac, int32_t a, int32_t b, int32_t ac,
                    int32_t fmt, int32_t mode, int32_t saturate, uint32_t *dspctl );
    int64_t dspMulsa( int64_t dspac, int32_t a, int32_t b, int32_t ac, int32_t fmt );
    int64_t dspMulsaq( int64_t dspac, int32_t a, int32_t b, int32_t ac, int32_t fmt, uint32_t *dspctl );
    void dspCmp( int32_t a, int32_t b, int32_t fmt, int32_t sign, int32_t op, uint32_t *dspctl );
    int32_t dspCmpg( int32_t a, int32_t b, int32_t fmt, int32_t sign, int32_t op );
    int32_t dspCmpgd( int32_t a, int32_t b, int32_t fmt, int32_t sign, int32_t op, uint32_t *dspctl );
    int32_t dspPrece( int32_t a, int32_t infmt, int32_t insign, int32_t outfmt, int32_t outsign, int32_t mode );
    int32_t dspPrecrqu( int32_t a, int32_t b, uint32_t *dspctl );
    int32_t dspPrecrq( int32_t a, int32_t b, int32_t fmt, uint32_t *dspctl );
    int32_t dspPrecrSra( int32_t a, int32_t b, int32_t sa, int32_t fmt, int32_t round );
    int32_t dspPick( int32_t a, int32_t b, int32_t fmt, uint32_t *dspctl );
    int32_t dspPack( int32_t a, int32_t b, int32_t fmt );
    int32_t dspExtr( int64_t dspac, int32_t fmt, int32_t sa, int32_t round,
                     int32_t saturate, uint32_t *dspctl );
    int32_t dspExtp( int64_t dspac, int32_t size, uint32_t *dspctl );
    int32_t dspExtpd( int64_t dspac, int32_t size, uint32_t *dspctl );

    // SIMD pack/unpack utility functions
    void simdPack( uint64_t *values_ptr, int32_t *reg, int32_t fmt );
    void simdUnpack( int32_t reg, uint64_t *values_ptr, int32_t fmt, int32_t sign );

    // DSPControl r/w utility functions
    void writeDSPControl( uint32_t *dspctl, uint32_t value, uint32_t mask );
    uint32_t readDSPControl( uint32_t *dspctl, uint32_t mask );
};

#endif
