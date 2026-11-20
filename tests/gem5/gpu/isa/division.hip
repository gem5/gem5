/*
 * Copyright (c) 2026 Boyana Norris
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "test_common.hh"

namespace
{

constexpr unsigned NumLanes = 64;
constexpr uint32_t Sentinel = 0xdeadbeef;
constexpr uint64_t EvenLanes = 0x5555555555555555ULL;
constexpr uint64_t OddLanes = 0xaaaaaaaaaaaaaaaaULL;

enum Result
{
    FmasClear,
    FmasSet,
    FmasScaleUp,
    FmasMixed,
    FmasMasked,
    FmasEmpty,
    ScaleDen,
    ScaleDenVcc,
    ScaleNum,
    ScaleNumVcc,
    FixupQuotient,
    Quotient,
    NumResults
};

struct Input
{
    uint32_t scaleA, scaleB, divA, divB;
};

struct ScaleCase
{
    uint32_t a, b, den;
    bool denVcc;
    uint32_t num;
    bool numVcc;
};

// Measured on an MI300X (gfx942). The cases cover
// signed exponent differences 95/96, exponent(S1) 252/253, exponent(S2) 24/25,
// zero, subnormal, normal, infinite and NaN operands.
const ScaleCase ScaleCases[] = {
    {0x3f800000, 0x7f800000, 0x7f800000, true, 0x3f800000, true},
    {0x3f800000, 0x7fc00000, 0x7fc00000, true, 0x3f800000, true},
    {0x3f800000, 0x7f812345, 0x7f812345, true, 0x3f800000, true},
    {0x7f800000, 0x3f800000, 0x5f800000, true, 0x7f800000, true},
    {0x00000000, 0x00000000, 0xffc00000, false, 0xffc00000, false},
    {0x00000000, 0x00000001, 0xffc00000, false, 0xffc00000, false},
    {0x00000000, 0x00800000, 0xffc00000, false, 0xffc00000, false},
    {0x00000000, 0x3f800000, 0xffc00000, true, 0xffc00000, true},
    {0x00000000, 0x7f7fffff, 0xffc00000, true, 0xffc00000, true},
    {0x00000000, 0x7f800000, 0xffc00000, true, 0xffc00000, true},
    {0x00000000, 0x7fc00000, 0xffc00000, true, 0xffc00000, true},
    {0x00000000, 0x7f812345, 0xffc00000, true, 0xffc00000, true},
    {0x00000001, 0x00000001, 0x15000000, false, 0x15000000, false},
    {0x00000001, 0x3f800000, 0x3f800000, true, 0x15000000, true},
    {0x00000001, 0x7f7fffff, 0x5f7fffff, true, 0x00000001, true},
    {0x3f800000, 0x00000000, 0xffc00000, true, 0xffc00000, true},
    {0x0c000000, 0x00000000, 0xffc00000, false, 0xffc00000, false},
    {0xf974b71b, 0x49f401be, 0x49f401be, false, 0xf974b71b, false},
    {0xf58e78d7, 0x45cfc04a, 0x65cfc04a, true, 0xf58e78d7, true},
    {0xb2b82aef, 0xe23b5b82, 0xe23b5b82, false, 0xb2b82aef, false},
    {0x91bf287f, 0xc1ad9bd2, 0xc1ad9bd2, true, 0xb1bf287f, true},
    {0xfc4d626b, 0x7e0694ce, 0x7e0694ce, false, 0xfc4d626b, false},
    {0xeac99ec1, 0xfec5e92c, 0xdec5e92c, false, 0xcac99ec1, false},
    {0x8caa1365, 0x09a92d80, 0x09a92d80, false, 0x8caa1365, false},
};

struct QuotientCase
{
    uint32_t a, b, q;
};

// Full division sequence results measured on an MI300X.
const QuotientCase QuotientCases[] = {
    {0x80000000, 0x3f800000, 0x80000000}, {0x3fc00000, 0x3f800000, 0x3fc00000},
    {0x3f800000, 0xbf800000, 0xbf800000}, {0x50464bcd, 0xff800000, 0x80000000},
    {0x00000000, 0x00000000, 0xffc00000}, {0x00000000, 0x3f800000, 0x00000000},
    {0x00000001, 0x00000001, 0x3f800000}, {0x00000001, 0x3f800000, 0x00000001},
    {0x00000001, 0x40000000, 0x00000000}, {0x00800000, 0x00800000, 0x3f800000},
    {0x00800000, 0x3fc00000, 0x00555555}, {0x3f800000, 0x00000000, 0x7f800000},
    {0x3f800000, 0x00800000, 0x7e800000}, {0x5f800000, 0x00800000, 0x7f800000},
    {0x5f800000, 0x7f800000, 0x00000000}, {0x5f800000, 0x7fc00000, 0x7fc00000},
    {0x7f800000, 0x00000001, 0x7f800000}, {0x7f800000, 0x7f800000, 0xffc00000},
    {0x7fc00000, 0x00000000, 0x7fc00000}, {0x7fc00000, 0x00000001, 0x7fc00000},
    {0x7f812345, 0x7f812345, 0x7fc12345}, {0xb2b82aef, 0xe23b5b82, 0x0ffba441},
    {0x91bf287f, 0xc1ad9bd2, 0x0f8cf068}, {0x800d3089, 0xbbd20054, 0x0280a10e},
    {0xff84a155, 0xcfc002b0, 0xffc4a155}, {0x4f9f6c54, 0x7f800000, 0x00000000},
};

__device__ float
fmas(float s0, float s1, float s2, uint64_t mask)
{
    float result = __uint_as_float(Sentinel);
    asm volatile("s_mov_b64 vcc, %4\n\t"
                 "s_nop 4\n\t"
                 "v_div_fmas_f32 %0, %1, %2, %3\n\t"
                 : "+&v"(result)
                 : "v"(s0), "v"(s1), "v"(s2), "s"(mask)
                 : "vcc");
    return result;
}

__device__ uint32_t
maskedFmas(uint64_t mask)
{
    float result = __uint_as_float(Sentinel);
    uint64_t saved_exec;
    // EXEC has its original value when the asm statement finishes.
    // Restore EXEC before returning, so inactive lanes also store their
    // destination register and the host can check that it was preserved.
    asm volatile("s_mov_b64 vcc, -1\n\t"
                 "s_and_saveexec_b64 %1, %5\n\t"
                 "s_nop 4\n\t"
                 "v_div_fmas_f32 %0, %2, %3, %4\n\t"
                 "s_mov_b64 exec, %1\n\t"
                 : "+&v"(result), "=&s"(saved_exec)
                 : "v"(0.25f), "v"(2.0f), "v"(1.0f), "s"(mask)
                 : "vcc", "scc");
    return __float_as_uint(result);
}

__global__ void
division(const Input *inputs, uint32_t *outputs)
{
    const unsigned lane = threadIdx.x;
    uint32_t *out = outputs + lane * NumResults;
    out[FmasClear] = __float_as_uint(fmas(0.25f, 2.0f, 1.0f, 0));
    out[FmasSet] = __float_as_uint(fmas(0.25f, 2.0f, 1.0f, ~0ULL));
    out[FmasScaleUp] = __float_as_uint(fmas(0.25f, 2.0f, 2.0f, ~0ULL));
    out[FmasMixed] = __float_as_uint(fmas(0.25f, 2.0f, 1.0f, OddLanes));
    out[FmasMasked] = maskedFmas(EvenLanes);
    out[FmasEmpty] = maskedFmas(0);

    const float a = __uint_as_float(inputs[lane].scaleA);
    const float b = __uint_as_float(inputs[lane].scaleB);
    float den, num;
    uint32_t den_vcc, num_vcc;
    uint64_t mask;
    asm volatile("v_div_scale_f32 %0, %1, %2, %2, %3\n\t"
                 "s_nop 4\n\t"
                 : "=v"(den), "=&s"(mask)
                 : "v"(b), "v"(a));
    asm volatile("v_cndmask_b32_e64 %0, 0, 1, %1" : "=v"(den_vcc) : "s"(mask));
    asm volatile("v_div_scale_f32 %0, vcc, %2, %3, %2\n\t"
                 "s_nop 4\n\t"
                 "v_cndmask_b32_e64 %1, 0, 1, vcc\n\t"
                 : "=&v"(num), "=&v"(num_vcc)
                 : "v"(a), "v"(b)
                 : "vcc");
    out[ScaleDen] = __float_as_uint(den);
    out[ScaleDenVcc] = den_vcc;
    out[ScaleNum] = __float_as_uint(num);
    out[ScaleNumVcc] = num_vcc;

    // FIXUP must use its supplied quotient rather than recomputing 1 / 2.
    float fixup;
    asm volatile("v_div_fixup_f32 %0, %1, %2, %3"
                 : "=v"(fixup)
                 : "v"(1.5f), "v"(2.0f), "v"(1.0f));
    out[FixupQuotient] = __float_as_uint(fixup);

    const float div_a = __uint_as_float(inputs[lane].divA);
    const float div_b = __uint_as_float(inputs[lane].divB);
    float sb, sa, r, t1, t2, q;
    uint64_t den_mask;
    // The two DIV_SCALE operations produce the register values and VCC
    // consumed by the reciprocal, refinement and final FMAS/FIXUP.
    asm volatile("v_div_scale_f32 %0, %6, %8, %8, %7\n\t"
                 "v_rcp_f32 %2, %0\n\t"
                 "v_div_scale_f32 %1, vcc, %7, %8, %7\n\t"
                 "v_fma_f32 %3, -%0, %2, 1.0\n\t"
                 "v_fmac_f32_e32 %2, %3, %2\n\t"
                 "v_mul_f32_e32 %3, %1, %2\n\t"
                 "v_fma_f32 %4, -%0, %3, %1\n\t"
                 "v_fmac_f32_e32 %3, %4, %2\n\t"
                 "v_fma_f32 %0, -%0, %3, %1\n\t"
                 "v_div_fmas_f32 %0, %0, %2, %3\n\t"
                 "v_div_fixup_f32 %5, %0, %8, %7\n\t"
                 : "=&v"(sb), "=&v"(sa), "=&v"(r), "=&v"(t1), "=&v"(t2),
                   "=v"(q), "=&s"(den_mask)
                 : "v"(div_a), "v"(div_b)
                 : "vcc");
    out[Quotient] = __float_as_uint(q);
}

} // anonymous namespace

int
main()
{
    requireMi300x();
    constexpr unsigned NumScaleCases =
        sizeof(ScaleCases) / sizeof(ScaleCases[0]);
    constexpr unsigned NumQuotientCases =
        sizeof(QuotientCases) / sizeof(QuotientCases[0]);
    static_assert(NumScaleCases <= NumLanes);
    static_assert(NumQuotientCases <= NumLanes);
    Input inputs[NumLanes];
    uint32_t outputs[NumLanes * NumResults];
    for (unsigned lane = 0; lane < NumLanes; ++lane) {
        const auto &scale = ScaleCases[lane % NumScaleCases];
        const auto &quotient = QuotientCases[lane % NumQuotientCases];
        inputs[lane] = {scale.a, scale.b, quotient.a, quotient.b};
    }

    Input *device_inputs;
    uint32_t *device_outputs;
    HIP_CHECK(hipMalloc(&device_inputs, sizeof(inputs)));
    HIP_CHECK(hipMalloc(&device_outputs, sizeof(outputs)));
    HIP_CHECK(hipMemcpy(device_inputs, inputs, sizeof(inputs),
                        hipMemcpyHostToDevice));
    division<<<1, NumLanes>>>(device_inputs, device_outputs);
    HIP_CHECK(hipGetLastError());
    HIP_CHECK(hipDeviceSynchronize());
    HIP_CHECK(hipMemcpy(outputs, device_outputs, sizeof(outputs),
                        hipMemcpyDeviceToHost));
    HIP_CHECK(hipFree(device_inputs));
    HIP_CHECK(hipFree(device_outputs));

    unsigned checks = 0, errors = 0;
    for (unsigned lane = 0; lane < NumLanes; ++lane) {
        const auto &scale = ScaleCases[lane % NumScaleCases];
        const auto &quotient = QuotientCases[lane % NumQuotientCases];
        // With VCC set, FMAS scales by 2^64 if the biased exponent of S2
        // exceeds 127, otherwise by 2^-64; VCC clear leaves it unscaled.
        // Mixed lanes select those results; inactive lanes keep the sentinel.
        const uint32_t expected[NumResults] = {
            0x3fc00000, 0x1fc00000, 0x60200000,
            lane % 2 ? 0x1fc00000U : 0x3fc00000U,
            lane % 2 ? Sentinel : 0x1fc00000U, Sentinel, scale.den,
            scale.denVcc, scale.num, scale.numVcc,
            // FIXUP preserves the supplied quotient for these normal inputs.
            0x3fc00000, quotient.q};
        const char *names[NumResults] = {
            "fmas-clear",  "fmas-set",      "fmas-scale-up",  "fmas-mixed",
            "fmas-masked", "fmas-empty",    "scale-den",      "scale-den-vcc",
            "scale-num",   "scale-num-vcc", "fixup-quotient", "division"};
        for (unsigned result = 0; result < NumResults; ++result) {
            expectBits(names[result], lane,
                       outputs[lane * NumResults + result], expected[result],
                       checks, errors);
        }
    }
    return finish("division", checks, errors);
}
