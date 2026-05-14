/*
 * emissary_effect_test.c
 *
 * A synthetic instruction-cache benchmark for gem5 EMISSARY-style
 * replacement policies.  The benchmark alternates between:
 *
 *   1. A reusable hot instruction footprint: larger than a typical L1I,
 *      smaller than a typical L2.
 *   2. A large one-pass cold instruction footprint: intended to evict useful
 *      hot lines under plain LRU.
 *
 * EMISSARY should help when hot lines repeatedly cause front-end starvation
 * and are later threatened by cold instruction-cache pollution.
 *
 * Build examples:
 *   gcc -O2 -std=c11 -fno-inline -fno-reorder-functions \
 *       -o emissary_effect_test emissary_effect_test.c
 *
 * AArch64 cross build example:
 *   aarch64-linux-gnu-gcc -O2 -static -std=c11 -fno-inline \
 *       -fno-reorder-functions -o emissary_effect_test.aarch64 \
 *       emissary_effect_test.c
 *
 * Optional tuning at compile time:
 *   -DEMISSARY_HOT_PAD_REPT=512
 *   -DEMISSARY_COLD_PAD_REPT=2048
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef USE_GEM5_M5OPS
#include <gem5/m5ops.h>
#endif

#ifndef EMISSARY_HOT_PAD_REPT
#define EMISSARY_HOT_PAD_REPT 512
#endif

#ifndef EMISSARY_COLD_PAD_REPT
#define EMISSARY_COLD_PAD_REPT 2048
#endif

#define STR2(x) #x
#define STR(x) STR2(x)

#if defined(__GNUC__) || defined(__clang__)
#define NOINLINE __attribute__((noinline))
#define ALIGNED64 __attribute__((aligned(64)))
#else
#define NOINLINE
#define ALIGNED64
#endif

#define ASM_NOPS(count) \
    __asm__ volatile(".rept " STR(count) "\n\t" \
                     "nop\n\t" \
                     ".endr\n\t" ::: "memory")

typedef uint64_t (*kernel_fn)(uint64_t);

static volatile uint64_t sink;

static NOINLINE ALIGNED64 uint64_t
mix(uint64_t x, uint64_t salt)
{
    x ^= salt + 0x9e3779b97f4a7c15ULL + (x << 6) + (x >> 2);
    x *= 0xbf58476d1ce4e5b9ULL;
    x ^= x >> 31;
    return x;
}

#define DEFINE_HOT(n) \
    static NOINLINE ALIGNED64 uint64_t hot_##n(uint64_t x) \
    { \
        ASM_NOPS(EMISSARY_HOT_PAD_REPT); \
        return mix(x, 0x100000001b3ULL + (uint64_t)(n)); \
    }

#define DEFINE_COLD(n) \
    static NOINLINE ALIGNED64 uint64_t cold_##n(uint64_t x) \
    { \
        ASM_NOPS(EMISSARY_COLD_PAD_REPT); \
        return mix(x, 0xcbf29ce484222325ULL + ((uint64_t)(n) << 1)); \
    }

#define HOT_LIST(X) \
    X(0) X(1) X(2) X(3) X(4) X(5) X(6) X(7) \
    X(8) X(9) X(10) X(11) X(12) X(13) X(14) X(15) \
    X(16) X(17) X(18) X(19) X(20) X(21) X(22) X(23) \
    X(24) X(25) X(26) X(27) X(28) X(29) X(30) X(31) \
    X(32) X(33) X(34) X(35) X(36) X(37) X(38) X(39) \
    X(40) X(41) X(42) X(43) X(44) X(45) X(46) X(47) \
    X(48) X(49) X(50) X(51) X(52) X(53) X(54) X(55) \
    X(56) X(57) X(58) X(59) X(60) X(61) X(62) X(63)

#define COLD_LIST(X) \
    X(0) X(1) X(2) X(3) X(4) X(5) X(6) X(7) \
    X(8) X(9) X(10) X(11) X(12) X(13) X(14) X(15) \
    X(16) X(17) X(18) X(19) X(20) X(21) X(22) X(23) \
    X(24) X(25) X(26) X(27) X(28) X(29) X(30) X(31) \
    X(32) X(33) X(34) X(35) X(36) X(37) X(38) X(39) \
    X(40) X(41) X(42) X(43) X(44) X(45) X(46) X(47) \
    X(48) X(49) X(50) X(51) X(52) X(53) X(54) X(55) \
    X(56) X(57) X(58) X(59) X(60) X(61) X(62) X(63) \
    X(64) X(65) X(66) X(67) X(68) X(69) X(70) X(71) \
    X(72) X(73) X(74) X(75) X(76) X(77) X(78) X(79) \
    X(80) X(81) X(82) X(83) X(84) X(85) X(86) X(87) \
    X(88) X(89) X(90) X(91) X(92) X(93) X(94) X(95) \
    X(96) X(97) X(98) X(99) X(100) X(101) X(102) X(103) \
    X(104) X(105) X(106) X(107) X(108) X(109) X(110) X(111) \
    X(112) X(113) X(114) X(115) X(116) X(117) X(118) X(119) \
    X(120) X(121) X(122) X(123) X(124) X(125) X(126) X(127) \
    X(128) X(129) X(130) X(131) X(132) X(133) X(134) X(135) \
    X(136) X(137) X(138) X(139) X(140) X(141) X(142) X(143) \
    X(144) X(145) X(146) X(147) X(148) X(149) X(150) X(151) \
    X(152) X(153) X(154) X(155) X(156) X(157) X(158) X(159) \
    X(160) X(161) X(162) X(163) X(164) X(165) X(166) X(167) \
    X(168) X(169) X(170) X(171) X(172) X(173) X(174) X(175) \
    X(176) X(177) X(178) X(179) X(180) X(181) X(182) X(183) \
    X(184) X(185) X(186) X(187) X(188) X(189) X(190) X(191) \
    X(192) X(193) X(194) X(195) X(196) X(197) X(198) X(199) \
    X(200) X(201) X(202) X(203) X(204) X(205) X(206) X(207) \
    X(208) X(209) X(210) X(211) X(212) X(213) X(214) X(215) \
    X(216) X(217) X(218) X(219) X(220) X(221) X(222) X(223) \
    X(224) X(225) X(226) X(227) X(228) X(229) X(230) X(231) \
    X(232) X(233) X(234) X(235) X(236) X(237) X(238) X(239) \
    X(240) X(241) X(242) X(243) X(244) X(245) X(246) X(247) \
    X(248) X(249) X(250) X(251) X(252) X(253) X(254) X(255)

HOT_LIST(DEFINE_HOT)
COLD_LIST(DEFINE_COLD)

#define FN_ENTRY(prefix, n) prefix##_##n,
#define HOT_ENTRY(n) FN_ENTRY(hot, n)
#define COLD_ENTRY(n) FN_ENTRY(cold, n)

static kernel_fn hot_fns[] = { HOT_LIST(HOT_ENTRY) };
static kernel_fn cold_fns[] = { COLD_LIST(COLD_ENTRY) };

static NOINLINE uint64_t
run_hot(uint64_t x, int passes)
{
    const int n = (int)(sizeof(hot_fns) / sizeof(hot_fns[0]));

    for (int p = 0; p < passes; ++p) {
        for (int i = 0; i < n; ++i) {
            x = hot_fns[i](x);
        }
    }

    return x;
}

static NOINLINE uint64_t
run_cold(uint64_t x, int stride)
{
    const int n = (int)(sizeof(cold_fns) / sizeof(cold_fns[0]));

    if (stride < 1) {
        stride = 1;
    }

    for (int base = 0; base < stride; ++base) {
        for (int i = base; i < n; i += stride) {
            x = cold_fns[i](x);
        }
    }

    return x;
}

static int
parse_int_arg(const char *s, int fallback)
{
    char *end = NULL;
    long v = strtol(s, &end, 0);
    if (end == s || *end != '\0' || v <= 0 || v > 1000000000L) {
        return fallback;
    }
    return (int)v;
}

int
main(int argc, char **argv)
{
    int rounds = 80;
    int hot_passes = 8;
    int cold_stride = 1;

    if (argc > 1) {
        rounds = parse_int_arg(argv[1], rounds);
    }
    if (argc > 2) {
        hot_passes = parse_int_arg(argv[2], hot_passes);
    }
    if (argc > 3) {
        cold_stride = parse_int_arg(argv[3], cold_stride);
    }

    uint64_t x = 0x123456789abcdef0ULL;

    x = run_hot(x, hot_passes * 2);
    x = run_cold(x, cold_stride);
    sink = x;

#ifdef USE_GEM5_M5OPS
    m5_reset_stats(0, 0);
    m5_work_begin(1, 0);
#endif

    for (int r = 0; r < rounds; ++r) {
        x = run_hot(x + (uint64_t)r, hot_passes);
        x = run_cold(x, cold_stride);
        x = run_hot(x ^ (uint64_t)r, hot_passes);
        sink = x;
    }

#ifdef USE_GEM5_M5OPS
    m5_dump_stats(0, 0);
#endif

    printf("emissary_effect_test rounds=%d hot_passes=%d cold_stride=%d "
           "hot_fns=%zu cold_fns=%zu checksum=%llu\n",
           rounds, hot_passes, cold_stride,
           sizeof(hot_fns) / sizeof(hot_fns[0]),
           sizeof(cold_fns) / sizeof(cold_fns[0]),
           (unsigned long long)sink);

    return (sink == 0) ? 1 : 0;
}
