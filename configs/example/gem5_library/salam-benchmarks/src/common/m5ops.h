#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define ENABLED 1
#if defined(__arm__)
static void
m5_checkpoint(void)
{
    __asm__ __volatile__("mov r0, #0; mov r1, #0;"
                         " mov r2, #0; mov r3, #0;"
                         " .inst 0xEE000110 | (0x43 << 16);");
};
static void
m5_dump_stats(void)
{
    __asm__ __volatile__("mov r0, #0; mov r1, #0;"
                         " mov r2, #0; mov r3, #0;"
                         " .inst 0xEE000110 | (0x41 << 16);");
};
static void
m5_exit(void)
{
    __asm__ __volatile__("mov r0, #0; mov r1, #0;"
                         " .inst 0xEE000110 | (0x21 << 16);");
};
static void
m5_fail_1(void)
{
    __asm__ __volatile__("mov r0, #0; mov r1, #0;"
                         " mov r2, #1; mov r3, #0;"
                         " .inst 0xEE000110 | (0x22 << 16);");
};
static void
m5_reset_stats(void)
{
    __asm__ __volatile__("mov r0, #0; mov r1, #0;"
                         " mov r2, #0; mov r3, #0;"
                         " .inst 0xEE000110 | (0x40 << 16);");
};
#elif defined(__aarch64__)
static void
m5_checkpoint(void)
{
    __asm__ __volatile__("mov x0, #0; mov x1, #0;"
                         " .inst 0xFF000110 | (0x43 << 16);");
};
static void
m5_dump_stats(void)
{
    __asm__ __volatile__("mov x0, #0; mov x1, #0;"
                         " .inst 0xFF000110 | (0x41 << 16);");
};
static void
m5_exit(void)
{
    __asm__ __volatile__("mov x0, #0;"
                         " .inst 0XFF000110 | (0x21 << 16);");
};
static void
m5_fail_1(void)
{
    __asm__ __volatile__("mov x0, #0; mov x1, #1;"
                         " .inst 0xFF000110 | (0x22 << 16);");
};
static void
m5_reset_stats(void)
{
    __asm__ __volatile__("mov x0, #0; mov x1, #0;"
                         " .inst 0XFF000110 | (0x40 << 16);");
};
#else
#undef ENABLED
#define ENABLED 0
#endif
