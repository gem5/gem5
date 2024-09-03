#include <linux/kernel.h>
#include <linux/kstrtox.h>
#include <linux/module.h>
#include <linux/string.h>

#include "gem5_bridge.h"
#include "gem5_ops.h"

/* ========================================================================= *
 * Op Function Forward Declarations
 * ========================================================================= */

static ssize_t read_file(struct gem5_op *op, char *buff, size_t len,
                            loff_t *off);
static ssize_t op_int(struct gem5_op *op, char *buff, size_t len,
                            loff_t *off);


/* ========================================================================= *
 * Op Configurations
 * ========================================================================= */

struct gem5_op op_list[] =
{
    /* Always keep "bridge" as first op */
    { .name = "bridge", /* no opcode */ /* hard-coded mmap */ },

    { .name = "exit",           .opcode = 0x21, .write = op_int },

    { .name = "readfile",       .opcode = 0x50, .read = read_file },
};
const int op_count = ARRAY_SIZE(op_list);


/* ========================================================================= *
 * Utility
 * ========================================================================= */

struct gem5_op *gem5_op_search(const char *dev_name)
{
    int i;
    for (i = 0; i < op_count; ++i) {
        if (!strcmp(dev_name, op_list[i].name)) {
            return &op_list[i];
        }
    }
    return NULL;
}

/* Takes a pointer to a string, extracts the first space-delimited token and
 * has outstr point to it. Advances bufp to the location after the end of this
 * token. */
static int parse_arg_str(char **bufp, char **outstr)
{
    /* Extract token */
    *outstr = NULL;
    do {
        if (!(*bufp) || !(*bufp)[0]) /* End of buffer */
            break;
        *outstr = strsep(bufp, " \t\n");
    } while ((*outstr) && !(*outstr)[0]); /* Skip long chains of whitespace */

    if (!(*outstr) || !(*outstr)[0]) {
        pr_err("%s: missing argument\n", __func__);
        return -EINVAL;
    }

    return 0;
}

/* Calls parse_arg_str to extract a token and then converts it to u64. */
static int parse_arg_int(char **bufp, u64 *outint)
{
    char *argstr;
    if (parse_arg_str(bufp, &argstr) < 0) {
        pr_err("%s: failed to get argument string\n", __func__);
        return -EINVAL;
    }

    /* Convert to int */
    if (kstrtou64(argstr, 10, outint) < 0) {
        pr_err("%s: failed parsing int from \"%s\"\n", __func__, argstr);
        return -EINVAL;
    }

    return 0;
}

/* We want to directly do a 64-bit write to our calculated memory address.
 * There is an iowrite64() function, but this would both taint the register
 * state and may be disabled since KVM configuration tends to have 32-bit
 * addresses, which prevents iowrite64() from being available in the kernel
 * headers. */
static u16 poke_opcode; /* Global variable for setting poke target */
#define POKE \
    *(volatile u64 __force *)(gem5_bridge_mmio + (poke_opcode << 8))

/* Necessary macro to ensure argument loading is not elided by the compiler
 * while also doing best effort to not taint the argument registers. */
#define USED_INT(_var)                                             \
do {                                                               \
    volatile u64 dummy = _var; /* Ensure argument is not elided */ \
    (void)dummy;               /* Silence unused var warnings */   \
} while (0)
#define USED_STR(_var)                                               \
do {                                                                 \
    char *volatile dummy = _var; /* Ensure argument is not elided */ \
    (void)dummy;                 /* Silence unused var warnings */   \
} while (0)

/* Assembly fragment to explicitly return the value placed in the result
 * register by gem5 while executing the op. */
u64 __retval; /* Dummy var for storing op result from assembly */
#if defined(__x86_64__)
    #define ASM_RETVAL asm("movq %%rax, %0" : "=r" (__retval) :: "%rax")
#elif defined(__aarch64__)
    #define ASM_RETVAL asm("mov %0, x0" : "=r" (__retval) :: "x0")
#else
    #define ASM_RETVAL pr_err("%s: unsupported architecture\n", __func__)
#endif
#define RET          \
do {                 \
    ASM_RETVAL;      \
    return __retval; \
} while (0)


/* ========================================================================= *
 * MMIO Poking Functions
 * ========================================================================= */

/*
static noinline u64 poke_nil(void)
{
    POKE; RET;
}
*/

static noinline u64 poke_int(u64 a)
{
    USED_INT(a);
    POKE; RET;
}

/*
static noinline u64 poke_int_int(u64 a, u64 b)
{
    USED_INT(a); USED_INT(b);
    POKE; RET;
}
*/

static noinline u64 poke_str_int_int(char *a, u64 b, u64 c)
{
    USED_STR(a); USED_INT(b); USED_INT(c);
    POKE; RET;
}


/* ========================================================================= *
 * Op Function Definitions - Specialized
 * ========================================================================= */

static ssize_t read_file(struct gem5_op *op, char *buff, size_t len,
                            loff_t *off)
{
    ssize_t bytes_read = 0;

    poke_opcode = op->opcode;
    bytes_read = poke_str_int_int(buff, len, *off);

    if (bytes_read > 0)
        *off += bytes_read;
    return bytes_read;
}


/* ========================================================================= *
 * Op Function Definitions - General
 * ========================================================================= */

#define OP_ARG_ERR(_err, _argfmt)                                     \
    if (_err) {                                                       \
        pr_err("%s: failed parsing args, expected the form \"%s\"\n", \
                __func__, _argfmt);                                   \
        return -EINVAL;                                               \
    }

static ssize_t op_int(struct gem5_op *op, char *buff, size_t len, loff_t *off)
{
    u64 arg0;
    int err = parse_arg_int(&buff, &arg0);
    OP_ARG_ERR(err, "<int>");

    poke_opcode = op->opcode;
    (void)poke_int(arg0);
    return len; /* unconditionally consume entire input */
}
/*
static ssize_t op_int_int(struct gem5_op *op, char *buff, size_t len,
                            loff_t *off)
{
    u64 arg0, arg1;
    int err = parse_arg_int(&buff, &arg0)
            | parse_arg_int(&buff, &arg1);
    OP_ARG_ERR(err, "<int> <int>");

    poke_opcode = op->opcode;
    (void)poke_int_int(arg0, arg1);
    return len;
}
*/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Noah Krim");
MODULE_DESCRIPTION("Supports gem5_bridge by defining gem5 op functionality");
