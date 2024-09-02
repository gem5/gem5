#include <linux/kernel.h>
#include <linux/kstrtox.h>
#include <linux/module.h>
#include <linux/string.h>

#include "gem5_bridge.h"
#include "gem5_ops.h"

/* ========================================================================= *
 * Op Function Forward Declarations
 * ========================================================================= */

// static int read_file(struct gem5_op *op, char *buff);
// static int write_file(struct gem5_op *op, char *buff);
static int op_args_int(struct gem5_op *op, char *buff);


/* ========================================================================= *
 * Op Configurations
 * ========================================================================= */

struct gem5_op op_list[] =
{
    /* Always keep "bridge" as first op */
    { .name = "bridge",    /* no opcode */ /* hard-coded mmap */ },

    { .name = "exit",      .opcode = 0x21, .write = op_args_int },

//    { .name = "writefile", .opcode = 0x4F, .read  = read_file },
//    { .name = "readfile",  .opcode = 0x50, .read  = read_file },
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

/* We want to directly do a 64-bit read to our calculated memory address.
 * There is an ioread64() function, but this would both taint the register
 * state and may be disabled since KVM configuration tends to have 32-bit
 * addresses, which prevents ioread64() from being available in the kernel
 * headers. */
static u16 poke_opcode; /* Global variable for setting poke target */
#define POKE \
    *(volatile u64 __force *)(gem5_bridge_mmio + (poke_opcode << 8))

/* Necessary macro to ensure argument loading is not elided by the compiler
 * while also doing best effort to not taint the argument registers. */
#define UNUSED(_type, _var)                                          \
do {                                                                 \
    volatile _type dummy = _var; /* Ensure argument is not elided */ \
    (void)dummy;                 /* Silence unused var warnings */   \
} while (0)


/* ========================================================================= *
 * Op Function Definitions
 * ========================================================================= */

static noinline void poke_int(u64 a)
{
    UNUSED(u64, a);
    POKE;
}
static int op_args_int(struct gem5_op *op, char *buff)
{
    u64 arg0;
    int err = 0;
    err |= parse_arg_int(&buff, &arg0);

    if (err) {
        pr_err("%s: failed parsing args, expected the form \"%s\"\n",
                __func__, "<int>");
        return -EINVAL;
    }

    poke_opcode = op->opcode;
    poke_int(arg0);

    pr_info("%s: SUCCESS!\n", __func__);
    return 0;
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Noah Krim");
MODULE_DESCRIPTION("Supports gem5_bridge by defining gem5 op functionality");
