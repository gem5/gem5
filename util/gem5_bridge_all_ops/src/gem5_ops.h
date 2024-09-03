#ifndef GEM5_OPS_H
#define GEM5_OPS_H

#include <linux/types.h>

/* Types for op function pointers */
struct gem5_op; /* Forward decl */
typedef ssize_t (*read_f)(struct gem5_op *op, char *buff, size_t len,
                            loff_t *off);
typedef ssize_t (*write_f)(struct gem5_op *op, char *buff, size_t len,
                            loff_t *off);

/* Op configuration struct */
struct gem5_op
{
    const char *name;
    u8 opcode;
    read_f read;
    write_f write;
};

/* Exported op configuration list */
extern struct gem5_op op_list[];
extern const int op_count;

/* Utility funcitons */
struct gem5_op *gem5_op_search(const char *dev_name);

#endif /* GEM5_OPS_H */
