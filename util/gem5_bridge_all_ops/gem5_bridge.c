#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/uaccess.h>

/* ========================================================================= *
 * Constants
 * ========================================================================= */

/* Device information */
#define DEV_NAME "gem5_bridge"
#define DEV_MODE ((umode_t)0666) /* All users have RW access to this device */

/* Physical address for base of gem5ops MMIO range */
#ifndef GEM5OPS_BASE
#define GEM5OPS_BASE 0xffff0000 /* default, also used for x86 */
#endif

/* Size of gem5ops MMIO range */
#ifndef GEM5OPS_SIZE
#define GEM5OPS_SIZE 0x10000
#endif

/* ========================================================================= *
 * Global Variables
 * ========================================================================= */

static void __iomem *gem5_bridge_mmio;

/* ========================================================================= *
 * File Operations
 * ========================================================================= */

static int gem5_bridge_open(struct inode *inode, struct file *file)
{
    pr_info("gem5_bridge_open: SUCCESS!\n");
    return 0;
}

static int gem5_bridge_release(struct inode *inode, struct file *file)
{
    pr_info("gem5_bridge_release: SUCCESS!\n");
    return 0;
}

static int gem5_bridge_mmap(struct file *file, struct vm_area_struct *vma)
{
    unsigned long vm_size = vma->vm_end - vma->vm_start + vma->vm_pgoff;
    unsigned long gem5_bridge_pfn = GEM5OPS_BASE >> PAGE_SHIFT;

    if (vm_size > GEM5OPS_SIZE) {
        pr_info("gem5_bridge_mmap: requested memory range is too large\n");
        return -EINVAL;
    }

    /* Map the virtual memory area to our gem5 ops MMIO range */
    if (io_remap_pfn_range(
            vma, vma->vm_start, gem5_bridge_pfn, vm_size, vma->vm_page_prot
    )) {
        pr_info("gem5_bridge_mmap: failed to remap vm area to phys addr\n");
        return -EAGAIN;
    }

    pr_info("gem5_bridge_mmap: SUCCESS!\n");
    return 0;
}

#if 0 /* !!! DISABLED !!! */
/* This function is disabled, but still included to show how one can directly
 * cause this driver to trigger MMIO traps. Included so far is the ability to
 * detect when "exit" has been written to the device node and, once detected,
 * a memory request is made to the address that gem5 will detect as an m5exit
 * operation. Currently, there is no implementation of the m5op arguments which
 * prevents this from being functional in this state as the delay argument will
 * be undefined and could cause the simulation to crash. */
static ssize_t gem5_bridge_write(struct file *file, const char *ubuf,
                                    size_t count, loff_t *offp)
{
    static char kbuf[1024];

    if (copy_from_user(kbuf, ubuf, count) > 0) {
        pr_err("gem5_bridge_write: unknown error when copying user buffer\n");
        return -EINVAL; /* @TODO: find better error code */
    }

    if (!strncmp(kbuf, "exit", 5)) {
        pr_info("gem5_bridge_write: EXIT command\n");
        /* @TODO: setup arguments, right now it does not work */
        /* Kernel may have been built for 32-bit addresses, in which case we
         * do not have access to `ioread64`. However, we know that this MMIO
         * access will reach KVM and it will expect a 64-bit request so we
         * just force it explicitly here. */
        *(volatile u64 __force *)(gem5_bridge_mmio + 0x2100);
    } else {
        pr_err("gem5_bridge_write: unknown command = %s\n", ubuf);
        return -EINVAL;
    }

    pr_info("gem5_bridge_write: SUCCESS!\n");
    return count;
}
#endif /* !!! DISABLED !!! */

static struct file_operations gem5_bridge_fops = {
    .owner      = THIS_MODULE,
    .open       = gem5_bridge_open,
    .release    = gem5_bridge_release,
    .mmap       = gem5_bridge_mmap,
//    .write      = gem5_bridge_write,
};

/* ========================================================================= *
 * Module Initialization + Destruction
 * ========================================================================= */

static struct miscdevice gem5_bridge_miscdevice = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = DEV_NAME,
    .mode = DEV_MODE,
    .fops = &gem5_bridge_fops,
};

static int __init gem5_bridge_init(void)
{
    /* Make virtual mapping to physical MMIO address */
    gem5_bridge_mmio = ioremap(GEM5OPS_BASE, GEM5OPS_SIZE);
    if (gem5_bridge_mmio < 0) {
        pr_err("gem5_bridge_init: failed to map gem5ops MMIO range\n");
        goto error_mmio;
    }

    /* Register misc device */
    if (misc_register(&gem5_bridge_miscdevice) < 0) {
        pr_err("gem5_bridge_init: failed to register misc device\n");
        goto error_misc_register;
    }

    pr_info("gem5_bridge_init: SUCCESS!\n");
    return 0;

error_misc_register:
    iounmap(gem5_bridge_mmio);
error_mmio:
    return -1;
}

static void __exit gem5_bridge_exit(void)
{
    /* Undo all allocations and registrations in reverse order */
    misc_deregister(&gem5_bridge_miscdevice);
    iounmap(gem5_bridge_mmio);
    pr_info("gem5_bridge_exit: SUCCESS!\n");
}

module_init(gem5_bridge_init);
module_exit(gem5_bridge_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Noah Krim");
MODULE_DESCRIPTION("Diver that provides user-space mapping to MMIO range");
