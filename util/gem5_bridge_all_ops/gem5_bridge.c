#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kdev_t.h>
#include <linux/kernel.h>
#include <linux/kstrtox.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/version.h>

/* ========================================================================= *
 * gem5_bridge Device Configuration
 * ========================================================================= */

struct gem5_bridge_dev_config
{
    /* @TODO: Still needs required fields for functionality */
    const char *name;
    u8 opcode;
    int num_args;
};
static struct gem5_bridge_dev_config config_list[] = {
    { .name = "bridge" /* bridge has no config, can only mmap */ },
    { .name = "exit", .opcode=0x21, .num_args = 1 },
};
enum gem5_bridge_op
{
    /* Op enum values must match connected device indices in `config_list` */
    GEM5_EXIT = 1,
};

/* Just helpful for printing */
#define PATH "/dev/gem5/"


/* ========================================================================= *
 * Constants
 * ========================================================================= */

/* Device information */
#define DEV_NAME "gem5_bridge"
#define DEV_MODE ((umode_t)0666) /* All users have RW access to this device */
#define DEV_COUNT ARRAY_SIZE(config_list)

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

/* gem5 bridge */
static void __iomem *gem5_bridge_mmio;
static u16 gem5_bridge_nextop;

/* Device information */
static dev_t dev_number;
static struct class *dev_class;
static struct cdev cdev_list[DEV_COUNT];


/* ========================================================================= *
 * Utility
 * ========================================================================= */

static int gem5_bridge_search(const char *dev_name) {
    int i;
    for (i = 0; i < DEV_COUNT; ++i) {
        if (!strcmp(dev_name, config_list[i].name)) {
            return i;
        }
    }
    return -1;
}

/* Returns a pointer to the first buffer position after the parsed ints,
 * or NULL on error */
static char *gem5_bridge_parse_ints(int numargs, char *buff, uint64_t *outargs)
{
    int i;
    char *bufp = buff; /* Need a local pointer that can be modified */
    char *tok;
    for (i = 0; i < numargs; ++i) {
        do {
            if (!bufp[0]) /* End of buffer */
                break;
            tok = strsep(&bufp, " ");
        } while (!tok[0]); /* Skip long chains of whitespace */
        if (kstrtou64(tok, 10, &outargs[i]) < 0) {
            pr_err("%s: failed parsing int from \"%s\"\n", __func__, tok);
            return NULL;
        }
    }
    if (i < numargs)
        return NULL;
    return bufp;
}

/* Kernel may have been built for 32-bit addresses, in which case we
 * do not have access to `ioread64`. However, we know that this MMIO
 * access will reach KVM and it will expect a 64-bit request so we
 * just force it explicitly here. */
#define POKE \
    *(volatile u64 __force *)(gem5_bridge_mmio + (gem5_bridge_nextop << 8))

static void gem5_bridge_poke_int(int a) {
    POKE;
}


/* ========================================================================= *
 * File Operations
 * ========================================================================= */

static int gem5_bridge_open(struct inode *inode, struct file *file)
{
    pr_info("%s: SUCCESS!\n", __func__);
    return 0;
}

static int gem5_bridge_release(struct inode *inode, struct file *file)
{
    pr_info("%s: SUCCESS!\n", __func__);
    return 0;
}

static int gem5_bridge_mmap(struct file *filp, struct vm_area_struct *vma)
{
    unsigned long vm_size = vma->vm_end - vma->vm_start + vma->vm_pgoff;
    unsigned long gem5_bridge_pfn = GEM5OPS_BASE >> PAGE_SHIFT;
    const char *file_name = filp->f_path.dentry->d_iname;
    int dev_i = gem5_bridge_search(file_name);

    if (dev_i < 0) {
        pr_err("%s: unsupported device node "PATH"%s\n", __func__, file_name);
        return -EINVAL;
    }

    if (dev_i != 0) {
        pr_err("%s: mmap unsupported for "PATH"%s, only works with /dev/%s\n",
                __func__, config_list[dev_i].name, config_list[0].name);
        return -EINVAL;
    }

    if (vm_size > GEM5OPS_SIZE) {
        pr_err("%s: requested memory range is too large\n", __func__);
        return -EINVAL;
    }

    /* Map the virtual memory area to our gem5 ops MMIO range */
    if (io_remap_pfn_range(vma, vma->vm_start, gem5_bridge_pfn,
                            vm_size, vma->vm_page_prot))
    {
        pr_err("%s: failed to remap vm area to phys addr\n", __func__);
        return -EAGAIN;
    }

    pr_info("%s: SUCCESS!\n", __func__);
    return 0;
}

/* This function is disabled, but still included to show how one can directly
 * cause this driver to trigger MMIO traps. Included so far is the ability to
 * detect when "exit" has been written to the device node and, once detected,
 * a memory request is made to the address that gem5 will detect as an m5exit
 * operation. Currently, there is no implementation of the m5op arguments which
 * prevents this from being functional in this state as the delay argument will
 * be undefined and could cause the simulation to crash. */
#define WRITE_BUFF_SIZE 1024
static ssize_t gem5_bridge_write(struct file *filp, const char *ubuff,
                                    size_t count, loff_t *offp)
{
    static char kbuff[WRITE_BUFF_SIZE];
    static u64 intargs[2];
    char *kbufp = kbuff;

    const char *file_name = filp->f_path.dentry->d_iname;
    int dev_i = gem5_bridge_search(file_name);

    if (count >= WRITE_BUFF_SIZE) {
        pr_err("%s: write content reaches or exceeds buffer size of %d\n",
                __func__, WRITE_BUFF_SIZE);
        return -EINVAL;
    }

    if (copy_from_user(kbuff, ubuff, count) > 0) {
        pr_err("%s: unknown error when copying user buffer\n", __func__);
        return -EINVAL; /* @TODO: find better error code */
    }

    if (dev_i < 0) {
        pr_err("%s: unsupported device node "PATH"%s\n", __func__, file_name);
        return -EINVAL;
    }

    switch ((enum gem5_bridge_op)dev_i) {
    case GEM5_EXIT:
        kbufp = gem5_bridge_parse_ints(1, kbufp, intargs);
        pr_info("%s: op=exit, args=%llu\n", __func__, intargs[0]);
        gem5_bridge_nextop = config_list[dev_i].opcode;
        gem5_bridge_poke_int(intargs[0]);
        break;
    default:
        pr_err("%s: write unsupported for "PATH"%s",
                __func__, config_list[dev_i].name);
        return -EINVAL;
    }

    pr_info("%s: SUCCESS!\n", __func__);
    return count;
}

static struct file_operations gem5_bridge_fops = {
    .owner      = THIS_MODULE,
    .open       = gem5_bridge_open,
    .release    = gem5_bridge_release,
    .mmap       = gem5_bridge_mmap,
    .write      = gem5_bridge_write,
};


/* ========================================================================= *
 * Module Initialization + Destruction
 * ========================================================================= */

static int gem5_bridge_uevent(struct device *dev, struct kobj_uevent_env *env)
{
    add_uevent_var(env, "DEVMODE=%#o", DEV_MODE);
    return 0;
}

static int __init gem5_bridge_init(void)
{
    int error;
    int cdev_i;
    dev_t cdev_number;
    struct device *devp;

    /* Make virtual mapping to physical MMIO address */
    gem5_bridge_mmio = ioremap(GEM5OPS_BASE, GEM5OPS_SIZE);
    if (gem5_bridge_mmio < 0) {
        pr_err("%s: failed to map gem5ops MMIO range\n", __func__);
        goto error_ioremap;
    }

    /* Register misc device */
    error = alloc_chrdev_region(&dev_number, 0, DEV_COUNT, DEV_NAME);
    if (error < 0) {
        pr_err("%s: failed to allocate device major number\n", __func__);
        goto error_alloc_chrdev_region;
    }

    /* Create device class */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0))
    dev_class = class_create(DEV_NAME);
#else
    dev_class = class_create(THIS_MODULE, DEV_NAME);
#endif
    if (IS_ERR(dev_class)) {
        pr_err("%s: failed to create device class\n", __func__);
        goto error_class_create;
    }

    /* Map udev event that sets access permissions on our devices */
    dev_class->dev_uevent = gem5_bridge_uevent;

    /* Create all character devices */
    for (cdev_i = 0; cdev_i < DEV_COUNT; ++cdev_i) {
        /* Create cdev struct for this device */
        cdev_number = MKDEV(MAJOR(dev_number), cdev_i);
        cdev_init(&cdev_list[cdev_i], &gem5_bridge_fops);
        error = cdev_add(&cdev_list[cdev_i], cdev_number, 1);
        if (error < 0) {
            pr_err("%s: failed to init/add character device: %s\n",
                    __func__, config_list[cdev_i].name);
            goto error_cdev_add;
        }

        /* Create device */
        devp = device_create(dev_class, NULL, cdev_number, NULL,
                             "gem5/%s", config_list[cdev_i].name);
        if (IS_ERR(devp)) {
            pr_err("%s: failed to create device: %s\n",
                    __func__, config_list[cdev_i].name);
            goto error_device_create;
        }
    }

    pr_info("%s: SUCCESS!\n", __func__);
    return 0;

error_device_create:
    /* Delete the last device's cdev */
    cdev_del(&cdev_list[cdev_i]);
error_cdev_add:
    /* Go back and delete all previously created devices */
    for (--cdev_i; cdev_i >= 0; --cdev_i) {
        cdev_number = MKDEV(MAJOR(dev_number), cdev_i);
        device_destroy(dev_class, cdev_number);
        cdev_del(&cdev_list[cdev_i]);
    }
    class_destroy(dev_class);
error_class_create:
    unregister_chrdev_region(dev_number, DEV_COUNT);
error_alloc_chrdev_region:
    iounmap(gem5_bridge_mmio);
error_ioremap:
    return -1;
}

static void __exit gem5_bridge_exit(void)
{
    int cdev_i;
    dev_t cdev_number;

    /* Undo all allocations and registrations in reverse order */
    for (cdev_i = 0; cdev_i < DEV_COUNT; ++cdev_i) {
        cdev_number = MKDEV(MAJOR(dev_number), cdev_i);
        device_destroy(dev_class, cdev_number);
        cdev_del(&cdev_list[cdev_i]);
    }
    class_destroy(dev_class);
    unregister_chrdev_region(dev_number, DEV_COUNT);
    iounmap(gem5_bridge_mmio);
    pr_info("%s: SUCCESS!\n", __func__);
}

module_init(gem5_bridge_init);
module_exit(gem5_bridge_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Noah Krim");
MODULE_DESCRIPTION("Diver that provides user-space mapping to MMIO range");
