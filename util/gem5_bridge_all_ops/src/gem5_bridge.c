#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kdev_t.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include "gem5_bridge.h"
#include "gem5_ops.h"

/* ========================================================================= *
 * Constants
 * ========================================================================= */

/* Device information */
#define DEV_NAME "gem5_bridge"
#define DEV_GROUP "gem5"
#define DEV_MODE ((umode_t)0666) /* All users have RW access to this device */
#define DEV_COUNT op_count /* Comes from size of op_list */

/* Just helpful for printing */
#define PATH "/dev/gem5/"


/* ========================================================================= *
 * Exported Variables
 * ========================================================================= */

/* gem5 bridge */
void __iomem *gem5_bridge_mmio;


/* ========================================================================= *
 * Local Variables
 * ========================================================================= */

/* Device information */
static dev_t dev_number;
static struct class *dev_class;
static struct cdev *cdev_list;


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
    struct gem5_op *op = gem5_ops_search(file_name);

    if (vm_size > GEM5OPS_SIZE) {
        pr_err("%s: requested memory range is too large\n", __func__);
        return -EINVAL;
    }

    /* Hard-coded check for bridge device, no other device can mmap */
    if (!op || op != &op_list[0]) {
        pr_err("%s: mmap unsupported for "PATH"%s\n", __func__, op->name);
        return -ENOTTY;
    }

    /* Map the virtual memory area to our gem5 ops MMIO range */
    if (io_remap_pfn_range(vma, vma->vm_start, gem5_bridge_pfn,
                            vm_size, vma->vm_page_prot))
    {
        pr_err("%s: failed to remap vm area to phys addr\n", __func__);
        return -EINVAL;
    }

    pr_info("%s: SUCCESS!\n", __func__);
    return 0;
}

static ssize_t gem5_bridge_read(struct file *filp, char *ubuff,
                                    size_t len, loff_t *offp)
{
    ssize_t result;
    char *kbuff;
    const char *file_name = filp->f_path.dentry->d_iname;
    struct gem5_op *op = gem5_ops_search(file_name);

    /* Allocate kernel buffer */
    kbuff = kvmalloc(len, GFP_KERNEL);
    if (!kbuff) {
        pr_err("%s: unable to allocate kernel buffer\n", __func__);
        return -ENOMEM;
    }

    /* Check that the device is valid and its op has read functionality */
    if (!op || !op->read) {
        pr_err("%s: read unsupported for "PATH"%s\n", __func__, op->name);
        kvfree(kbuff);
        return -ENOTTY;
    }

    /* Run this op's read function */
    result = op->read(op, kbuff, len, offp);
    if (result < 0) {
        pr_err("%s: read failed for "PATH"%s\n", __func__, op->name);
        kvfree(kbuff);
        return result; /* propogate the error */
    }

    /* Copy kernel buffer to user buffer */
    if (copy_to_user(ubuff, kbuff, result) != 0) {
        pr_err("%s: unable to copy kernel buffer to user buffer\n", __func__);
        kvfree(kbuff);
        return -EFAULT;
    }

    pr_info("%s: SUCCESS! (result=%lu)\n", __func__, result);
    kvfree(kbuff);
    return result;
}

static ssize_t gem5_bridge_write(struct file *filp, const char *ubuff,
                                    size_t len, loff_t *offp)
{
    ssize_t result;
    char *kbuff;
    const char *file_name = filp->f_path.dentry->d_iname;
    struct gem5_op *op = gem5_ops_search(file_name);

    /* Allocate kernel buffer */
    kbuff = kvmalloc(len, GFP_KERNEL);
    if (!kbuff) {
        pr_err("%s: unable to allocate kernel buffer\n", __func__);
        return -ENOMEM;
    }

    /* Copy user buffer to kernel buffer */
    if (copy_from_user(kbuff, ubuff, len) != 0) {
        pr_err("%s: unable to copy user buffer to kernel buffer\n", __func__);
        kvfree(kbuff);
        return -EFAULT;
    }

    /* Check that the device is valid and its op has write functionality */
    if (!op || !op->write) {
        pr_err("%s: write unsupported for "PATH"%s\n", __func__, op->name);
        kvfree(kbuff);
        return -ENOTTY;
    }

    /* Run this op's write function */
    result = op->write(op, kbuff, len, offp);
    if (result < 0) {
        pr_err("%s: write failed for "PATH"%s\n", __func__, op->name);
        kvfree(kbuff);
        return result; /* propogate the error */
    }

    pr_info("%s: SUCCESS! (result=%lu)\n", __func__, result);
    kvfree(kbuff);
    return result;
}

static struct file_operations gem5_bridge_fops = {
    .owner      = THIS_MODULE,
    .open       = gem5_bridge_open,
    .release    = gem5_bridge_release,
    .mmap       = gem5_bridge_mmap,
    .read       = gem5_bridge_read,
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
    cdev_list = kvcalloc(DEV_COUNT, sizeof(struct cdev), GFP_KERNEL);
    if (!cdev_list) {
        pr_err("%s: failed to allocate cdev list\n", __func__);
        goto error_cdev_list_kvcalloc;
    }
    for (cdev_i = 0; cdev_i < DEV_COUNT; ++cdev_i) {
        /* Create cdev struct for this device */
        cdev_number = MKDEV(MAJOR(dev_number), cdev_i);
        cdev_init(&cdev_list[cdev_i], &gem5_bridge_fops);
        error = cdev_add(&cdev_list[cdev_i], cdev_number, 1);
        if (error < 0) {
            pr_err("%s: failed to init/add character device: %s\n",
                    __func__, op_list[cdev_i].name);
            goto error_cdev_add;
        }

        /* Create device */
        devp = device_create(dev_class, NULL, cdev_number, NULL,
                             DEV_GROUP"/%s", op_list[cdev_i].name);
        if (IS_ERR(devp)) {
            pr_err("%s: failed to create device: %s\n",
                    __func__, op_list[cdev_i].name);
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
    kvfree(cdev_list);
error_cdev_list_kvcalloc:
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
    kvfree(cdev_list);
    class_destroy(dev_class);
    unregister_chrdev_region(dev_number, DEV_COUNT);
    iounmap(gem5_bridge_mmio);
    pr_info("%s: SUCCESS!\n", __func__);
}

module_init(gem5_bridge_init);
module_exit(gem5_bridge_exit);


/* ========================================================================= *
 * FOOTER
 * ========================================================================= */

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Noah Krim");
MODULE_DESCRIPTION(
    "Driver that facilitates gem5 op calls via MMIO accesses"
);
