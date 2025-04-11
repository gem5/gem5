// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-Only
/*
 * Copyright (c) 2024 The Regents of the University of California
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/uaccess.h>

/* ========================================================================= *
 * Constants
 * ========================================================================= */

#define DEV_NAME "gem5_bridge"
#define DEV_MODE ((umode_t)0666) /* All users have RW access to this device */


/* ========================================================================= *
 * Device Parameters
 * ========================================================================= */

unsigned long gem5_bridge_baseaddr;
module_param(gem5_bridge_baseaddr, long, 0644);
MODULE_PARM_DESC(gem5_bridge_baseaddr,
    "Physical base addr for gem5ops MMIO range");

unsigned long gem5_bridge_rangesize;
module_param(gem5_bridge_rangesize, long, 0644);
MODULE_PARM_DESC(gem5_bridge_rangesize,
    "Size of gem5ops MMIO range");


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
    unsigned long gem5_bridge_pfn = gem5_bridge_baseaddr >> PAGE_SHIFT;

    if (vm_size > gem5_bridge_rangesize) {
        pr_err("%s: requested memory range is too large\n", __func__);
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

static struct file_operations gem5_bridge_fops = {
    .owner      = THIS_MODULE,
    .open       = gem5_bridge_open,
    .release    = gem5_bridge_release,
    .mmap       = gem5_bridge_mmap,
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
    /* Ensure params were defined */
    if (!gem5_bridge_baseaddr || !gem5_bridge_rangesize) {
        pr_err("%s: some parameters undefined or invalid, must give non-zero "
               "values for gem5_bridge_baseaddr and gem5_bridge_rangesize\n",
               __func__);
        return -1;
    }

    /* Register misc device */
    if (misc_register(&gem5_bridge_miscdevice) < 0) {
        pr_err("gem5_bridge_init: failed to register misc device\n");
        return -1;
    }

    pr_info("gem5_bridge_init: SUCCESS!\n");
    return 0;
}

static void __exit gem5_bridge_exit(void)
{
    /* Undo registration */
    misc_deregister(&gem5_bridge_miscdevice);
    pr_info("gem5_bridge_exit: SUCCESS!\n");
}

module_init(gem5_bridge_init);
module_exit(gem5_bridge_exit);


/* ========================================================================= *
 * FOOTER
 * ========================================================================= */

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Noah Krim");
MODULE_DESCRIPTION(
    "Driver that provides user-accessible mapping to gem5ops MMIO range"
);
