/*
 * Copyright (c) 2004 The Regents of The University of Michigan
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
 *
 * Authors: Ali Saidi
 */

#include <asm/io.h>
#include <asm/page.h>
#include <asm/uaccess.h>
#include <linux/config.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/netdevice.h>

#ifdef __i386__
#include <asm/msr.h>
#include <asm/processor.h>
#endif

#define DRIVER_AUTHOR "Ali Saidi"
#define DRIVER_DESC   "Interface to time uncacachable read and writes to device registers"
#define DRIVER_VER    "0.1"

static char *dataAddr = NULL;
static int count = 0;
#ifdef __alpha__
static int memTest = 0;
#endif

static inline uint32_t cycleCounter(uint32_t dep);

static int __init devtime_start(void)
{
    uint64_t addr;
    uint32_t t1, t2;
    uint32_t trash;
    int x;
    uint32_t *times;
    uint32_t num = 0;
    struct net_device *dev;

    printk("Devtime Driver Version %s Loaded...\n", DRIVER_VER);

#ifdef __alpha__
    if (memTest) {
           addr = 0xfffffc0000000000;
//         addr += 16*1024*1024;

            printk("Preparing memory test.\n");

            t1 = cycleCounter(trash);
            for (x = 0; x < count; x++) {
                trash = readl(addr);
                t2 = cycleCounter(trash);
                times[num++] = t2 - t1;
                t1 = t2;
               addr += 4096;
            }

            printk("Measurements:\n");
            for (x = 0; x < count; x++) {
                printk("%d ", times[x]);
                if (((x + 1) % 10) == 0)
                    printk("\n");
            }
            printk("\nDone.\n");
    } else
#endif
    if (dataAddr != 0 && count != 0) {
        addr = simple_strtoull(dataAddr, NULL, 0);

        addr = ioremap(addr, PAGE_SIZE);
        /**
         * Make sure that the remapping actually worked. On alpha we have
         * linear addressing, so its not a problem. But it can fail in x86
         * if physical memory is mapped to this address.
         */
        times = kmalloc(sizeof(uint32_t) * count, GFP_USER);
        if (!times) {
            printk("Could not allocate memory... Try again later.\n");
            return -1;
        }

        if (addr) {
            printk("Preparing to read %#llx %d times.\n", addr, count);

            t1 = cycleCounter(trash);
            for (x = 0; x < count; x++) {
                trash = readl(addr);
                t2 = cycleCounter(trash);
                times[num++] = t2 - t1;
                t1 = t2;
            }

            /**
             * Unmap the address.
             */
            iounmap(addr);

            printk("Measurements:\n");
            for (x = 0; x < count; x++) {
                printk("%d ", times[x]);
                if (((x + 1) % 10) == 0)
                    printk("\n");
            }
            printk("\nDone.\n");
        } else {
            printk("Unable to remap address. Please try again later.\n");
        }
    } else {
        dev = dev_get_by_name("eth0");
        if (dev) {
            printk("Eth0: MemStart: %#lx MemEnd: %#lx I/O Addr: %#lx\n",
                   dev->mem_start, dev->mem_end, dev->base_addr);
            dev_put(dev);
        }
        dev = 0;
        dev = dev_get_by_name("eth1");
        if (dev) {
            printk("Eth1: MemStart: %#lx MemEnd: %#lx I/O Addr: %#lx\n",
                   dev->mem_start, dev->mem_end, dev->base_addr);
            dev_put(dev);
        }

        printk("Required information not supplied.\n");
    }

    return 0;
}

#ifdef __i386__

static inline uint32_t cycleCounter(uint32_t dep)
{
    uint32_t time;
    cpuid_eax(0);
    rdtscl(time);
    cpuid_eax(0);
    return time;
}

#elif __alpha__

inline uint32_t cycleCounter(uint32_t dep)
{
    uint32_t res;
    asm volatile ("rpcc %0, %1" : "=r"(res) : "r" (dep) : "memory");
    return res;
}
#else
#error Architecture NOT SUPPORTED
#endif

static void __exit devtime_end(void)
{
    printk("Devtime Driver Version %s Unloaded...\n", DRIVER_VER);
}


module_init(devtime_start);
module_exit(devtime_end);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
module_param(dataAddr, charp, 0);
module_param(count, int, 0);
#ifdef __alpha__
module_param(memTest, int, 0);
#endif
