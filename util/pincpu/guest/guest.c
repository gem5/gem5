/*
 * Copyright (c) 2026 The Board of Trustees of the Leland Stanford
 * Junior University
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

#include <asm/prctl.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/syscall.h>

#define STDERR_FILENO 2
#define ENOMEM 12
#define EEXIST 17

#include "arch/x86/pin/message.h"
#include "ops.h"
#include "printf.h"
#include "syscall.h"

#ifdef printf
#undef printf
#endif
#define printf(...)                                                           \
    do {                                                                      \
    } while (0)

#define err(fmt, ...)                                                         \
    do {                                                                      \
        printf_("error: " fmt " (%d)\n" __VA_OPT__(, ) __VA_ARGS__, errno);   \
    } while (0)

// FIXME: Virtual
#define vsyscall_base 0xffffffffff600000ULL
#define vsyscall_end (vsyscall_base + 0x1000)

#define min(a, b) (((a) < (b)) ? (a) : (b))

static void __attribute__((unused))
do_assert_failure(const char *file, int line, const char *desc)
{ printf_("%s:%d: assertion failed: %s\n", file, line, desc); }

#define assert(pred)                                                          \
    do {                                                                      \
        if (!(pred))                                                          \
            do_assert_failure(__FILE__, __LINE__, #pred);                     \
    } while (false)

static int req_fd;
static int resp_fd;
static int mem_fd;

typedef struct Message Message;

static void
diagnose_ENOMEM(void)
{
    // Read /proc/self/maps.
    int maps_fd;
    if ((maps_fd = open("/proc/self/maps", O_RDONLY)) < 0) {
        err("open: /proc/self/maps");
        return;
    }

    // Count the number of maps present.
    int num_maps = 0;
    while (true) {
        char c;
        ssize_t bytes_read = read(maps_fd, &c, 1);
        if (bytes_read < 0) {
            err("read: /proc/self/maps");
            return;
        }
        if (bytes_read == 0) {
            break;
        }
        printf_("%c", c);
        if (c == '\n') {
            ++num_maps;
        }
    }

    printf_("ENOMEM: num maps: %d\n", num_maps);
}

void
read_all(int fd, void *data_, size_t size)
{
    char *data = (char *)data_;
    while (size) {
        const ssize_t bytes_read = read(fd, data, size);
        if (bytes_read < 0) {
            err("read");
            pinop_abort();
        }
        data += bytes_read;
        size -= bytes_read;
    }
}

void
write_all(int fd, const void *data_, size_t size)
{
    const char *data = (const char *)data_;
    while (size) {
        const ssize_t bytes_written = write(fd, data, size);
        if (bytes_written < 0) {
            err("write");
            pinop_abort();
        }
        data += bytes_written;
        size -= bytes_written;
    }
}

void
_putchar(char c)
{ write(STDERR_FILENO, &c, 1); }

void
msg_read(Message *msg)
{
    printf("GUEST: reading request\n");
    read_all(req_fd, msg, sizeof *msg);
    printf("GUEST: read request\n");
}

void
msg_write(const Message *msg)
{ write_all(resp_fd, msg, sizeof *msg); }

void
main_event_loop(void)
{
    while (true) {
        Message msg;
        msg_read(&msg);

        switch (msg.type) {
            case Ack:
                msg_write(&msg);
                break;

            case Map: {
                // Check if vsyscall. This is special case.
                bool is_vsyscall = false;
                if (msg.map.vaddr == vsyscall_base) {
                    printf("GUEST: fixing up vsyscall mapping 0x%" PRIx64
                           "->0x%" PRIx64 "\n",
                           msg.map.vaddr, msg.map.paddr);
                    msg.map.vaddr = 0xcafebabe000;
                    is_vsyscall = true;
                }

                // printf_("mapping page: %p->%p 0x%lx\n", (void *)
                // msg.map.vaddr, (void *) msg.map.paddr, msg.map.size);
                // MAP_FIXED_NOREPLACE rather than MAP_FIXED: the guest's
                // address space shares this process with Pin, and MAP_FIXED
                // would silently unmap whatever it landed on. A conflict here
                // means the guest range overlaps Pin's own memory, which
                // otherwise corrupts Pin and surfaces much later as an
                // unrelated crash inside its code cache.
                void *map;
                if ((map = mmap((void *)msg.map.vaddr, msg.map.size,
                                msg.map.prot, MAP_SHARED | MAP_FIXED_NOREPLACE,
                                mem_fd, msg.map.paddr)) == MAP_FAILED) {
                    if (errno == EEXIST) {
                        printf_("error: guest mapping at %p (size %zu) "
                                "overlaps memory already in use by Pin\n",
                                (void *)msg.map.vaddr, msg.map.size);
                    }
                    err("mmap failed: vaddr=%p size=%zu paddr=%p",
                        (void *)msg.map.vaddr, msg.map.size,
                        (void *)msg.map.paddr);
                    if (errno == ENOMEM) {
                        diagnose_ENOMEM();
                    }
                    pinop_abort();
                }
                if (map != (void *)msg.map.vaddr) {
                    printf_("error: mmap mapped wrong address\n");
                    pinop_abort();
                }
                printf_("mapped page: %p->%p %p\n", (void *)msg.map.vaddr,
                        (void *)msg.map.paddr, (void *)msg.map.size);
                // printf_("first byte: %02hhx\n", * (uint8_t *) map);
                if (is_vsyscall) {
                    pinop_set_vsyscall_base((void *)vsyscall_base, map);
                }
                msg.type = Ack;
                msg_write(&msg);
            } break;

            case Unmap: {
                if (munmap((void *)msg.map.vaddr, msg.map.size) < 0) {
                    printf_("error: munmap failed (%d): vaddr=%p "
                            "size=0x%" PRIx64 "\n",
                            errno, (void *)msg.map.vaddr, msg.map.size);
                    pinop_abort();
                }
                printf_("unmapped page: %p\n", (void *)msg.map.vaddr);
                msg.type = Ack;
                msg_write(&msg);
            } break;

            case Run: {
                printf("GUEST handling RUN request\n");
                struct RunResult result;
                pinop_run(&result);
                Message msg;
                msg.inst_count = pinop_get_instcount();
                switch (result.result) {
                    case RUNRESULT_PAGEFAULT:
                        // Send this up to gem5.
                        printf("GUEST: got page fault: %" PRIx64 "\n",
                               result.addr);
                        msg.type = PageFault;
                        msg.faultaddr = result.addr;
                        break;

                    case RUNRESULT_SYSCALL:
                        // Send this up to gem5.
                        msg.type = Syscall;
                        break;

                    case RUNRESULT_CPUID:
                        // Send up to gem5.
                        msg.type = Cpuid;
                        break;

                    case RUNRESULT_BREAK:
                        // Send up to gem5.
                        msg.type = Break;
                        break;

                    default:
                        printf_("GUEST ERROR: unhandled run result: %d\n",
                                result.result);
                        pinop_abort();
                }

                msg_write(&msg);
            } break;

            case SetRegs:
                pinop_set_regs(&msg.regfile);
                msg.type = Ack;
                msg_write(&msg);
                break;

            case GetRegs:
                pinop_get_regs(&msg.regfile);
                msg.type = SetRegs;
                msg_write(&msg);
                break;

            case Exit:
                exit(0);

            case AddSymbol:
                pinop_add_symbol(msg.symbol.name, (void *)msg.symbol.vaddr);
                msg.type = Ack;
                msg_write(&msg);
                break;

            case ExecCommand: {
                const size_t bytes = pinop_exec_command(msg.command);
                msg.type = CommandResult;
                msg.command_result_size = bytes;
                msg_write(&msg);

                // TODO: Should just send null-terminated string, once we use
                // buffered files on the gem5 end.
                for (size_t i = 0; i != bytes;) {
                    char buf[1024];
                    const size_t chunk = min(bytes - i, sizeof buf);
                    pinop_read_command_result(buf, i, chunk);
                    write_all(resp_fd, buf, chunk);
                    i += chunk;
                }
            } break;

            default:
                printf_("error: bad message type (%d)\n", msg.type);
                pinop_abort();
        }

        // printf("GUEST: handled message, going on to next iteration\n");
    }
}

void main2(void);
void switch_stacks(void *new_stack, void (*f)(void));
void
main(void)
{
    // Switch stacks.
    printf_("Switching stacks...\n");
    const uint64_t stack_base = 0xcafe0000000;
    const uint64_t stack_size = 0x1000 * 32;
    void *stack;
    if ((stack = mmap((void *)stack_base, stack_size, PROT_READ | PROT_WRITE,
                      MAP_PRIVATE | MAP_ANON, -1, 0)) == MAP_FAILED) {
        printf_("error: failed to map stack (errno=%d)\n", errno);
        pinop_abort();
    }
    switch_stacks((uint8_t *)stack + stack_size, main2);
}

__attribute__((naked)) void
switch_stacks(void *new_stack, void (*f)(void))
{
    asm volatile("mov %rsp, -8(%rdi)\n" // Save old stack.
                 "mov %rdi, %rsp\n"     // Set to new stack.
                 "sub $8, %rsp\n"
                 "call *%rsi\n"
                 "mov (%rsp), %rsp\n"
                 "ret\n");
}

void
main2(void)
{
    char path[256];
    printf("GUEST: starting up\n");

    // Open request file.
    pinop_get_reqpath(path, sizeof path);
    if ((req_fd = open(path, O_RDONLY)) < 0) {
        printf("error: open failed: %s (%d)\n", path, errno);
        pinop_abort();
    }

    printf("GUEST: opened request file\n");

    // Open response file.
    pinop_get_resppath(path, sizeof path);
    if ((resp_fd = open(path, O_WRONLY)) < 0) {
        err("open: %s", path);
        pinop_abort();
    }

    // Open physmem file.
    char mem_path[256];
    pinop_get_mempath(mem_path, sizeof mem_path);
    if ((mem_fd = open(mem_path, O_RDWR)) < 0) {
        err("open: %s", mem_path);
        pinop_abort();
    }

    // Initialize user context.
    pinop_resetuser();

    main_event_loop();

    pinop_exit(0);
}
