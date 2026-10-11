/*
* Copyright (c) 2025  The Regents of the University of California
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

#ifndef HYPERCALL_IDS_H
#define HYPERCALL_IDS_H

/*
 * Central list of gem5's built-in exit-handler hypercalls. A hypercall is the
 * broader guest/simulator request mechanism; this list is the subset whose IDs
 * dispatch through the simulation-exit/stdlib ExitHandler path. The list is
 * expressed as a macro so multiple translation units (C and C++) can derive
 * enums, constants, and documentation without duplicating the mapping.
 *
 * CLASSIC_GENERATOR is intentionally not included in
 * GEM5_FOREACH_PUBLIC_EXIT_HYPERCALL. It is an internal compatibility path
 * that requires a legacy cause/code payload; guest calls to m5_hypercall() can
 * supply only an ID.
 *
 * OP arguments: (Name, Value, Description)
 */
#define GEM5_FOREACH_EXIT_HYPERCALL(OP)                                       \
    OP(CLASSIC_GENERATOR, 0,                                                  \
       "Legacy generator-based exit handling (ExitEvent translation)")        \
    GEM5_FOREACH_PUBLIC_EXIT_HYPERCALL(OP)

#define GEM5_FOREACH_PUBLIC_EXIT_HYPERCALL(OP)                                \
    OP(KERNEL_BOOTED, 1, "Guest kernel reported it has booted")               \
    OP(AFTER_BOOT, 2, "Guest entered the after_boot hook")                    \
    OP(AFTER_BOOT_SCRIPT, 3, "Guest completed after_boot.sh")                 \
    OP(WORK_BEGIN, 4, "Entered a region-of-interest (workbegin)")             \
    OP(WORK_END, 5, "Exited a region-of-interest (workend)")                  \
    OP(SCHEDULED_EXIT, 6, "Simulator scheduled tick/max-tick exit")           \
    OP(CHECKPOINT, 7, "Take a checkpoint and continue running")               \
    OP(ORCHESTRATOR, 1000, "Orchestrator control/status hypercall")

/*
 * Define C-compatible M5_HYPERCALL_* constants for IDs guest code may pass to
 * m5_hypercall().
 */
enum
{
#define GEM5_DECLARE_M5_HYPERCALL(name, value, desc)                          \
    M5_HYPERCALL_##name = value, /* desc */
    GEM5_FOREACH_PUBLIC_EXIT_HYPERCALL(GEM5_DECLARE_M5_HYPERCALL)
#undef GEM5_DECLARE_M5_HYPERCALL
};

/*
 * Backwards-compatible C-style names (preserve older header semantics). These
 * aliases intentionally point at the M5_HYPERCALL_* constants above so the
 * numeric ABI is centralized in GEM5_FOREACH_EXIT_HYPERCALL.
 */
enum
{
    KERNEL_BOOTED_EXIT = M5_HYPERCALL_KERNEL_BOOTED,
    STARTED_AFTERBOOT_SCRIPT_EXIT = M5_HYPERCALL_AFTER_BOOT,
    FINISHED_AFTERBOOT_SCRIPT_EXIT = M5_HYPERCALL_AFTER_BOOT_SCRIPT,
    WORK_BEGIN_EXIT = M5_HYPERCALL_WORK_BEGIN,
    WORK_END_EXIT = M5_HYPERCALL_WORK_END,
    SCHEDULED_EXIT = M5_HYPERCALL_SCHEDULED_EXIT,
    CHECKPOINT_EXIT = M5_HYPERCALL_CHECKPOINT,
    ORCHESTRATOR_EXIT = M5_HYPERCALL_ORCHESTRATOR,
};

#endif // HYPERCALL_IDS_H
