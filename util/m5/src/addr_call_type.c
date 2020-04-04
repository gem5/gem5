/*
 * Copyright 2020 Google Inc.
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

#include <string.h>

#include "addr_call_type.h"
#include "args.h"
#include "m5_mmap.h"

#define M5OP(name, func) __typeof__(name) M5OP_MERGE_TOKENS(name, _addr);
M5OP_FOREACH
#undef M5OP

static DispatchTable addr_dispatch = {
#define M5OP(name, func) .name = &M5OP_MERGE_TOKENS(name, _addr),
M5OP_FOREACH
#undef M5OP
};

int
addr_call_type_detect(Args *args)
{
    static const char *prefix = "--addr";
    const size_t prefix_len = strlen(prefix);
    uint64_t addr_override;

    // If the first argument starts with --addr...
    if (args->argc && memcmp(args->argv[0], prefix, prefix_len) == 0) {
        const char *argv0 = pop_arg(args);

        // If there's more text in this argument...
        if (strlen(argv0) != prefix_len) {
            // If it doesn't start with '=', it's malformed.
            if (argv0[prefix_len] != '=')
                return -1;
            // Attempt to extract an address after the '='.
            const char *temp_argv[] = { &argv0[prefix_len + 1] };
            Args temp_args = { 1, temp_argv };
            if (!parse_int_args(&temp_args, &addr_override, 1))
                return -1;
            // If we found an address, use it to override m5op_addr.
            m5op_addr = addr_override;
            return 1;
        }
        // If an address override wasn't part of the first argument, check if
        // it's the second argument. If not, then there's no override.
        if (args->argc && parse_int_args(args, &addr_override, 1)) {
            m5op_addr = addr_override;
            return 1;
        }
        // If the default address was zero, an override is required.
        if (!m5op_addr)
            return -1;
        return 1;
    }
    return 0;
}

DispatchTable *
addr_call_type_init()
{
    map_m5_mem();
    return &addr_dispatch;
}
