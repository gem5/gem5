/*
 * Copyright (c) 2013-2014 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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
 * Authors: Andrew Bardsley
 */

/**
 * @file
 *
 *  This file contains miscellaneous classes and functions for formatting
 *  general trace information and also MinorTrace information.
 *
 *  MinorTrace is this model's cycle-by-cycle trace information for use by
 *  minorview.
 */

#ifndef __CPU_MINOR_TRACE_HH__
#define __CPU_MINOR_TRACE_HH__

#include <string>

#include "base/trace.hh"
#include "debug/MinorTrace.hh"

namespace Minor
{

/** DPRINTFN for MinorTrace reporting */
#define MINORTRACE(...) \
    DPRINTF(MinorTrace, "MinorTrace: " __VA_ARGS__)

/** DPRINTFN for MinorTrace MinorInst line reporting */
#define MINORINST(sim_object, ...) \
    DPRINTFS(MinorTrace, (sim_object), "MinorInst: " __VA_ARGS__)

/** DPRINTFN for MinorTrace MinorLine line reporting */
#define MINORLINE(sim_object, ...) \
    DPRINTFS(MinorTrace, (sim_object), "MinorLine: " __VA_ARGS__)

}

#endif /* __CPU_MINOR_TRACE_HH__ */
