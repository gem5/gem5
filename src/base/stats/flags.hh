/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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
 * Authors: Nathan Binkert
 */

#ifndef __BASE_STATS_FLAGS_HH__
#define __BASE_STATS_FLAGS_HH__
namespace Stats {

/**
 * Define the storage for format flags.
 * @todo Can probably shrink this.
 */
typedef u_int32_t StatFlags;

/** Nothing extra to print. */
const StatFlags none =		0x00000000;
/** This Stat is Initialized */
const StatFlags init =		0x00000001;
/** Print this stat. */
const StatFlags print =		0x00000002;
/** Print the total. */
const StatFlags total =		0x00000010;
/** Print the percent of the total that this entry represents. */
const StatFlags pdf =		0x00000020;
/** Print the cumulative percentage of total upto this entry. */
const StatFlags cdf =		0x00000040;
/** Print the distribution. */
const StatFlags dist = 		0x00000080;
/** Don't print if this is zero. */
const StatFlags nozero =	0x00000100;
/** Don't print if this is NAN */
const StatFlags nonan =		0x00000200;
/** Used for SS compatability. */
const StatFlags __substat = 	0x80000000;

/** Mask of flags that can't be set directly */
const StatFlags __reserved =	init | print | __substat;

enum DisplayMode
{
    mode_m5,
    mode_simplescalar
};

extern DisplayMode DefaultMode;

/* namespace Stats */ }

#endif //  __BASE_STATS_FLAGS_HH__
