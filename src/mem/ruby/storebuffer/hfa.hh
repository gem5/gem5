/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
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

// this code was modified to fit into Rochs

#ifndef _HFA_H_
#define _HFA_H_

using namespace std;

/*
 * Global include file for entire project.
 * Should be included first in all ".cc" project files
 */

/*------------------------------------------------------------------------*/
/* Includes                                                               */
/*------------------------------------------------------------------------*/
#include "mem/ruby/common/Global.hh"
#include <string>
#include <map>
#include <set>
#include <list>
#include <fstream>
#include <iostream>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>             // va_start(), va_end()
#include <strings.h>            // declaration of bzero()

#include <sys/time.h>           // gettimeofday() includes
#include <errno.h>
#include <unistd.h>

/*------------------------------------------------------------------------*/
/* Type Includes                                                          */
/*------------------------------------------------------------------------*/

#include "mem/ruby/storebuffer/hfatypes.hh"

/*------------------------------------------------------------------------*/
/* Forward class declaration(s)                                           */
/*------------------------------------------------------------------------*/

class wait_list_t;
class waiter_t;
class free_list_t;
class pipestate_t;
class pipepool_t;


/** Maximum size of a load or store that may occur to/from the memory system.
 *  (in 64-bit quantities). Currently this is set to 8 * 64-bits = 64-bytes.
 */
const uint32 MEMOP_MAX_SIZE = 8;

/** 64-bit int memory masks */
#define MEM_BYTE_MASK 0x00000000000000ffULL
#define MEM_HALF_MASK 0x000000000000ffffULL
#define MEM_WORD_MASK 0x00000000ffffffffULL
#define MEM_EXTD_MASK 0xffffffffffffffffULL
#define MEM_QUAD_MASK 0xffffffffffffffffULL

#define ISEQ_MASK     0x0000ffffffffffffULL

/*------------------------------------------------------------------------*/
/* Configuration Parameters                                               */
/*------------------------------------------------------------------------*/

#define  SIM_HALT  assert(0);

#include <assert.h>

#endif  /* _HFA_H_ */


