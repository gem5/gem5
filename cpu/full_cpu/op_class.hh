/*
 * Copyright (c) 2003-2004 The Regents of The University of Michigan
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

#ifndef __OP_CLASS_HH__
#define __OP_CLASS_HH__

/**
 * @file
 * Definition of operation classes.
 */

/**
 * Instruction operation classes.  These classes are used for
 * assigning instructions to functional units.
 */
enum OpClass {
    No_OpClass = 0,	/* inst does not use a functional unit */
    IntAluOp,		/* integer ALU */
    IntMultOp,		/* integer multiplier */
    IntDivOp,		/* integer divider */
    FloatAddOp,		/* floating point adder/subtractor */
    FloatCmpOp,		/* floating point comparator */
    FloatCvtOp,		/* floating point<->integer converter */
    FloatMultOp,	/* floating point multiplier */
    FloatDivOp,		/* floating point divider */
    FloatSqrtOp,	/* floating point square root */
    MemReadOp,		/* memory read port */
    MemWriteOp,		/* memory write port */
    IprAccessOp,	/* Internal Processor Register read/write port */
    InstPrefetchOp,	/* instruction prefetch port (on I-cache) */
    Num_OpClasses	/* total functional unit classes */
};

/**
 * Array mapping OpClass enum values to strings.  Defined in fu_pool.cc.
 */
extern const char *opClassStrings[];

#endif // __OP_CLASS_HH__
