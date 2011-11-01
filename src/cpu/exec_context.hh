/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 * Authors: Kevin Lim
 */

#error "Cannot include this file"

/**
 * The ExecContext is not a usable class.  It is simply here for
 * documentation purposes.  It shows the interface that is used by the
 * ISA to access and change CPU state.
 */
class ExecContext {
    // The register accessor methods provide the index of the
    // instruction's operand (e.g., 0 or 1), not the architectural
    // register index, to simplify the implementation of register
    // renaming.  We find the architectural register index by indexing
    // into the instruction's own operand index table.  Note that a
    // raw pointer to the StaticInst is provided instead of a
    // ref-counted StaticInstPtr to reduce overhead.  This is fine as
    // long as these methods don't copy the pointer into any long-term
    // storage (which is pretty hard to imagine they would have reason
    // to do).

    /** Reads an integer register. */
    uint64_t readIntRegOperand(const StaticInst *si, int idx);

    /** Reads a floating point register of single register width. */
    FloatReg readFloatRegOperand(const StaticInst *si, int idx);

    /** Reads a floating point register in its binary format, instead
     * of by value. */
    FloatRegBits readFloatRegOperandBits(const StaticInst *si, int idx);

    /** Sets an integer register to a value. */
    void setIntRegOperand(const StaticInst *si, int idx, uint64_t val);

    /** Sets a floating point register of single width to a value. */
    void setFloatRegOperand(const StaticInst *si, int idx, FloatReg val);

    /** Sets the bits of a floating point register of single width
     * to a binary value. */
    void setFloatRegOperandBits(const StaticInst *si, int idx,
                                FloatRegBits val);

    /** Reads the PC. */
    uint64_t readPC();
    /** Reads the NextPC. */
    uint64_t readNextPC();
    /** Reads the Next-NextPC. Only for architectures like SPARC or MIPS. */
    uint64_t readNextNPC();

    /** Sets the PC. */
    void setPC(uint64_t val);
    /** Sets the NextPC. */
    void setNextPC(uint64_t val);
    /** Sets the Next-NextPC.  Only for architectures like SPARC or MIPS. */
    void setNextNPC(uint64_t val);

    /** Reads a miscellaneous register. */
    MiscReg readMiscRegNoEffect(int misc_reg);

    /** Reads a miscellaneous register, handling any architectural
     * side effects due to reading that register. */
    MiscReg readMiscReg(int misc_reg);

    /** Sets a miscellaneous register. */
    void setMiscRegNoEffect(int misc_reg, const MiscReg &val);

    /** Sets a miscellaneous register, handling any architectural
     * side effects due to writing that register. */
    void setMiscReg(int misc_reg, const MiscReg &val);

    /** Records the effective address of the instruction.  Only valid
     * for memory ops. */
    void setEA(Addr EA);
    /** Returns the effective address of the instruction.  Only valid
     * for memory ops. */
    Addr getEA();

    /** Returns a pointer to the ThreadContext. */
    ThreadContext *tcBase();

    Fault readMem(Addr addr, uint8_t *data, unsigned size, unsigned flags);

    Fault writeMem(uint8_t *data, unsigned size,
                   Addr addr, unsigned flags, uint64_t *res);

    /** Somewhat Alpha-specific function that handles returning from
     * an error or interrupt. */
    Fault hwrei();

    /**
     * Check for special simulator handling of specific PAL calls.  If
     * return value is false, actual PAL call will be suppressed.
     */
    bool simPalCheck(int palFunc);

    /** Executes a syscall specified by the callnum. */
    void syscall(int64_t callnum);

    /** Finish a DTB address translation. */
    void finishTranslation(WholeTranslationState *state);
};
