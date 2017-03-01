/*
 * QuickThreads -- Threads-building toolkit.
 * Copyright (c) 1993 by David Keppel
 *
 * Permission to use, copy, modify and distribute this software and
 * its documentation for any purpose and without fee is hereby
 * granted, provided that the above copyright notice and this notice
 * appear in all copies.  This software is provided as a
 * proof-of-concept and for demonstration purposes; there is no
 * representation about the suitability of this software for any
 * purpose.
 */

	.file	"ksr1.s"
	.def	.debug;	.endef

	.align 128
	.globl qt_blocki
	.globl qt_blocki$TXT
	.globl qt_block
	.globl qt_block$TXT
	.globl qt_start$TXT
        .globl qt_start
	.globl qt_abort$TXT
	.globl qt_abort
	.globl qt_vstart
	.globl qt_vstart$TXT

#
# KSR convention: on procedure calls, load both the procedure address
# and a pointer to a constant block.  The address of function `f' is
# `f$TXT', and the constant block address is `f'.  The constant block
# has several reserved values:
#
#	8 bytes fpu register save mask
#	4 bytes ipu register save mask
#	4 bytes ceu register save mask
#   f:  f$TXT
#	... whatever you want ... (not quite...read on)
#
# Note, by the way, that a pointer to a function is passed as a
# pointer to the constant area, and the constant area has the text
# address.
#

#
# Procedures that do not return structures prefix their code with
#
# proc$TXT:
#   finop; cxnop
#   finop; cxnop
#   <proc code>
#
# Calls to those procedures branch to a 16 byte offset (4 instrs) in
# to the procedure to skip those instructions.
#
# Procedures that return structures use a different code prefix:
#
# proc$TXT:
#   finop; beq.qt %rc, %rc, 24		# return value entry
#   finop; cxnop
#   finop; movi8 0, %rc			# no return value entry
#   <proc code>
#
# Calls that want the returned structure branch directly to the
# procedure address.  Callers that don't want (or aren't expecting) a
# return value branche 16 bytes in to the procedure, which will zero
# %rc, telling the called procedure not to return a structure.
#

#
# On entry:
#   %i2 -- control block of helper function to run
#          (dereference to get helper)
#   %i3 -- a1
#   %i4 -- a2
#   %i5 -- sp of new to run
#

        .data
	.half 0x0, 0x0, 0x7ffff000, 0x7fff8000
qt_blocki:
qt_abort:
	.word qt_blocki$TXT
	.word qt_restore$TXT

	.text
qt_abort$TXT:
qt_blocki$TXT:
	finop			; cxnop			# entry prefix
	finop			; cxnop			# entry prefix
	add8.ntr 75,%i31,%i31	; movi8 512,%c5		# ICR; stk adjust
	finop			; ssub8.ntr 0,%sp,%c5,%sp
	finop			; st8 %fp,504(%sp)	# Save caller's fp
	finop			; st8 %cp,496(%sp)	# Save caller's cp
	finop			; ld8 8(%c10),%c5	# ld qt_restore$TXT
	finop			; st8 %c14,0(%sp)	# Save special ret addr
	finop			; mov8_8 %c10, %cp	# Our cp
	finop			; sadd8.ntr 0,%sp,%c5,%fp # Our frame ptr
	finop			; st8 %c5,8(%sp)	# st qt_restore$TXT
#
# CEU registers %c15-%c24, %c26-%c30 (%c14 we restore later)
#
	finop			; st8  %c15,456(%sp)
	finop			; st8  %c16,448(%sp)
	finop			; st8  %c17,440(%sp)
	finop			; st8  %c18,432(%sp)
	finop			; st8  %c19,424(%sp)
	finop			; st8  %c20,416(%sp)
	finop			; st8  %c21,408(%sp)
	finop			; st8  %c22,400(%sp)
	finop			; st8  %c23,392(%sp)
	finop			; st8  %c24,384(%sp)
#
# %c25 is the Enclosing Frame Pointer (EFP) -- since C doesn't
# use nested procedures, we ignore it (leaving a gap, though)
#
	finop			; st8 %c26,368(%sp)
	finop			; st8 %c27,360(%sp)
	finop			; st8 %c28,352(%sp)
	finop			; st8 %c29,344(%sp)
	finop			; st8 %c30,336(%sp)
#
# IPU registers %i12-%i30
# 
	finop			; st8 %i12,328(%sp)
	finop			; st8 %i13,320(%sp)
	finop			; st8 %i14,312(%sp)
	finop			; st8 %i15,304(%sp)
# (gap to get alignment for st64)
# -- Doesn't work on version 1.1.3 of the OS
#	finop			; st64 %i16,256(%sp)

	finop			; st8 %i16,256(%sp)
	finop			; st8 %i17,248(%sp)
	finop			; st8 %i18,240(%sp)
	finop			; st8 %i19,232(%sp)
	finop			; st8 %i20,224(%sp)
	finop			; st8 %i21,216(%sp)
	finop			; st8 %i22,208(%sp)
	finop			; st8 %i23,200(%sp)
	finop			; st8 %i24,192(%sp)
	finop			; st8 %i25,184(%sp)
	finop			; st8 %i26,176(%sp)
	finop			; st8 %i27,168(%sp)
	finop			; st8 %i28,160(%sp)
	finop			; st8 %i29,152(%sp)
	finop			; st8 %i30,144(%sp)
#
# FPU already saved, or saving not necessary
#

#
# Switch to the stack passed in as fourth argument to the block
# routine (%i5) and call the helper routine passed in as the first
# argument (%i2).  Note that the address of the helper's constant
# block is passed in, so we must derefence it to get the helper's text
# address.
#
	finop			; movb8_8 %i2,%c10	# helper's ConstBlock
	finop			; cxnop			# Delay slot, fill w/
	finop			; cxnop			# .. 2 st8 from above
	finop			; ld8 0(%c10),%c4	# load addr of helper
	finop			; movb8_8 %sp, %i2	# 1st arg to helper
							# is this stack; other
							# args remain in regs
	finop			; movb8_8 %i5,%sp	# switch stacks
	finop			; jsr %c14,16(%c4)	# call helper
	movi8 3, %i0		; movi8 0,%c8		# nargs brain dmg
	finop			; cxnop	
	finop			; cxnop	
#
# Here is where behavior differs for threads being restored and threads
# being started.  Blocked threads have a pointer to qt_restore$TXT on
# the top of their stacks; manufactured stacks have a pointer to qt_start$TXT
# on the top of their stacks.  With this setup, starting threads
# skip the (unecessary) restore operations.
#
# We jump to an offset of 16 to either (1) skip past the two noop pairs
# at the start of qt_start$TXT, or (2) skip past the two noop pairs
# after qt_restore$TXT.
#
	finop			; ld8 8(%sp),%c4
	finop			; cxnop
	finop			; cxnop
	finop			; jmp 16(%c4)
qt_restore$TXT:
	finop			; cxnop
	finop			; cxnop
#
# Point of Restore:
#
# The helper funtion will return here.  Any result it has placed in
# a return register (most likely %i0) will not get overwritten below
# and will consequently be the return value of the blocking routine.
#

#
# CEU registers %c15-%c24, %c26-%c30 (%c14 we restore later)
#
	finop			; ld8  456(%sp),%c15
	finop			; ld8  448(%sp),%c16
	finop			; ld8  440(%sp),%c17
	finop			; ld8  432(%sp),%c18
	finop			; ld8  424(%sp),%c19
	finop			; ld8  416(%sp),%c20
	finop			; ld8  408(%sp),%c21
	finop			; ld8  400(%sp),%c22
	finop			; ld8  392(%sp),%c23
	finop			; ld8  384(%sp),%c24
#
# %c25 is the Enclosing Frame Pointer (EFP) -- since C doesn't
# use nested procedures, we ignore it (leaving a gap, though)
#
	finop			; ld8 368(%sp),%c26
	finop			; ld8 360(%sp),%c27
	finop			; ld8 352(%sp),%c28
	finop			; ld8 344(%sp),%c29
	finop			; ld8 336(%sp),%c30
#
# IPU registers %i12-%i30
# 
	finop			; ld8 328(%sp),%i12
	finop			; ld8 320(%sp),%i13
	finop			; ld8 312(%sp),%i14
	finop			; ld8 304(%sp),%i15
# (gap to get alignment for ld64)
# -- Doesn't work on version 1.1.3 of the OS
#	finop			; ld64 256(%sp),%i16

	finop			; ld8 256(%sp),%i16
	finop			; ld8 248(%sp),%i17
	finop			; ld8 240(%sp),%i18
	finop			; ld8 232(%sp),%i19
	finop			; ld8 224(%sp),%i20
	finop			; ld8 216(%sp),%i21
	finop			; ld8 208(%sp),%i22
	finop			; ld8 200(%sp),%i23
	finop			; ld8 192(%sp),%i24
	finop			; ld8 184(%sp),%i25
	finop			; ld8 176(%sp),%i26
	finop			; ld8 168(%sp),%i27
	finop			; ld8 160(%sp),%i28
	finop			; ld8 152(%sp),%i29
	finop			; ld8 144(%sp),%i30

#
# FPU registers don't need to be loaded, or will be loaded by an
# enclosing scope (e.g., if this is called by qt_block).
#

#
# Load the special registers.  We don't load the stack ptr because
# the new stack is passed in as an argument, we don't load the EFP
# because we don't use it, and we load the return address specially
# off the top of the stack.
#
	finop			; ld8 0(%sp),%c14	# return addr
	finop			; ld8 496(%sp),%cp
	finop			; ld8 504(%sp),%fp

	finop			; jmp 32(%c14)		# jump back to thread
	finop			; movi8 512,%c5		# stack adjust
	finop			; sadd8.ntr 0,%sp,%c5,%sp

        .data
	.half 0x0, 0x0, 0x7ffff000, 0x7fff8000
qt_block:
	.word	qt_block$TXT
	.word	qt_error
	.word	qt_error$TXT
	.word	qt_blocki
#
# Handle saving and restoring the FPU regs, relying on qt_blocki
# to save and restore the remaining registers.
#
        .text
qt_block$TXT:
	finop			; cxnop			# entry prefix
	finop			; cxnop			# entry prefix

	add8.ntr 29,%i31,%i31	; movi8 512,%c5		# ICR; stk adjust
	finop			; ssub8.ntr 0,%sp,%c5,%sp
	finop			; st8 %fp,504(%sp)	# Save caller's fp
	finop			; st8 %cp,496(%sp)	# Save caller's cp
	finop			; st8 %c14,488(%sp)	# store ret addr
	finop			; sadd8.ntr 0,%sp,%c5,%fp # Our frame ptr
	finop			; mov8_8 %c10, %cp	# Our cp

#
# Store 8 registers at once...destination must be a multiple of 64
#
	finop			; st64 %f16,384(%sp)
	finop			; st64 %f24,320(%sp)
	finop			; st64 %f32,256(%sp)
	finop			; st64 %f40,192(%sp)
	finop			; st64 %f48,128(%sp)
	finop			; st64 %f56,64(%sp)

#
# Call the integer blocking routine, passing the arguments passed to us
#
	finop			; ld8 24(%cp), %c10
	finop			; cxnop
	finop			; jsr %c14, qt_blocki$TXT
	finop			; cxnop
	finop			; cxnop
	movi8 4,%i0		; movi8 0,%c8		# nargs brain dmg

#
# Load 8 registers at once...source must be a multiple of 64
#
	finop			; ld64 64(%sp),%f56
	finop			; ld64 128(%sp),%f48
	finop			; ld64 192(%sp),%f40
	finop			; ld64 256(%sp),%f32
	finop			; ld64 320(%sp),%f24
	finop			; ld64 384(%sp),%f16

	finop			; ld8 488(%sp),%c14
	finop			; ld8 496(%sp),%cp
	finop			; ld8 504(%sp),%fp
	finop			; jmp 32(%c14)		# jump back to thread
	finop			; movi8 512,%c5		# stack adjust
	finop			; sadd8.ntr 0,%sp,%c5,%sp


        .data
	.half 0x0, 0x0, 0x7ffff000, 0x7fff8000
qt_start:
	.word qt_start$TXT
#
# A new thread is set up to "appear" as if it were executing code at
# the beginning of qt_start and then it called a blocking routine
# (qt_blocki).  So when a new thread starts to run, it gets unblocked
# by the code above and "returns" to `qt_start$TXT' in the
# restore step of the switch.  Blocked threads jump to 16(qt_restore$TXT),
# and starting threads jump to 16(qt_start$TXT).
#
        .text
qt_start$TXT:
	finop			; cxnop			# 
	finop			; cxnop			# 
	finop			; ld8 40(%sp),%c10	# `only' constant block
	finop			; ld8 32(%sp),%i4	# `userf' arg.
	finop			; ld8 24(%sp),%i3	# `t' arg.
	finop			; ld8 0(%c10),%c4	# `only' text location
	finop			; ld8 16(%sp),%i2	# `u' arg.
	finop			; cxnop
	finop			; jsr %c14,16(%c4)	# call `only'
#
# Pop the frame used to store the thread's initial data
#
	finop			; sadd8.ntr 0,%sp,128,%sp
	finop			; cxnop
	movi8 2,%i0		; movi8 0,%c8		# nargs brain dmg
#
# If we ever return, it's an error.
#
	finop			; jmp qt_error$TXT
	finop			; cxnop
	finop			; cxnop
	movi8 0,%i0		; movi8 0,%c8		# nargs brain dmg


#
# This stuff is broken
#
        .data
	.half 0x0, 0x0, 0x7ffff000, 0x7fff8000
qt_vstart:
	.word qt_vstart$TXT

	.text
qt_vstart$TXT:
	finop			; cxnop			# entry prefix
	finop			; cxnop			# entry prefix
	finop			; cxnop
	finop			; cxnop
	add8.ntr 11,%i31,%i31	; movi8 512,%c5
	finop			; ssub8.ntr 0,%sp,%c5,%sp	# fix stack
	finop			; ld8 8(%sp),%i2	# load `t' as arg to
	finop			; cxnop			# `startup'
	finop			; cxnop
	finop			; ld8 16(%sp),%c10	# `startup' const block
	finop			; cxnop
	finop			; cxnop
	finop			; ld8 0(%c10),%c4	# `startup' text loc.
	finop			; cxnop
	finop			; cxnop
	finop			; jsr %c14,16(%c4)	# call `startup'
	finop			; cxnop
	finop			; cxnop
	movi8 1, %i0		; movi8 0,%c8		# nargs brain dmg
#
#	finop			; sadd 0,%sp,128,%sp	# alter stack
#
	finop			; ld8 8(%sp),%i2	# load `t' as arg to
	finop			; ld8 8(%sp),%i2	# load `t' as arg to
	finop			; ld8 8(%sp),%i2	# load `t' as arg to
	finop			; ld8 8(%sp),%i2	# load `t' as arg to

	finop			; ld8 32(%sp),%c10	# `only' constant block
	finop			; ld8 8(%sp),%i2	# `u' arg.
	finop			; ld8 16(%sp),%i3	# `t' arg.
	finop			; ld8 0(%c10),%c4	# `only' text location
	finop			; ld8 24(%sp),%i4	# `userf' arg.
	finop			; cxnop
	finop			; jsr %c4,16(%c4)	# call `only'
	finop			; cxnop
	finop			; cxnop
#
# If the callee ever calls `nargs', the following instruction (pair)
# will be executed.  However, we don't know when we compile this code
# how many args are being passed.  So we give our best guess: 0.
#
	movi8 0,%i0		; movi8 0,%c8		# nargs brain dmg
#
# If we ever return, it's an error.
#
	finop			; jmp qt_error$TXT
	finop			; cxnop
	finop			; cxnop
	movi8 0,%i0		; movi8 0,%c8		# nargs brain dmg
