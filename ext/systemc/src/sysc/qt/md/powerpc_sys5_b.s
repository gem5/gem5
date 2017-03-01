/* speed test for basic CPU operations  */


/* Marco Bucci <marco.bucci@inwind.it> */

/* This code was developed with the Code Warrior integrate ppc assembler.
 * Macros are use to hide illegal constructs whether you are using a 
 * "normal" assembler or the "C integrated" assembler.
 */

#if 0


	.text
	.align 4
	
	.globl b_call_reg
	.globl _b_call_reg
	.globl b_call_imm
	.globl _b_call_imm
	.globl b_add
	.globl _b_add
	.globl b_load
	.globl _b_load

.set fsize, 64
.set lrsave, 4

#else

#define fsize 64
#define lrsave 4

#endif




#if 0
.if 0
#endif
asm void b_null(void)
{
#if 0
.endif
#endif

#if 0
b_null:
#endif

	blr

#if 0
.if 0
#endif
}
#if 0
.endif
#endif


/* actually the same as the following. How to get "b_null" address?
 * I didnt find the right sintax or the right way.
 * I should take the current PC, then the difference to "b_null"
 * (making the difference beween the labels), perform the sum and go?! 
 */  
#if 0
.if 0
#endif
asm void b_call_reg(long n)
{
#if 0
.endif
#endif

#if 0
b_call_reg:
_b_call_reg:
#endif

	mflr	%r0
	stw		%r31,-4(%r1)
	stw		%r30,-8(%r1)
	stw		%r0,lrsave(%r1)
	stwu	%r1,-fsize(%r1)
	mr		%r30,%r3
	li		%r31,0
	
	b		L1
L0:
	bl		b_null
	bl		b_null
	bl		b_null
	bl		b_null
	bl		b_null

	addi       %r31,%r31,5
L1:
	cmpw       %r31,%r30
	blt        L0
	
	
	lwz        %r0,lrsave+fsize(%r1)
	mtlr       %r0
	lwz        %r31,-4+fsize(%r1)
	lwz        %r30,-8+fsize(%r1)
	addi       %r1,%r1,fsize
	blr

#if 0
.if 0
#endif
}
#if 0
.endif
#endif




#if 0
.if 0
#endif
asm void b_call_imm(long n)
{
#if 0
.endif
#endif

#if 0
b_call_imm:
_b_call_imm:
#endif

	mflr	%r0
	stw		%r31,-4(%r1)
	stw		%r30,-8(%r1)
	stw		%r0,lrsave(%r1)
	stwu	%r1,-fsize(%r1)
	mr		%r30,%r3
	li		%r31,0
	
	b		L3
L2:
	bl		b_null
	bl		b_null
	bl		b_null
	bl		b_null
	bl		b_null

	addi       %r31,%r31,5
L3:
	cmpw       %r31,%r30
	blt        L2
	
	
	lwz        %r0,lrsave+fsize(%r1)
	mtlr       %r0
	lwz        %r31,-4+fsize(%r1)
	lwz        %r30,-8+fsize(%r1)
	addi       %r1,%r1,fsize
	blr

#if 0
.if 0
#endif
}
#if 0
.endif
#endif



#if 0
.if 0
#endif
asm void b_add(long n)
{
#if 0
.endif
#endif

#if 0
b_add:
_b_add:
#endif

	mflr	%r0
	stw		%r31,-4(%r1)
	stw		%r30,-8(%r1)
	stw		%r0,lrsave(%r1)
	stwu	%r1,-fsize(%r1)
	mr		%r30,%r3
	li		%r31,0
	
	b		L5
L4:
	addi	%r3,%r3,5
	addi	%r4,%r4,5
	addi	%r5,%r5,5
	addi	%r6,%r6,5
	addi	%r7,%r7,5

	addi	%r3,%r3,5
	addi	%r4,%r4,5
	addi	%r5,%r5,5
	addi	%r6,%r6,5
	addi	%r7,%r7,5

	addi       %r31,%r31,10
L5:
	cmpw       %r31,%r30
	blt        L4
	
	
	lwz        %r0,lrsave+fsize(%r1)
	mtlr       %r0
	lwz        %r31,-4+fsize(%r1)
	lwz        %r30,-8+fsize(%r1)
	addi       %r1,%r1,fsize
	blr

#if 0
.if 0
#endif
}
#if 0
.endif
#endif



#if 0
.if 0
#endif
asm void b_load(long n)
{
#if 0
.endif
#endif

#if 0
b_load:
_b_load:
#endif

	mflr	%r0
	stw		%r31,-4(%r1)
	stw		%r30,-8(%r1)
	stw		%r0,lrsave(%r1)
	stwu	%r1,-fsize(%r1)
	mr		%r30,%r3
	li		%r31,0
	
	b		L7
L6:
	lwz		%r3,4(%r1)
	lwz		%r4,8(%r1)
	lwz		%r5,12(%r1)
	lwz		%r6,16(%r1)
	lwz		%r7,20(%r1)

	lwz		%r3,24(%r1)
	lwz		%r4,28(%r1)
	lwz		%r5,32(%r1)
	lwz		%r6,36(%r1)
	lwz		%r7,40(%r1)


	addi       %r31,%r31,10
L7:
	cmpw       %r31,%r30
	blt        L6
	
	
	lwz        %r0,lrsave+fsize(%r1)
	mtlr       %r0
	lwz        %r31,-4+fsize(%r1)
	lwz        %r30,-8+fsize(%r1)
	addi       %r1,%r1,fsize
	blr

#if 0
.if 0
#endif
}
#if 0
.endif
#endif
