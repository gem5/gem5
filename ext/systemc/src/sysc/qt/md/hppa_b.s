; QuickThreads -- Threads-building toolkit.
; Copyright (c) 1993 by David Keppel

; Permission to use, copy, modify and distribute this software and
; its documentation for any purpose and without fee is hereby
; granted, provided that the above copyright notice and this notice
; appear in all copies.  This software is provided as a
; proof-of-concept and for demonstration purposes; there is no
; representation about the suitability of this software for any
; purpose.

; This file (pa-risc_b.s) is part of the port of QuickThreads for
; PA-RISC 1.1 architecture.  It contains assembly-level support for
; raw processor performance measurement.  It was written in 1994 by
; Uwe Reder (`uereder@cip.informatik.uni-erlangen.de')
; for the Operating Systems Department (IMMD4) at the
; University of Erlangen/Nuernberg Germany.


; Note that the number of instructions in the measurement-loops, differ
; from implementation to implementation. I took eight instructions in a loop
; for every test (execute eight instructions and loop to the start).

            .CODE

            .IMPORT $global$,DATA
            .IMPORT $$dyncall,MILLICODE
            .EXPORT b_call_reg
            .EXPORT b_call_imm
            .EXPORT b_add
            .EXPORT b_load

; Just do nothing, only return to caller. This procedure is called by
; `b_call_reg' and `b_call_imm'.

b_null
            .PROC
            .CALLINFO   NO_CALLS, FRAME=0
            .ENTRY

            bv,n        %r0(%rp)        ; just return

            .EXIT
            .PROCEND

; Call the procedure `b_null' with function pointer in a register.

b_call_reg
            .PROC
            .CALLINFO   CALLER, FRAME=0
            .ENTRY
    
            stwm        %r3,64(%sp)     ; store r3 (may be used by caller)
            stw         %rp,-20(%sp)    ; save return-pointer to frame-marker

            addil       LR'to_call-$global$,%r27
            ldw         RR'to_call-$global$(%r1),%r3

_loop0
            copy        %r3,%r22        ; copy the procedure label to r22, ...
            .CALL                       ; ...this is the input to $$dyncall
            bl          $$dyncall,%mrp  ; call $$dyncall (millicode function)
            copy        %mrp,%rp        ; remember the return-pointer

            copy        %r3,%r22
            .CALL
            bl          $$dyncall,%mrp
            copy        %mrp,%rp

            copy        %r3,%r22
            .CALL
            bl          $$dyncall,%mrp
            copy        %mrp,%rp

            copy        %r3,%r22
            .CALL
            bl          $$dyncall,%mrp
            copy        %mrp,%rp

            copy        %r3,%r22
            .CALL
            bl          $$dyncall,%mrp
            copy        %mrp,%rp

            copy        %r3,%r22
            .CALL
            bl          $$dyncall,%mrp
            copy        %mrp,%rp

            copy        %r3,%r22
            .CALL
            bl          $$dyncall,%mrp
            copy        %mrp,%rp

            copy        %r3,%r22
            .CALL
            bl          $$dyncall,%mrp
            copy        %mrp,%rp

            addibf,<=   -8,%arg0,_loop0 ; decrement counter by 8 and loop
            nop

            ldw         -20(%sp),%rp    ; restore return-pointer
            bv          %r0(%rp)        ; return to caller
            ldwm        -64(%sp),%r3    ; resore r3 and remove stack frame

            .EXIT
            .PROCEND

; Call the procedure `b_null' immediate.

b_call_imm
            .PROC
            .CALLINFO   CALLER, FRAME=0, SAVE_RP
            .ENTRY

            ldo         64(%sp),%sp     ; caller needs a stack-frame
            stw         %rp,-20(%sp)    ; save return-pointer to frame-marker

_loop1
            bl          b_null,%rp      ; call `b_null' immediate (8 times)
            nop 
            bl          b_null,%rp
            nop
            bl          b_null,%rp
            nop
            bl          b_null,%rp
            nop
            bl          b_null,%rp
            nop
            bl          b_null,%rp
            nop
            bl          b_null,%rp
            nop
            bl          b_null,%rp
            nop

            addibf,<=   -8,%arg0,_loop1 ; decrement counter by 8 and loop
            nop
            
            ldw         -20(%sp),%rp    ; restore return-pointer
            bv          %r0(%rp)        ; return to caller
            ldo         -64(%sp),%sp    ; remove stack-frame

            .EXIT
            .PROCEND

; Copy register-to-register.
; On PA-RISC this is implemented with an `or'.
; The `or' is hidden by a pseudo-operation called `copy'.

b_add
            .PROC
            .CALLINFO   NO_CALLS, FRAME=0
            .ENTRY

_loop2
            copy        %r19,%r20       ; copy register-to-register
            copy        %r20,%r21       ; use caller-saves registers
            copy        %r21,%r22
            copy        %r22,%r21
            copy        %r21,%r20
            copy        %r20,%r19
            copy        %r19,%r20
            copy        %r20,%r21

            addibf,<=   -8,%arg0,_loop2 ; decrement counter by 8 and loop
            nop

            bv,n        %r0(%rp)

            .EXIT
            .PROCEND

; Load memory to a register.

b_load
            .PROC
            .CALLINFO   NO_CALLS, FRAME=0
            .ENTRY

_loop3
            ldw         -4(%sp),%r22    ; load data from frame-marker
            ldw         -8(%sp),%r22    ; use a caller-saves register
            ldw         -12(%sp),%r22
            ldw         -16(%sp),%r22
            ldw         -20(%sp),%r22
            ldw         -24(%sp),%r22
            ldw         -28(%sp),%r22
            ldw         -32(%sp),%r22

            addibf,<=   -8,%arg0,_loop3 ; decrement counter by 8 and loop
            nop

            bv,n        %r0(%rp)

            .EXIT
            .PROCEND


            .ALIGN 8
to_call
            .WORD       b_null
