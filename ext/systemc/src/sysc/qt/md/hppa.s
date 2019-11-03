; pa-risc.s -- assembly support.

; QuickThreads -- Threads-building toolkit.
; Copyright (c) 1993 by David Keppel
;
; Permission to use, copy, modify and distribute this software and
; its documentation for any purpose and without fee is hereby
; granted, provided that the above copyright notice and this notice
; appear in all copies.  This software is provided as a
; proof-of-concept and for demonstration purposes; there is no
; representation about the suitability of this software for any
; purpose.

; This file (pa-risc.s) is part of the port of QuickThreads for
; PA-RISC 1.1 architecture.  This file implements context switches
; and thread startup.  It was written in 1994 by Uwe Reder
; (`uereder@cip.informatik.uni-erlangen.de') for the Operating
; Systems Department (IMMD4) at the University of Erlangen/Nuernberg
; Germany.


; Callee saves general registers gr3..gr18,
;              floating-point registers fr12..fr21.

            .CODE

            .IMPORT $$dyncall, MILLICODE
            .IMPORT qt_error, CODE

            .EXPORT qt_blocki, ENTRY
            .EXPORT qt_block, ENTRY
            .EXPORT qt_abort, ENTRY
            .EXPORT qt_start, ENTRY
            .EXPORT qt_vstart, ENTRY


; arg0: ptr to function (helper) to call once curr is suspended
;       and control is on arg3's stack.
; arg1: 1'th arg to *arg0.
; arg2: 2'th arg to *arg0.
; arg3: sp of new thread.

qt_blocki
            .PROC
            .CALLINFO   CALLER, FRAME=0, SAVE_RP, ENTRY_GR=18
            .ENTRY

            stw         %rp,-20(%sp)    ; save rp to old frame-marker

            stwm        %r3,128(%sp)    ; save callee-saves general registers
            stw         %r4,-124(%sp)
            stw         %r5,-120(%sp)
            stw         %r6,-116(%sp)
            stw         %r7,-112(%sp)
            stw         %r8,-108(%sp)
            stw         %r9,-104(%sp)
            stw         %r10,-100(%sp)
            stw         %r11,-96(%sp)
            stw         %r12,-92(%sp)
            stw         %r13,-88(%sp)
            stw         %r14,-84(%sp)
            stw         %r15,-80(%sp)
            stw         %r16,-76(%sp)
            stw         %r17,-72(%sp)
            stw         %r18,-68(%sp)

qt_abort
            copy        %arg0,%r22      ; helper to be called by $$dyncall
            copy        %sp,%arg0       ; pass current sp as arg0 to helper
            copy        %arg3,%sp       ; set new sp

            .CALL
            bl          $$dyncall,%mrp  ; call helper
            copy        %mrp,%rp

            ldw         -68(%sp),%r18   ; restore general registers
            ldw         -72(%sp),%r17
            ldw         -76(%sp),%r16
            ldw         -80(%sp),%r15
            ldw         -84(%sp),%r14
            ldw         -88(%sp),%r13
            ldw         -92(%sp),%r12
            ldw         -96(%sp),%r11
            ldw         -100(%sp),%r10
            ldw         -104(%sp),%r9
            ldw         -108(%sp),%r8
            ldw         -112(%sp),%r7
            ldw         -116(%sp),%r6
            ldw         -120(%sp),%r5
            ldw         -124(%sp),%r4

            ldw         -148(%sp),%rp   ; restore return-pointer

            bv          %r0(%rp)        ; return to caller
            ldwm        -128(%sp),%r3

            .EXIT
            .PROCEND


qt_block
            .PROC
            .CALLINFO   CALLER, FRAME=0, SAVE_RP, ENTRY_FR=21
            .ENTRY

            stw         %rp,-20(%sp)    ; save rp to old frame-marker

            fstds,ma    %fr12,8(%sp)    ; save callee-saves float registers
            fstds,ma    %fr13,8(%sp)
            fstds,ma    %fr14,8(%sp)
            fstds,ma    %fr15,8(%sp)
            fstds,ma    %fr16,8(%sp)
            fstds,ma    %fr17,8(%sp)
            fstds,ma    %fr18,8(%sp)
            fstds,ma    %fr19,8(%sp)
            fstds,ma    %fr20,8(%sp)
            fstds,ma    %fr21,8(%sp)

            .CALL
            bl          qt_blocki,%rp
            ldo         48(%sp),%sp

            ldo         -48(%sp),%sp

            fldds,mb    -8(%sp),%fr21   ; restore callee-saves float registers
            fldds,mb    -8(%sp),%fr20
            fldds,mb    -8(%sp),%fr19
            fldds,mb    -8(%sp),%fr18
            fldds,mb    -8(%sp),%fr17
            fldds,mb    -8(%sp),%fr16
            fldds,mb    -8(%sp),%fr15
            fldds,mb    -8(%sp),%fr14
            fldds,mb    -8(%sp),%fr13

            ldw         -28(%sp),%rp    ; restore return-pointer
            
            bv          %r0(%rp)        ; return to caller.
            fldds,mb    -8(%sp),%fr12
            
            .EXIT
            .PROCEND


qt_start
            .PROC
            .CALLINFO   CALLER, FRAME=0
            .ENTRY

            copy        %r18,%arg0      ; set user arg `pu'.
            copy        %r17,%arg1      ; ... user function pt.
            copy        %r16,%arg2      ; ... user function userf.
                                        ; %r22 is a caller-saves register
            copy        %r15,%r22       ; function to be called by $$dyncall

            .CALL                       ; in=%r22
            bl          $$dyncall,%mrp  ; call `only'.
            copy        %mrp,%rp
            
            bl,n        qt_error,%r0    ; `only' erroniously returned.

            .EXIT
            .PROCEND


; Varargs
;
; First, call `startup' with the `pt' argument.
;
; Next, call the user's function with all arguments.
; We don't know whether arguments are integers, 32-bit floating-points or
; even 64-bit floating-points, so we reload all the registers, possibly
; with garbage arguments.  The thread creator provided non-garbage for
; the arguments that the callee actually uses, so the callee never gets
; garbage.
;
;            -48    -44    -40    -36    -32
;             | arg3 | arg2 | arg1 | arg0 |
;             -----------------------------
; integers:     arg3   arg2   arg1   arg0
; 32-bit fps:  farg3  farg2  farg1  farg0
; 64-bit fps:  <---farg3-->  <---farg1-->
;
; Finally, call `cleanup' with the `pt' argument and with the return value
; from the user's function.  It is an error for `cleanup' to return.

qt_vstart
            .PROC
            .CALLINFO   CALLER, FRAME=0
            .ENTRY

            ; Because the startup function may damage the fixed arguments
            ; on the stack (PA-RISC Procedure Calling Conventions Reference
            ; Manual, 2.4 Fixed Arguments Area), we allocate a seperate
            ; stack frame for it.
            ldo         64(%sp),%sp

            ; call: void startup(void *pt)

            copy        %r15,%arg0      ; `pt' is arg0 to `startup'.
            copy        %r16,%r22
            .CALL
            bl          $$dyncall,%mrp  ; Call `startup'.
            copy        %mrp,%rp

            ldo         -64(%sp),%sp

            ; call: void *qt_vuserf_t(...)

            ldw         -36(%sp),%arg0  ; Load args to integer registers.
            ldw         -40(%sp),%arg1
            ldw         -44(%sp),%arg2
            ldw         -48(%sp),%arg3
            ; Index of fld[w|d]s only ranges from -16 to 15, so we
            ; take r22 to be our new base register.
            ldo         -32(%sp),%r22
            fldws       -4(%r22),%farg0 ; Load args to floating-point registers.
            fldds       -8(%r22),%farg1
            fldws       -12(%r22),%farg2
            fldds       -16(%r22),%farg3
            copy        %r17,%r22
            .CALL
            bl          $$dyncall,%mrp  ; Call `userf'.
            copy        %mrp,%rp

            ; call: void cleanup(void *pt, void *vuserf_return)

            copy        %r15,%arg0      ; `pt' is arg0 to `cleanup'.
            copy        %ret0,%arg1     ; Return-value is arg1 to `cleanup'.
            copy        %r18,%r22
            .CALL
            bl          $$dyncall,%mrp  ; Call `cleanup'.
            copy        %mrp,%rp

            bl,n        qt_error,%r0

            .EXIT
            .PROCEND
