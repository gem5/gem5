//===-- Generated From GDBRemoteSignals.cpp ------------------------===//
//
// Part of the LLVM Project,
// under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===---------------------------------------------------------------===//

#include <stdint.h>

#ifndef __BASE_GDB_SIGNALS_HH__
#define __BASE_GDB_SIGNALS_HH__

/*
These signals definitions are produced from LLVM's
  lldb/source/Plugins/Process/Utility/GDBRemoteSignals.cpp
*/
namespace gem5{
  enum class GDBSignal : uint8_t
  {
    ZERO = 0, //Signal 0
    HUP = 1, //hangup
    INT = 2, //interrupt
    QUIT = 3, //quit
    ILL = 4, //illegal instruction
    TRAP = 5, //trace trap (not reset when caught)
    ABRT = 6, //SIGIOT
    EMT = 7, //emulation trap
    FPE = 8, //floating point exception
    KILL = 9, //kill
    BUS = 10, //bus error
    SEGV = 11, //segmentation violation
    SYS = 12, //invalid system call
    PIPE = 13, //write to pipe with reading end closed
    ALRM = 14, //alarm
    TERM = 15, //termination requested
    URG = 16, //urgent data on socket
    STOP = 17, //process stop
    TSTP = 18, //tty stop
    CONT = 19, //process continue
    CHLD = 20, //SIGCLD
    TTIN = 21, //background tty read
    TTOU = 22, //background tty write
    IO = 23, //input/output ready/Pollable event
    XCPU = 24, //CPU resource exceeded
    XFSZ = 25, //file size limit exceeded
    VTALRM = 26, //virtual time alarm
    PROF = 27, //profiling time alarm
    WINCH = 28, //window size changes
    LOST = 29, //resource lost
    USR1 = 30, //user defined signal 1
    USR2 = 31, //user defined signal 2
    PWR = 32, //power failure
    POLL = 33, //pollable event
    WIND = 34, //SIGWIND
    PHONE = 35, //SIGPHONE
    WAITING = 36, //process's LWPs are blocked
    LWP = 37, //signal LWP
    DANGER = 38, //swap space dangerously low
    GRANT = 39, //monitor mode granted
    RETRACT = 40, //need to relinquish monitor mode
    MSG = 41, //monitor mode data available
    SOUND = 42, //sound completed
    SAK = 43, //secure attention
    PRIO = 44, //SIGPRIO

    SIG33 = 45, //real-time event 33
    SIG34 = 46, //real-time event 34
    SIG35 = 47, //real-time event 35
    SIG36 = 48, //real-time event 36
    SIG37 = 49, //real-time event 37
    SIG38 = 50, //real-time event 38
    SIG39 = 51, //real-time event 39
    SIG40 = 52, //real-time event 40
    SIG41 = 53, //real-time event 41
    SIG42 = 54, //real-time event 42
    SIG43 = 55, //real-time event 43
    SIG44 = 56, //real-time event 44
    SIG45 = 57, //real-time event 45
    SIG46 = 58, //real-time event 46
    SIG47 = 59, //real-time event 47
    SIG48 = 60, //real-time event 48
    SIG49 = 61, //real-time event 49
    SIG50 = 62, //real-time event 50
    SIG51 = 63, //real-time event 51
    SIG52 = 64, //real-time event 52
    SIG53 = 65, //real-time event 53
    SIG54 = 66, //real-time event 54
    SIG55 = 67, //real-time event 55
    SIG56 = 68, //real-time event 56
    SIG57 = 69, //real-time event 57
    SIG58 = 70, //real-time event 58
    SIG59 = 71, //real-time event 59
    SIG60 = 72, //real-time event 60
    SIG61 = 73, //real-time event 61
    SIG62 = 74, //real-time event 62
    SIG63 = 75, //real-time event 63

    CANCEL = 76, //LWP internal signal

    SIG32 = 77, //real-time event 32
    SIG64 = 78, //real-time event 64
    SIG65 = 79, //real-time event 65
    SIG66 = 80, //real-time event 66
    SIG67 = 81, //real-time event 67
    SIG68 = 82, //real-time event 68
    SIG69 = 83, //real-time event 69
    SIG70 = 84, //real-time event 70
    SIG71 = 85, //real-time event 71
    SIG72 = 86, //real-time event 72
    SIG73 = 87, //real-time event 73
    SIG74 = 88, //real-time event 74
    SIG75 = 89, //real-time event 75
    SIG76 = 90, //real-time event 76
    SIG77 = 91, //real-time event 77
    SIG78 = 92, //real-time event 78
    SIG79 = 93, //real-time event 79
    SIG80 = 94, //real-time event 80
    SIG81 = 95, //real-time event 81
    SIG82 = 96, //real-time event 82
    SIG83 = 97, //real-time event 83
    SIG84 = 98, //real-time event 84
    SIG85 = 99, //real-time event 85
    SIG86 = 100, //real-time event 86
    SIG87 = 101, //real-time event 87
    SIG88 = 102, //real-time event 88
    SIG89 = 103, //real-time event 89
    SIG90 = 104, //real-time event 90
    SIG91 = 105, //real-time event 91
    SIG92 = 106, //real-time event 92
    SIG93 = 107, //real-time event 93
    SIG94 = 108, //real-time event 94
    SIG95 = 109, //real-time event 95
    SIG96 = 110, //real-time event 96
    SIG97 = 111, //real-time event 97
    SIG98 = 112, //real-time event 98
    SIG99 = 113, //real-time event 99
    SIG100 = 114, //real-time event 100
    SIG101 = 115, //real-time event 101
    SIG102 = 116, //real-time event 102
    SIG103 = 117, //real-time event 103
    SIG104 = 118, //real-time event 104
    SIG105 = 119, //real-time event 105
    SIG106 = 120, //real-time event 106
    SIG107 = 121, //real-time event 107
    SIG108 = 122, //real-time event 108
    SIG109 = 123, //real-time event 109
    SIG110 = 124, //real-time event 110
    SIG111 = 125, //real-time event 111
    SIG112 = 126, //real-time event 112
    SIG113 = 127, //real-time event 113
    SIG114 = 128, //real-time event 114
    SIG115 = 129, //real-time event 115
    SIG116 = 130, //real-time event 116
    SIG117 = 131, //real-time event 117
    SIG118 = 132, //real-time event 118
    SIG119 = 133, //real-time event 119
    SIG120 = 134, //real-time event 120
    SIG121 = 135, //real-time event 121
    SIG122 = 136, //real-time event 122
    SIG123 = 137, //real-time event 123
    SIG124 = 138, //real-time event 124
    SIG125 = 139, //real-time event 125
    SIG126 = 140, //real-time event 126
    SIG127 = 141, //real-time event 127

    INFO = 142, //information request
    unknown = 143, //unknown signal

    EXC_BAD_ACCESS = 145, //could not access memory
    EXC_BAD_INSTRUCTION = 146, //illegal instruction/operand
    EXC_ARITHMETIC = 147, //arithmetic exception
    EXC_EMULATION = 148, //emulation instruction
    EXC_SOFTWARE = 149, //software generated exception
    EXC_BREAKPOINT = 150, //breakpoint

    LIBRT = 151, //librt internal signal
  };
}
#endif /* __BASE_GDB_SIGNALS_HH__ */
