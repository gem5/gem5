/*
 * Copyright (c) 2005 The Regents of The University of Michigan
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

static const int RTC_SEC = 0x00;
static const int RTC_SEC_ALRM = 0x01;
static const int RTC_MIN = 0x02;
static const int RTC_MIN_ALRM = 0x03;
static const int RTC_HR = 0x04;
static const int RTC_HR_ALRM = 0x05;
static const int RTC_DOW = 0x06;
static const int RTC_DOM = 0x07;
static const int RTC_MON = 0x08;
static const int RTC_YEAR = 0x09;

static const int RTC_STAT_REGA = 0x0A;

static const int RTCA_DV_4194304HZ = 0x0;
static const int RTCA_DV_1048576HZ = 0x1;
static const int RTCA_DV_32768HZ = 0x2;
static const int RTCA_DV_DISABLED0 = 0x6;
static const int RTCA_DV_DISABLED1 = 0x7;

static const int RTCA_RS_DISABLED = 0x0;
static const int RTCA_RS_1024HZ = 0x6;

static const int RTC_STAT_REGB = 0x0B;

static const int RTC_STAT_REGC = 0x0C;
static const int RTC_STAT_REGD = 0x0D;
