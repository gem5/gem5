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
 *
 * Authors: Ali Saidi
 *          Miguel Serrano
 *          Nathan Binkert
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
static const int RTCA_1024HZ = 0x06;  /* 1024Hz periodic interrupt frequency */
static const int RTCA_32768HZ = 0x20; /* 22-stage divider, 32.768KHz timebase */
static const int RTCA_UIP = 0x80;     /* 1 = date and time update in progress */

static const int RTC_STAT_REGB = 0x0B;
static const int RTCB_DST = 0x01;     /* USA Daylight Savings Time enable */
static const int RTCB_24HR = 0x02;    /* 0 = 12 hours, 1 = 24 hours */
static const int RTCB_BIN = 0x04;     /* 0 = BCD, 1 = Binary coded time */
static const int RTCB_SQWE = 0x08;    /* 1 = output sqare wave at SQW pin */
static const int RTCB_UPDT_IE = 0x10; /* 1 = enable update-ended interrupt */
static const int RTCB_ALRM_IE = 0x20; /* 1 = enable alarm interrupt */
static const int RTCB_PRDC_IE = 0x40; /* 1 = enable periodic clock interrupt */
static const int RTCB_NO_UPDT = 0x80; /* stop clock updates */

static const int RTC_STAT_REGC = 0x0C;
static const int RTC_STAT_REGD = 0x0D;

