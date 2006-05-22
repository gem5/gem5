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

#define RTC_SEC                 0x00
#define RTC_SEC_ALRM            0x01
#define RTC_MIN                 0x02
#define RTC_MIN_ALRM            0x03
#define RTC_HR                  0x04
#define RTC_HR_ALRM             0x05
#define RTC_DOW                 0x06
#define RTC_DOM                 0x07
#define RTC_MON                 0x08
#define RTC_YEAR                0x09

#define RTC_STAT_REGA           0x0A
#define  RTCA_1024HZ            0x06  /* 1024Hz periodic interrupt frequency */
#define  RTCA_32768HZ           0x20  /* 22-stage divider, 32.768KHz timebase */
#define  RTCA_UIP               0x80  /* 1 = date and time update in progress */

#define RTC_STAT_REGB           0x0B
#define  RTCB_DST               0x01  /* USA Daylight Savings Time enable */
#define  RTCB_24HR              0x02  /* 0 = 12 hours, 1 = 24 hours */
#define  RTCB_BIN               0x04  /* 0 = BCD, 1 = Binary coded time */
#define  RTCB_SQWE              0x08  /* 1 = output sqare wave at SQW pin */
#define  RTCB_UPDT_IE           0x10  /* 1 = enable update-ended interrupt */
#define  RTCB_ALRM_IE           0x20  /* 1 = enable alarm interrupt */
#define  RTCB_PRDC_IE           0x40  /* 1 = enable periodic clock interrupt */
#define  RTCB_NO_UPDT           0x80  /* stop clock updates */

#define RTC_STAT_REGC           0x0C
#define RTC_STAT_REGD           0x0D

