/*
 * Copyright (c) 2006 The Regents of The University of Michigan
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * Copyright (c) 2013 Mark D. Hill and David A. Wood
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
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 */

#ifndef __SIM_CORE_HH__
#define __SIM_CORE_HH__

/** @file This header provides some core simulator functionality such as time
 * information, output directory and exit events
 */

#include <string>

#include "base/types.hh"
#include "sim/eventq.hh"

/// The universal simulation clock.
inline Tick curTick() { return _curEventQueue->getCurTick(); }

const Tick retryTime = 1000;

/// These are variables that are set based on the simulator frequency
///@{
namespace SimClock {
extern Tick Frequency; ///< The number of ticks that equal one second

namespace Float {

/** These variables equal the number of ticks in the unit of time they're
 * named after in a double.
 * @{
 */
extern double s;  ///< second
extern double ms; ///< millisecond
extern double us; ///< microsecond
extern double ns; ///< nanosecond
extern double ps; ///< picosecond
/** @} */

/** These variables the inverse of above. They're all < 1.
 * @{
 */
extern double Hz;  ///< Hz
extern double kHz; ///< kHz
extern double MHz; ///< MHz
extern double GHz; ///< GHz
/** @}*/
} // namespace Float

/** These variables equal the number of ticks in the unit of time they're
 *  named after in a 64 bit integer.
 *
 * @{
 */
namespace Int {
extern Tick s;  ///< second
extern Tick ms; ///< millisecond
extern Tick us; ///< microsecond
extern Tick ns; ///< nanosecond
extern Tick ps; ///< picosecond
/** @} */
} // namespace Int
} // namespace SimClock
/** @} */

void fixClockFrequency();
bool clockFrequencyFixed();

void setClockFrequency(Tick ticksPerSecond);
Tick getClockFrequency(); // Ticks per second.

void setOutputDir(const std::string &dir);

class Callback;
void registerExitCallback(Callback *callback);
void doExitCleanup();

#endif /* __SIM_CORE_HH__ */
