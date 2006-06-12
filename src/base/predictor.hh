/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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
 * Authors: Steve Raasch
 *          Nathan Binkert
 */

//
//  Abstract base class for a generic predictor
//
//

#ifndef __PREDICTOR_HH__
#define __PREDICTOR_HH__

class GenericPredictor {

  public:
    virtual void clear() = 0;

    virtual unsigned predict(unsigned long _index) = 0;
    virtual unsigned predict(unsigned long _index, unsigned &pdata) = 0;

    virtual unsigned peek(unsigned long _index) = 0;

    virtual void record(unsigned long _index, unsigned _actual_value,
                        unsigned _pred_value) = 0;
    virtual void record(unsigned long _index, unsigned _actual_value,
                        unsigned _pred_value, unsigned _pdata) = 0;

    virtual unsigned value(unsigned long _index) = 0;

    virtual void regStats() = 0;
    virtual void regFormulas() = 0;

    virtual ~GenericPredictor() {};
};

#endif //  __PREDICTOR_HH__
