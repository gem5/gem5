/*
 * Copyright (c) 2015 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#ifndef __SIM_LINEAR_SOLVER_HH__
#define __SIM_LINEAR_SOLVER_HH__

#include <cassert>
#include <sstream>
#include <string>
#include <vector>

namespace gem5
{

/**
 * This class describes a linear equation with constant coefficients.
 * The equation has a certain (variable) number of unkowns and it can hold
 * N+1 coefficients.
 */

class LinearEquation
{
  public:
    LinearEquation(unsigned unknowns)
    {
        eq = std::vector<double>(unknowns + 1, 0);
    }

    // Add two equations
    LinearEquation
    operator+(const LinearEquation &rhs)
    {
        assert(this->eq.size() == rhs.eq.size());

        LinearEquation res(this->eq.size() - 1);

        for (unsigned i = 0; i < res.eq.size(); i++)
            res.eq[i] = this->eq[i] + rhs.eq[i];

        return res;
    }

    // Multiply the equation by a constant
    LinearEquation &
    operator*=(const double cnt)
    {
        for (auto &c : eq)
            c *= cnt;

        return *this;
    }

    // Access a certain equation coefficient
    double &
    operator[](unsigned unkw)
    {
        assert(unkw < eq.size());
        return eq[unkw];
    }

    // Get a string representation
    std::string
    toStr() const
    {
        std::ostringstream oss;
        for (unsigned i = 0; i < eq.size(); i++) {
            if (i)
                oss << " + ";
            oss << eq[i];
            if (i != eq.size() - 1)
                oss << "*x" << i;
        }
        oss << " = 0";
        return oss.str();
    }

    // Index for the constant term
    unsigned
    cnt() const
    {
        return eq.size() - 1;
    }

  private:
    /** Coefficients */
    std::vector<double> eq;
};

class LinearSystem
{
  public:
    LinearSystem(unsigned unknowns)
    {
        for (unsigned i = 0; i < unknowns; i++)
            matrix.push_back(LinearEquation(unknowns));
    }

    LinearEquation &
    operator[](unsigned eq)
    {
        assert(eq < matrix.size());
        return matrix[eq];
    }

    std::string
    toStr() const
    {
        std::string r;
        for (auto &eq : matrix)
            r += eq.toStr() + "\n";
        return r;
    }

    std::vector<double> solve() const;

  private:
    std::vector<LinearEquation> matrix;
};

} // namespace gem5

#endif
