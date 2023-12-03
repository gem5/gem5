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

#include "sim/linear_solver.hh"

namespace gem5
{

std::vector<double>
LinearSystem::solve() const
{
    // Solve using gauss elimination, not ideal for big matrices
    std::vector<LinearEquation> smatrix = this->matrix;

    unsigned order = smatrix.size();
    for (unsigned row = 0; row < order - 1; row++) {
        // Look for a non-zero row, and swap
        for (unsigned i = row; i < order; i++) {
            if (smatrix[i][row] != 0.0f) {
                if (i != row) {
                    LinearEquation tmp = smatrix[i];
                    smatrix[i] = smatrix[row];
                    smatrix[row] = tmp;
                }
                break;
            }
        }

        // Divide row by leading number to make it 1.0
        smatrix[row] *= (1.0f / smatrix[row][row]);

        // Add it (properly scaled) to the rows below
        for (unsigned i = row + 1; i < order; i++) {
            LinearEquation t = smatrix[row];
            t *= -1.0f * smatrix[i][row];
            smatrix[i] = smatrix[i] + t;
        }
    }

    // smatrix is now a triangular matrix with diagonal being 1
    // Just backproagate variable values from order-1 till 0
    std::vector<double> ret(order, 0.0f);
    for (int row = order - 1; row >= 0; row--) {
        // Unknown value
        ret[row] = -smatrix[row][smatrix[row].cnt()] / smatrix[row][row];
        // Propagate variable in the cnt term
        for (int i = row - 1; i >= 0; i--) {
            smatrix[i][smatrix[i].cnt()] += ret[row] * smatrix[i][row];
            smatrix[i][row] = 0.0f;
        }
    }

    return ret;
}

} // namespace gem5
