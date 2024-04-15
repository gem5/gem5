/*
 * Copyright 2019 Texas A&M University
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Daniel A. Jiménez
 *  Adapted to gem5 by: Javier Bueno Hedo
 *
 */

/*
 * Multiperspective Perceptron Predictor (by Daniel A. Jiménez)
 * - 8KB version
 */

#include "cpu/pred/multiperspective_perceptron_8KB.hh"

namespace gem5
{

namespace branch_prediction
{

MultiperspectivePerceptron8KB::MultiperspectivePerceptron8KB(
    const MultiperspectivePerceptron8KBParams &p)
    : MultiperspectivePerceptron(p)
{}

void
MultiperspectivePerceptron8KB::createSpecs()
{
    addSpec(new BIAS(2.40625, 0, 6, *this));
    addSpec(new GHIST(0, 19, 1.4375, 0, 6, *this));
    addSpec(new GHIST(0, 65, 1.0, 0, 6, *this));
    addSpec(new GHIST(21, 64, 1.0, 0, 6, *this));
    addSpec(new GHIST(75, 150, 1.0625, 0, 6, *this));
    addSpec(new GHISTMODPATH(0, 7, 3, 1.625, 0, 6, *this));
    addSpec(new GHISTPATH(11, 2, -1, 1.25, 0, 6, *this));
    addSpec(new GHISTPATH(15, 4, -1, 1.125, 0, 6, *this));
    addSpec(new GHISTPATH(31, 1, -1, 1.40625, 0, 6, *this));
    addSpec(new GHISTPATH(7, 1, -1, 1.5, 600, 6, *this));
    addSpec(new IMLI(4, 1.28125, 375, 6, *this));
    addSpec(new LOCAL(-1, 1.5625, 512, 6, *this));
    addSpec(new RECENCY(14, 4, -1, 1.25, 0, 6, *this));
    addSpec(new RECENCYPOS(31, 1.875, 0, 6, *this));
    addSpec(new SGHISTPATH(0, 4, 3, 1.65625, 0, 6, *this));
    addSpec(new SGHISTPATH(1, 2, 5, 2.53125, 0, 5, *this));
}

} // namespace branch_prediction
} // namespace gem5
