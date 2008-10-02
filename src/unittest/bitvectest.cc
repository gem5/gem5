/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 */

#include <iostream>
#include <vector>

using namespace std;

int
main()
{
    vector<bool> v1(100);

    v1[0] = true;
    v1.resize(500);
    v1[100] = true;
    v1[499] = true;
    v1.resize(10000);
    v1[9999] = true;

    cout << "v1.size() = " << v1.size() << "\n";
    for (int i = 0; i < v1.size(); i++)
        if (v1[i])
            cout << "v1[" << i << "] = " << v1[i] << "\n";

    cout << "\n";

    vector<bool> v2 = v1;

    for (int i = 0; i < v2.size(); i++)
        if (v2[i])
            cout << "v2[" << i << "] = " << v2[i] << "\n";

    cout << "v1 " << ((v1 == v2) ? "==" : "!=") << " v2" << "\n";
    v2[8583] = true;
    cout << "v1 " << ((v1 == v2) ? "==" : "!=") << " v2" << "\n";
    v1[8583] = true;
    cout << "v1 " << ((v1 == v2) ? "==" : "!=") << " v2" << "\n";
    v1.resize(100000);
    cout << "v1 " << ((v1 == v2) ? "==" : "!=") << " v2" << "\n";
    cout << flush;
}
