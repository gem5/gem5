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
#include <string>
#include <vector>

#include "base/str.hh"

int
main(int argc, char *argv[])
{
    using namespace std;

    if (argc != 3) {
        cout << "Usage: " << argv[0] << " <string> <token>\n";
        exit(1);
    }

    int i;
    string test = argv[1];
    vector<string> tokens1;
    vector<string> tokens2;
    char token = argv[2][0];

    cout << "string = \"" << test << "\", token = \'" << token << "\'\n";
    cout << "testing without ignore\n";
    tokenize(tokens1, test, token, false);

    if (tokens1.size()) {
        int size = tokens1.size();
        cout << "size = " << size << "\n";
        for (i = 0; i < size; i++) {
            cout << "'" << tokens1[i] << "' (" << tokens1[i].size()
                 << ")" << ((i == size - 1) ? "\n" : ", ");
        }
    } else {
        cout << "no tokens" << endl;
    }

    cout << "testing with ignore\n";
    tokenize(tokens2, test, token, true);

    if (tokens2.size()) {
        int size = tokens2.size();
        cout << "size = " << size << "\n";
        for (i = 0; i < size; i++) {
            cout << "'" << tokens2[i] << "' (" << tokens2[i].size()
                 << ")" << ((i == size - 1) ? "\n" : ", ");
        }
    } else {
        cout << "no tokens" << endl;
    }

    return 0;
}
