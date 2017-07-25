/*
 * Copyright (c) 2017 Jason Lowe-Power
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
* Authors: Jason Lowe-Power
*/

#include <iostream>
#include <thread>

using namespace std;

/*
 * c = a + b
 */
void array_add(int *a, int *b, int *c, int tid, int threads, int num_values)
{
    for (int i = tid; i < num_values; i += threads) {
        c[i] = a[i] + b[i];
    }
}


int main(int argc, char *argv[])
{
    unsigned num_values;
    if (argc == 1) {
        num_values = 100;
    } else if (argc == 2) {
        num_values = atoi(argv[1]);
        if (num_values <= 0) {
            cerr << "Usage: " << argv[0] << " [num_values]" << endl;
            return 1;
        }
    } else {
        cerr << "Usage: " << argv[0] << " [num_values]" << endl;
        return 1;
    }

    unsigned cpus = thread::hardware_concurrency();

    cout << "Running on " << cpus << " cores. ";
    cout << "with " << num_values << " values" << endl;

    int *a, *b, *c;
    a = new int[num_values];
    b = new int[num_values];
    c = new int[num_values];

    if (!(a && b && c)) {
        cerr << "Allocation error!" << endl;
        return 2;
    }

    for (int i = 0; i < num_values; i++) {
        a[i] = i;
        b[i] = num_values - i;
        c[i] = 0;
    }

    thread **threads = new thread*[cpus];

    // NOTE: -1 is required for this to work in SE mode.
    for (int i = 0; i < cpus - 1; i++) {
        threads[i] = new thread(array_add, a, b, c, i, cpus, num_values);
    }
    // Execute the last thread with this thread context to appease SE mode
    array_add(a, b, c, cpus - 1, cpus, num_values);

    cout << "Waiting for other threads to complete" << endl;

    for (int i = 0; i < cpus - 1; i++) {
        threads[i]->join();
    }

    delete[] threads;

    cout << "Validating..." << flush;

    int num_valid = 0;
    for (int i = 0; i < num_values; i++) {
        if (c[i] == num_values) {
            num_valid++;
        } else {
            cerr << "c[" << i << "] is wrong.";
            cerr << " Expected " << num_values;
            cerr << " Got " << c[i] << "." << endl;
        }
    }

    if (num_valid == num_values) {
        cout << "Success!" << endl;
        return 0;
    } else {
        return 2;
    }
}
