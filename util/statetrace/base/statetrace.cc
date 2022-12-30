/*
 * Copyright (c) 2006-2007 The Regents of The University of Michigan
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

#include <netinet/in.h>
#include <sys/ptrace.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <netdb.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

#include "base/arch_check.h"
#include "tracechild.hh"

using namespace std;

void
printUsage(const char * execName)
{
    cout << execName << " <options> -- <command> <arguments>" << endl;
    cout << "options:" << endl;
    cout << "         -h          print this help" << endl;
    cout << "         --host      remote m5 host to connect to" << endl;
    cout << "         -i          print initial stack state" << endl;
    cout << "         -nt         don't trace execution" << endl;
}

int
main(int argc, char * argv[], char * envp[])
{
    TraceChild * child = genTraceChild();
    string args;
    int startProgramArgs;

    //Parse the command line arguments
    bool printInitial = false;
    bool printTrace = true;
    string host = "localhost";

    if (argc == 1) {
        printUsage(argv[0]);
        return 0;
    }
    for (int x = 1; x < argc; x++) {
        if (!strcmp(argv[x], "-h")) {
            printUsage(argv[0]);
            return 0;
        }
        if (!strcmp(argv[x], "--host")) {
            x++;
            if (x >= argc) {
                cerr << "Incorrect usage.\n" << endl;
                printUsage(argv[0]);
                return 1;
            }
            host = argv[x];
        } else if (!strcmp(argv[x], "-i")) {
            printInitial = true;
        } else if (!strcmp(argv[x], "-nt")) {
            printTrace = false;
        } else if (!strcmp(argv[x], "--")) {
            x++;
            if (x >= argc) {
                cerr << "Incorrect usage.\n" << endl;
                printUsage(argv[0]);
                return 1;
            }
            startProgramArgs = x;
            break;
        } else {
            cerr << "Incorrect usage.\n" << endl;
            printUsage(argv[0]);
            return 1;
        }
    }
    if (!child->startTracing(argv[startProgramArgs],
                argv + startProgramArgs)) {
        cerr << "Couldn't start target program" << endl;
        return 1;
    }
    child->step();
    if (printInitial)
        child->outputStartState(cout);
    if (printTrace) {
        // Connect to m5
        bool portSet = false;
        int port;
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            cerr << "Error opening socket! " << strerror(errno) << endl;
            return 1;
        }
        struct hostent *server;
        server = gethostbyname(host.c_str());
        if (!server) {
            cerr << "Couldn't get host ip! " << strerror(errno) << endl;
            return 1;
        }
        struct sockaddr_in serv_addr;
        bzero((char *)&serv_addr, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        bcopy((char *)server->h_addr,
                (char *)&serv_addr.sin_addr.s_addr,
                server->h_length);
        serv_addr.sin_port = htons(8000);
        if (connect(sock, (sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
            cerr << "Couldn't connect to server! " << strerror(errno) << endl;
            return 1;
        }
        while (child->isTracing()) {
            if (!child->sendState(sock))
                break;
            child->step();
        }
    }
    if (!child->stopTracing()) {
        cerr << "Couldn't stop child" << endl;
        return 1;
    }
    return 0;
}
