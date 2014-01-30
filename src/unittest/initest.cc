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
 *          Steve Reinhardt
 */

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "base/cprintf.hh"
#include "base/inifile.hh"

using namespace std;

char *progname;

void usage();

void
usage()
{
    cout << "Usage: " << progname << " <ini file>\n";
    exit(1);
}

#if 0
char *defines = getenv("CONFIG_DEFINES");
if (defines) {
    char *c = defines;
    while ((c = strchr(c, ' ')) != NULL) {
        *c++ = '\0';
        count++;
    }
    count++;
}

#endif

int
main(int argc, char *argv[])
{
    IniFile simConfigDB;

    progname = argv[0];

    for (int i = 1; i < argc; ++i) {
        char *arg_str = argv[i];

        // if arg starts with '-', parse as option,
        // else treat it as a configuration file name and load it
        if (arg_str[0] == '-') {
            // switch on second char
            switch (arg_str[1]) {
              case '-':
                // command-line configuration parameter:
                // '--<section>:<parameter>=<value>'

                if (!simConfigDB.add(arg_str + 2)) {
                    // parse error
                    ccprintf(cerr,
                             "Could not parse configuration argument '%s'\n"
                             "Expecting --<section>:<parameter>=<value>\n",
                             arg_str);
                    exit(0);
                }
                break;

              default:
                usage();
            }
        }
        else {
            // no '-', treat as config file name

            if (!simConfigDB.load(arg_str)) {
                cprintf("Error processing file %s\n", arg_str);
                exit(1);
            }
        }
    }

    string value;

#define FIND(C, E) \
  if (simConfigDB.find(C, E, value)) \
    cout << ">" << value << "<\n"; \
  else \
    cout << "Not Found!\n"

    FIND("General", "Test2");
    FIND("Junk", "Test3");
    FIND("Junk", "Test4");
    FIND("General", "Test1");
    FIND("Junk2", "test3");
    FIND("General", "Test3");

    cout << "\n";

    simConfigDB.dump();

    return 0;
}
