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

#include <fcntl.h>
#include <unistd.h>

#include <iostream>

#include "base/circlebuf.hh"

const char *strings[] = {
    "This is the first test\n",
    "he went with his woman to the store\n",
    "the man with the bat hit the woman with the hat\n",
    "that that is is that that was\n",
    "sue sells sea shells by the sea shore\n",
    "go to the store and buy me some milk and bread\n",
    "the friendly flight attendants spoke soothingly to "
    "the frightened passengers in their native languages\n"
};

const int num_strings = sizeof(strings) / sizeof(char *);

int
main()
{
    CircleBuf buf(1024);

    for (int count = 0; count < 100; count++)
        buf.write(strings[count % num_strings]);
    buf.read(STDOUT_FILENO);
    write(STDOUT_FILENO, "<\n", 2);

    for (int count = 0; count < 100; count++)
        buf.write(strings[count % num_strings]);
    buf.read(STDOUT_FILENO, 100);
    write(STDOUT_FILENO, "<\n", 2);

    buf.flush();
    buf.write("asdfa asdf asd fasdf asdf\n");
    buf.write("");
    buf.write("");
    buf.write("");
    buf.write("");
    buf.write("");
    buf.write("");
    buf.read(STDOUT_FILENO);
    write(STDOUT_FILENO, "<\n", 2);
}
