/*
 * Copyright (c) 2008 The Hewlett-Packard Development Company
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

#ifndef __SIM_INIT_HH__
#define __SIM_INIT_HH__

#include <Python.h>

/*
 * Data structure describing an embedded python file.
 */
#include <list>

#include <inttypes.h>

#ifndef PyObject_HEAD
struct _object;
typedef _object PyObject;
#endif

struct EmbeddedPython
{
    const char *filename;
    const char *abspath;
    const char *modpath;
    const uint8_t *code;
    int zlen;
    int len;

    EmbeddedPython(const char *filename, const char *abspath,
                   const char *modpath, const uint8_t *code, int zlen, int len);

    PyObject *getCode() const;
    bool addModule() const;

    static EmbeddedPython *importer;
    static PyObject *importerModule;
    static std::list<EmbeddedPython *> &getList();
    static int initAll();
};

struct EmbeddedSwig
{
    void (*initFunc)();

    EmbeddedSwig(void (*init_func)());

    static std::list<EmbeddedSwig *> &getList();
    static void initAll();
};

int initM5Python();
int m5Main(int argc, char **argv);
PyMODINIT_FUNC initm5(void);

#endif // __SIM_INIT_HH__
