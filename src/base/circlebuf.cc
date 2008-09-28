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

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>

#include "base/atomicio.hh"
#include "base/circlebuf.hh"
#include "base/cprintf.hh"
#include "base/intmath.hh"

using namespace std;

CircleBuf::CircleBuf(int l)
    : _rollover(false), _buflen(l), _size(0), _start(0), _stop(0)
{
    _buf = new char[_buflen];
}

CircleBuf::~CircleBuf()
{
    if (_buf)
        delete [] _buf;
}

void
CircleBuf::dump()
{
    cprintf("start = %10d, stop = %10d, buflen = %10d\n",
            _start, _stop, _buflen);
    fflush(stdout);
    atomic_write(STDOUT_FILENO, _buf, _buflen);
    atomic_write(STDOUT_FILENO, "<\n", 2);
}

void
CircleBuf::flush()
{
    _start = 0;
    _stop = 0;
    _rollover = false;
}

void
CircleBuf::read(char *b, int len)
{
    _size -= len;
    if (_size < 0)
        _size = 0;

    if (_stop > _start) {
        len = min(len, _stop - _start);
        memcpy(b, _buf + _start, len);
        _start += len;
    }
    else {
        int endlen = _buflen - _start;
        if (endlen > len) {
            memcpy(b, _buf + _start, len);
            _start += len;
        }
        else {
            memcpy(b, _buf + _start, endlen);
            _start = min(len - endlen, _stop);
            memcpy(b + endlen, _buf, _start);
        }
    }
}

void
CircleBuf::read(int fd, int len)
{
    _size -= len;
    if (_size < 0)
        _size = 0;

    if (_stop > _start) {
        len = min(len, _stop - _start);
        atomic_write(fd, _buf + _start, len);
        _start += len;
    }
    else {
        int endlen = _buflen - _start;
        if (endlen > len) {
            atomic_write(fd, _buf + _start, len);
            _start += len;
        }
        else {
            atomic_write(fd, _buf + _start, endlen);
            _start = min(len - endlen, _stop);
            atomic_write(fd, _buf, _start);
        }
    }
}

void
CircleBuf::read(int fd)
{
    _size = 0;

    if (_stop > _start) {
        atomic_write(fd, _buf + _start, _stop - _start);
    }
    else {
        atomic_write(fd, _buf + _start, _buflen - _start);
        atomic_write(fd, _buf, _stop);
    }

    _start = _stop;
}

void
CircleBuf::read(ostream &out)
{
    _size = 0;

    if (_stop > _start) {
        out.write(_buf + _start, _stop - _start);
    }
    else {
        out.write(_buf + _start, _buflen - _start);
        out.write(_buf, _stop);
    }

    _start = _stop;
}

void
CircleBuf::readall(int fd)
{
    if (_rollover)
        atomic_write(fd, _buf + _stop, _buflen - _stop);

    atomic_write(fd, _buf, _stop);
    _start = _stop;
}

void
CircleBuf::write(char b)
{
    write(&b, 1);
}

void
CircleBuf::write(const char *b)
{
    write(b, strlen(b));
}

void
CircleBuf::write(const char *b, int len)
{
    if (len <= 0)
        return;

    _size += len;
    if (_size > _buflen)
        _size = _buflen;

    int old_start = _start;
    int old_stop = _stop;

    if (len >= _buflen) {
        _start = 0;
        _stop = _buflen;
        _rollover = true;
        memcpy(_buf, b + (len - _buflen), _buflen);
        return;
    }

    if (_stop + len <= _buflen) {
        memcpy(_buf + _stop, b, len);
        _stop += len;
    } else {
        int end_len = _buflen - old_stop;
        _stop = len - end_len;
        memcpy(_buf + old_stop, b, end_len);
        memcpy(_buf, b + end_len, _stop);
        _rollover = true;
    }

    if ((old_start > old_stop && old_start < _stop) ||
        (old_start < old_stop && _stop < old_stop))
        _start = _stop + 1;
}
