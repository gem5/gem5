/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#include <algorithm>
#include <string>

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "base/circlebuf.hh"
#include "base/cprintf.hh"
#include "base/intmath.hh"

using namespace std;

CircleBuf::CircleBuf(int l)
  : rollover(false), buflen(l), size(0), start(0), stop(0)
{ buf = new char[buflen]; }

CircleBuf::~CircleBuf()
{ if (buf) delete [] buf; }

void
CircleBuf::dump()
{
  cprintf("start = %10d, stop = %10d, buflen = %10d\n", start, stop, buflen);
  fflush(stdout);
  ::write(STDOUT_FILENO, buf, buflen);
  ::write(STDOUT_FILENO, "<\n", 2);
}

void
CircleBuf::flush()
{
  start = 0;
  stop = 0;
  rollover = false;
}

void
CircleBuf::read(char *b, int len)
{
  size -= len;
  if (size < 0)
    size = 0;

  if (stop > start) {
    len = min(len, stop - start);
    memcpy(b, buf + start, len);
    start += len;
  }
  else {
    int endlen = buflen - start;
    if (endlen > len) {
      memcpy(b, buf + start, len);
      start += len;
    }
    else {
      memcpy(b, buf + start, endlen);
      start = min(len - endlen, stop);
      memcpy(b + endlen, buf, start);
    }
  }
}

void
CircleBuf::read(int fd, int len)
{
  size -= len;
  if (size < 0)
    size = 0;

  if (stop > start) {
    len = min(len, stop - start);
    ::write(fd, buf + start, len);
    start += len;
  }
  else {
    int endlen = buflen - start;
    if (endlen > len) {
      ::write(fd, buf + start, len);
      start += len;
    }
    else {
      ::write(fd, buf + start, endlen);
      start = min(len - endlen, stop);
      ::write(fd, buf, start);
    }
  }
}

void
CircleBuf::read(int fd)
{
  size = 0;

  if (stop > start) {
    ::write(fd, buf + start, stop - start);
  }
  else {
    ::write(fd, buf + start, buflen - start);
    ::write(fd, buf, stop);
  }

  start = stop;
}

void
CircleBuf::readall(int fd)
{
  if (rollover)
    ::write(fd, buf + stop, buflen - stop);

  ::write(fd, buf, stop);
  start = stop;
}

void
CircleBuf::write(char b)
{ write(&b, 1); }

void
CircleBuf::write(const char *b)
{ write(b, strlen(b)); }

void
CircleBuf::write(const char *b, int len)
{
  if (len <= 0)
    return;

  size += len;
  if (size > buflen)
      size = buflen;

  int old_start = start;
  int old_stop = stop;

  if (len >= buflen) {
    start = 0;
    stop = buflen;
    rollover = true;
    memcpy(buf, b + (len - buflen), buflen);
    return;
  }

  if (stop + len <= buflen) {
    memcpy(buf + stop, b, len);
    stop += len;
  } else {
    int end_len = buflen - old_stop;
    stop = len - end_len;
    memcpy(buf + old_stop, b, end_len);
    memcpy(buf, b + end_len, stop);
    rollover = true;
  }

  if (old_start > old_stop && old_start < stop ||
      old_start < old_stop && stop < old_stop)
    start = stop + 1;
}
