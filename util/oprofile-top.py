#! /usr/bin/env python3

# Copyright (c) 2005 The Regents of The University of Michigan
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Parse sampled function profile output (quick hack).

import sys
import re
import getopt
from categories import *


def category(app, sym):
    if re.search("vmlinux-2.6", app):
        name = sym
    else:
        name = app

    if name in categories:
        return categories[name]
    for regexp, cat in categories_re:
        if regexp.match(name):
            return cat
    print("no match for symbol %s" % name)
    return "other"


try:
    (opts, files) = getopt.getopt(sys.argv[1:], "i")
except getopt.GetoptError:
    print("usage", sys.argv[0], "[-i] <files>")
    sys.exit(2)

showidle = True

for o, v in opts:
    if o == "-i":
        showidle = False
print(files)
f = open(files.pop())
total = 0
prof = {}
linenum = 0
for line in f.readlines():
    line = re.sub("\(no symbols\)", "nosym", line)
    line = re.sub("anonymous.*", "nosym", line)
    linenum += 1
    if linenum < 4:
        continue
    (count, percent, app, sym) = line.split()
    # total += int(count)
    cat = category(app, sym)
    if cat != "idle" or showidle:
        total += int(count)
        prof[cat] = prof.get(cat, 0) + int(count)

cats = [
    "other",
    "user",
    "copy",
    "bufmgt",
    "stack",
    "driver",
    "interrupt",
    "alignment",
]

if showidle:
    cats.insert(0, "idle")

# syms = [(i[1], i[0]) for i in prof.items()]
# syms.sort()
# for i in range(len(syms)):
#    print "%s -- %5.1f%% " % (prof[i][1], 100 * float(prof[i][0])/float(total))

for d in cats:
    if d in prof:
        print("%s -- %5.1f%% " % (d, 100 * float(prof[d]) / float(total)))
