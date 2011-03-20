# Copyright (c) 2009 The Regents of The University of Michigan
# Copyright (c) 2011 Advanced Micro Devices, Inc.
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
#
# Authors: Lisa Hsu

from ConfigParser import ConfigParser
import gzip

import sys, re, optparse, os

class myCP(ConfigParser):
    def __init__(self):
        ConfigParser.__init__(self)

    def optionxform(self, optionstr):
        return optionstr

def aggregate(options, args):
    merged = myCP()
    page_ptr = 0

    allfiles = os.listdir(os.getcwd())
    cpts = []
    for arg in args:
        found = False
        for f in allfiles:
            if re.compile("cpt." + arg + ".\d+").search(f):
                found = True
                cpts.append(f)
                break
        if not found:
            print "missing checkpoint: ", arg
            sys.exit(1)

    dirname = "-".join([options.prefix, "cpt"])
    agg_name = "-".join(args)
    print agg_name
    fullpath = os.path.join("..", dirname, "cpt." + agg_name + ".10000")
    if not os.path.isdir(fullpath):
        os.system("mkdir -p " + fullpath)
    elif os.path.isfile(fullpath + "/system.physmem.physmem"):
        if os.path.isfile(fullpath + "/m5.cpt"):
            print fullpath, " already done"
            return

    myfile = open(fullpath + "/system.physmem.physmem", "wb+")
    merged_mem = gzip.GzipFile(fileobj=myfile, mode="wb")

    max_curtick = 0
    when = 0
    for (i, arg) in enumerate(args):
        print arg
        config = myCP()
        config.readfp(open(cpts[i] + "/m5.cpt"))

        for sec in config.sections():
            if re.compile("cpu").search(sec):
                newsec = re.sub("cpu", "cpu" + str(i), sec)
                merged.add_section(newsec)
                if re.compile("workload$").search(sec):
                    merged.set(newsec, "M5_pid", i)

                items = config.items(sec)
                if options.alpha:
                    for item in items:
                        if item[0] == "ppn":
                            if config.getint(sec, "tag") != 0:
                                merged.set(newsec, item[0], int(item[1]) + page_ptr)
                                continue
                        elif item[0] == "asn":
                            tmp = re.compile("(.*).Entry(\d+)").search(sec).groups()
                            if config.has_option(tmp[0], "nlu"):
                                size = config.getint(tmp[0], "nlu")
                                if int(tmp[1]) < size:
                                    merged.set(newsec, item[0], i)
                                    continue
                            else:
                                merged.set(newsec, item[0], i)
                                continue
                        merged.set(newsec, item[0], item[1])
                else:a #x86
                    for item in items:
                        if item[0] == "paddr":
                            merged.set(newsec, item[0], int(item[1]) + (page_ptr << 12))
                            continue
                        merged.set(newsec, item[0], item[1])

            elif sec == "system":
                pass
            elif sec == "Globals":
                tick = config.getint(sec, "curTick")
                if tick > max_curtick:
                    max_curtick = tick
                    when = config.getint("system.cpu.tickEvent", "_when")
            else:
                if i == 0:
                    merged.add_section(sec)
                    for item in config.items(sec):
                        merged.set(sec, item[0], item[1])
                        if item[0] == "curtick":
                            merged.optionxform(str("curTick"))
                        elif item[0] == "numevents":
                            merged.optionxform(str("numEvents"))

        page_ptr = page_ptr + int(config.get("system", "pagePtr"))

        ### memory stuff
        f = open(cpts[i] + "/system.physmem.physmem", "rb")
        gf = gzip.GzipFile(fileobj=f, mode="rb")
        pages = int(config.get("system", "pagePtr"))
        print "pages to be read: ", pages

        x = 0
        while x < pages:
            if options.alpha:
                bytesRead = gf.read(1 << 13)
            else: #x86
                bytesRead = gf.read(1 << 12)
            merged_mem.write(bytesRead)
            x += 1

        gf.close()
        f.close()

    merged.add_section("system")
    merged.set("system", "pagePtr", page_ptr)
    merged.set("system", "nextPID", len(args))

    print "WARNING: "
    print "Make sure the simulation using this checkpoint has at least ",
    if options.alpha:
        print page_ptr, "x 8K of memory"
    else:  # assume x86
        print page_ptr, "x 4K of memory"

    merged.add_section("Globals")
    merged.set("Globals", "curTick", max_curtick)

    for i in xrange(len(args)):
        merged.set("system.cpu" + str(i) + ".tickEvent", "_when", when)

    merged.write(file(fullpath + "/m5.cpt", "wb"))
    merged_mem.close()
    myfile.close()

if __name__ == "__main__":

    parser = optparse.OptionParser()
    parser.add_option("--prefix", type="string", default="agg")
    # If not alpha, then assume x86.  Any other ISAs would need
    # extra stuff in this script to appropriately parse their page tables
    # and understand page sizes.
    parser.add_option("--alpha", action="store_true")

    (options, args) = parser.parse_args()

    aggregate(options, args)

