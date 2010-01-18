#! /usr/bin/env python2.6

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
    print dirname
    agg_name = "-".join(args)
    print agg_name
    fullpath = os.path.join("..", dirname, "cpt." + agg_name + ".10000")
    if not os.path.isdir(fullpath):
        os.system("mkdir -p " + fullpath)

    myfile = open(fullpath + "/system.physmem.physmem", "wb+")
    merged_mem = gzip.GzipFile(fileobj=myfile, mode="wb")

    max_curtick = 0
    when = 0
    for (i, arg) in enumerate(args):
        config = myCP()
        config.readfp(open(cpts[i] + "/m5.cpt"))

        for sec in config.sections():
            if re.compile("cpu").search(sec):
                newsec = re.sub("cpu", "cpu" + str(i), sec)
                merged.add_section(newsec)

                items = config.items(sec)
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
            elif sec == "system":
                pass
            elif sec == "Globals":
                tick = config.getint(sec, "curTick")
                if tick > max_curtick:
                    max_curtick = tick
                    when = config.getint("system.cpu.tickEvent", "_when")
            else:
                if i == 0:
                    print sec
                    merged.add_section(sec)
                    for item in config.items(sec):
                        merged.set(sec, item[0], item[1])
                        if item[0] == "curtick":
                            merged.optionxform(str("curTick"))
                        elif item[0] == "numevents":
                            merged.optionxform(str("numEvents"))

        page_ptr = page_ptr + int(config.get("system", "page_ptr"))

        ### memory stuff
        f = open(cpts[i] + "/system.physmem.physmem", "rb")
        gf = gzip.GzipFile(fileobj=f, mode="rb")
        bytes = int(config.get("system", "page_ptr")) << 13
        print "bytes to be read: ", bytes

        bytesRead = gf.read(int(config.get("system", "page_ptr")) << 13)
        merged_mem.write(bytesRead)

        gf.close()
        f.close()

    merged.add_section("system")
    merged.set("system", "page_ptr", page_ptr)
    print "WARNING: "
    print "Make sure the simulation using this checkpoint has at least "
    if page_ptr > (1<<20):
        print "8G ",
    elif page_ptr > (1<<19):
        print "4G ",
    elif page_ptr > (1<<18):
        print "2G ",
    elif page_ptr > (1<<17):
        print "1G ",
    elif page_ptr > (1<<16):
        print "512KB ",
    else:
        print "this is a small sim, you're probably fine",
    print "of memory."

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

    (options, args) = parser.parse_args()

    aggregate(options, args)

