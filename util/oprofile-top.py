#! /usr/bin/env python
import sys
import re
import getopt
from categories import *

def category(app,sym):
    if re.search("vmlinux-2.6", app):
        name = sym
    else:
        name = app

    if categories.has_key(name):
        return categories[name]
    for regexp, cat in categories_re:
        if regexp.match(name):
            return cat
    print "no match for symbol %s" % name
    return 'other'

try:
   (opts, files) = getopt.getopt(sys.argv[1:], 'i')
except getopt.GetoptError:
        print "usage", sys.argv[0], "[-i] <files>"
        sys.exit(2)

showidle = True

for o,v in opts:
    if o == "-i":
        showidle = False
print files
f = open(files.pop())
total = 0
prof = {}
linenum  = 0
for line in f.readlines():
    line = re.sub("\(no symbols\)", "nosym", line)
    line = re.sub("anonymous.*", "nosym", line)
    linenum += 1
    if linenum < 4:
        continue
    (count, percent, app, sym) = line.split()
    #total += int(count)
    cat = category(app,sym)
    if cat != 'idle' or showidle:
      total += int(count)
      prof[cat] = prof.get(cat,0) + int(count)

cats = ['other', 'user', 'copy', 'bufmgt', 'stack', 'driver', 'interrupt', 'alignment' ]

if showidle:
   cats.insert(0,'idle')

#syms = [(i[1], i[0]) for i in prof.items()]
#syms.sort()
#for i in range(len(syms)):
#    print "%s -- %5.1f%% " % (prof[i][1], 100 * float(prof[i][0])/float(total))

for d in cats:
    if prof.has_key(d):
        print "%s -- %5.1f%% " % (d, 100 * float(prof[d])/float(total))

