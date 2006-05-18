from m5 import *

import os.path
import sys

# Edit the following list to include the possible paths to the binary
# and disk image directories.  The first directory on the list that
# exists will be selected.
SYSTEMDIR_PATH = ['/n/poolfs/z/dist/m5/system']

SYSTEMDIR = None
for d in SYSTEMDIR_PATH:
    if os.path.exists(d):
        SYSTEMDIR = d
        break

if not SYSTEMDIR:
    print >>sys.stderr, "Can't find a path to system files."
    sys.exit(1)

BINDIR = SYSTEMDIR + '/binaries'
DISKDIR = SYSTEMDIR + '/disks'

def disk(file):
    return os.path.join(DISKDIR, file)

def binary(file):
    return os.path.join(BINDIR, file)

def script(file):
    return os.path.join(SYSTEMDIR, 'boot', file)

