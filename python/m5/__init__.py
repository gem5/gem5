import sys, os

# define this here so we can use it right away if necessary
def panic(string):
    print >>sys.stderr, 'panic:', string
    sys.exit(1)

# Add given directory to system module search path, if it is not
# already there.
def AddToPath(path):
    path = os.path.realpath(path)
    if os.path.isdir(path) and path not in sys.path:
        sys.path.append(path)

# find the m5 compile options: must be specified as a dict in
# __main__.m5_build_env.
import __main__
if not hasattr(__main__, 'm5_build_env'):
    panic("__main__ must define m5_build_env")

# make a SmartDict out of the build options for our local use
import smartdict
build_env = smartdict.SmartDict()
build_env.update(__main__.m5_build_env)

# make a SmartDict out of the OS environment too
env = smartdict.SmartDict()
env.update(os.environ)

# import the main m5 config code
from config import *

# import the built-in object definitions
from objects import *

