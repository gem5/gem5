import sys, os

# define this here so we can use it right away if necessary
def panic(string):
    print >>sys.stderr, 'panic:', string
    sys.exit(1)

# the mpy import code is added to the global import meta_path as a
# side effect of this import
from mpy_importer import AddToPath, LoadMpyFile

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
config.add_param_types(config)

# import the built-in object definitions
from objects import *
config.add_param_types(objects)

cpp_classes = config.MetaSimObject.cpp_classes
cpp_classes.sort()

