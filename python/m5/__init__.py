from mpy_importer import AddToPath, LoadMpyFile

from config import *
config.add_param_types(config.__dict__)

from objects import *
config.add_param_types(objects.__dict__)

cpp_classes = config.MetaSimObject.cpp_classes
cpp_classes.sort()

