# configs/common/DualCacheSelector.py

from m5.params import *
from m5.SimObject import SimObject

class DualCacheSelector(SimObject):
    type = 'DualCacheSelector'
    cxx_header = "mem/cache/DualCacheSelector.hh"

    # 定义SimObject的参数
    # ...
