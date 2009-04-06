from m5.SimObject import SimObject
from m5.params import *

class CPA(SimObject):
    type = 'CPA'

    enabled = Param.Bool(False, "Is Annotation enabled?")
    user_apps = VectorParam.String([], "List of apps to get symbols for")
