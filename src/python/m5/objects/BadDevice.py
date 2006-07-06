from m5.config import *
from Device import BasicPioDevice

class BadDevice(BasicPioDevice):
    type = 'BadDevice'
    devicename = Param.String("Name of device to error on")
