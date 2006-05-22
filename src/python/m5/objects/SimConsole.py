from m5 import *
class ConsoleListener(SimObject):
    type = 'ConsoleListener'
    port = Param.TcpPort(3456, "listen port")

class SimConsole(SimObject):
    type = 'SimConsole'
    append_name = Param.Bool(True, "append name() to filename")
    intr_control = Param.IntrControl(Parent.any, "interrupt controller")
    listener = Param.ConsoleListener("console listener")
    number = Param.Int(0, "console number")
    output = Param.String('console', "file to dump output to")
