# src/dev/serial/CMakeLists.txt
# Converted from src/dev/serial/SConscript

gem5_add_source(serial.cc)
gem5_add_source(simple.cc)
gem5_add_source(terminal.cc)
gem5_add_source(uart.cc)
gem5_add_source(uart8250.cc)

gem5_add_debug_flag(Terminal "Terminal debugging")
gem5_add_debug_flag(TerminalVerbose "TerminalVerbose debugging")
gem5_add_debug_flag(Uart "Uart debugging")

gem5_add_simobject(Serial.py SIM_OBJECTS SerialDevice SerialNullDevice)
gem5_add_simobject(Terminal.py SIM_OBJECTS Terminal ENUMS TerminalDump)
gem5_add_simobject(Uart.py SIM_OBJECTS Uart SimpleUart Uart8250)
