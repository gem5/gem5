# src/base/CMakeLists.txt
# Converted from src/base/SConscript

# --- Sources ---
gem5_add_source(atomicio.cc)
gem5_add_source(bitfield.cc)
gem5_add_source(imgwriter.cc)
gem5_add_source(bmpwriter.cc)
gem5_add_source(channel_addr.cc)
gem5_add_source(cprintf.cc)
gem5_add_source(debug.cc)
gem5_add_source(fenv.cc CONDITION CONF_HAVE_FENV)
gem5_add_source(fiber.cc)
gem5_add_source(framebuffer.cc)
gem5_add_source(hostinfo.cc)
gem5_add_source(inet.cc)
gem5_add_source(inifile.cc)
gem5_add_source(logging.cc)
gem5_add_source(match.cc)
gem5_add_source(output.cc)
gem5_add_source(pixel.cc)
gem5_add_source(pollevent.cc)
gem5_add_source(random.cc)
gem5_add_source(remote_gdb.cc)
gem5_add_source(socket.cc)
gem5_add_source(statistics.cc)
gem5_add_source(str.cc)
gem5_add_source(time.cc)
gem5_add_source(version.cc)
gem5_add_source(temperature.cc)
gem5_add_source(trace.cc)
gem5_add_source(types.cc)

# Conditional: PNG support
if(CONF_HAVE_PNG)
    gem5_add_source(pngwriter.cc)
endif()

# --- Debug flags ---
gem5_add_debug_flag(Annotate "State machine annotation debugging")
gem5_add_debug_flag(AnnotateQ "State machine annotation queue debugging")
gem5_add_debug_flag(AnnotateVerbose "Dump all state machine annotation details")
gem5_add_debug_flag(GDBAcc "Remote debugger accesses")
gem5_add_debug_flag(GDBExtra "Dump extra information on reads and writes")
gem5_add_debug_flag(GDBMisc "Breakpoints, traps, watchpoints, etc.")
gem5_add_debug_flag(GDBRead "Reads to the remote address space")
gem5_add_debug_flag(GDBRecv "Messages received from the remote application")
gem5_add_debug_flag(GDBSend "Messages sent to the remote application")
gem5_add_debug_flag(GDBWrite "Writes to the remote address space")
gem5_add_debug_flag(SQL "SQL queries sent to the server")
gem5_add_debug_flag(Stats "Statistics management")
gem5_add_debug_flag(StatEvents "Statistics event tracking")

# Compound flags
gem5_add_debug_flag(GDBAll "All Remote debugging flags"
    COMPONENTS GDBMisc GDBAcc GDBRead GDBWrite GDBSend GDBRecv GDBExtra)
gem5_add_debug_flag(AnnotateAll "All Annotation flags"
    COMPONENTS Annotate AnnotateQ AnnotateVerbose)

# Format flags
gem5_add_debug_flag(FmtFlag
    "Show the --debug-flag that enabled each debug message" FORMAT)
gem5_add_debug_flag(FmtStackTrace
    "Print a stack trace after every debug message" FORMAT)
gem5_add_debug_flag(FmtTicksOff
    "Don't show tick count on debug messages" FORMAT)

# --- SimObjects ---
gem5_add_simobject(Graphics.py ENUMS ImageFormat)

# --- Subdirectories ---
add_subdirectory(filters)
add_subdirectory(gtest)
add_subdirectory(loader)
add_subdirectory(stats)
add_subdirectory(stl_helpers)
add_subdirectory(vnc)
