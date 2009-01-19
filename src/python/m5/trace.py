import internal
import traceflags as flags

from internal.trace import clear, output, set, ignore

def enable():
    internal.trace.cvar.enabled = True

def help():
    print "Base Flags:"
    for flag in trace.flags.basic:
        print "    %s: %s" % (flag, trace.flags.descriptions[flag])
    print
    print "Compound Flags:"
    for flag in trace.flags.compound:
        if flag == 'All':
            continue
        print "    %s: %s" % (flag, trace.flags.descriptions[flag])
        print_list(trace.flags.compoundMap[flag], indent=8)
        print
