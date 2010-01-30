from m5.params import *
from m5.SimObject import SimObject

class RubyDebug(SimObject):
    type = 'RubyDebug'
    cxx_class = 'Debug'

    filter_string = Param.String('none',
        "a string for filtering debugging output (see Debug.h)")
    verbosity_string = Param.String('none',
        "filters debugging messages based on priority (low, med, high)")
    output_filename = Param.String('none',
        "sends debugging messages to a file")
    start_time = Param.Tick(1,
        "filters debugging messages based on a ruby time")
    # For debugging purposes, one can enable a trace of all the protocol
    # state machine changes. Unfortunately, the code to generate the
    # trace is protocol specific. To enable the code for some of the
    # standard protocols,
    #   1. change protocol_trace = true
    #   2. enable debug in the Ruby Makefile
    protocol_trace = Param.Bool(False,
        "enable protocol state machine trace")
