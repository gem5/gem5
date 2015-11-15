#!/usr/bin/env bash
#scons build/ALPHA/gem5.fast
#scons build/ALPHA_MESI_Two_Level/gem5.opt
#scons build/ALPHA_MESI_Two_Level/gem5.debug TRACING_ON=1 -j8
scons build/ALPHA_MESI_Two_Level/gem5.opt -j8
#~ scons build/ARM/gem5.fast -j2
#scons build/ALPHA_MESI_CMP_directory/gem5.fast -j2
