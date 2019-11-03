# example test compile and run parameters
# Note: the absolute path to the chdir-print binary should be specified
# in the run command even if running from the same folder. This is needed
# because chdir is executed before triggering a clone for the file read,
# and the cloned process won't be able to find the executable if a relative
# path is provided.

# compile examples
scons --default=X86 ./build/X86/gem5.opt PROTOCOL=MOESI_hammer
scons --default=X86 ./build/X86/gem5.opt PROTOCOL=MESI_Three_Level

# run parameters
<GEM5_ROOT>/build/X86/gem5.opt <GEM5_ROOT>/configs/example/se.py -c <GEM5_ROOT>/tests/test-progs/chdir-print/chdir-print -n2 --ruby


# example successful output for MESI_Three_Level:

<...>

**** REAL SIMULATION ****
info: Entering event queue @ 0.  Starting simulation...
warn: Replacement policy updates recently became the responsibility of SLICC state machines. Make sure to setMRU() near callbacks in .sm files!
cwd: /proj/research_simu/users/jalsop/gem5-mem_dif_debug/tests/test-progs/chdir-print/
cwd: /proc

<...>

processor       : 0
vendor_id       : Generic
cpu family      : 0
model           : 0
model name      : Generic
stepping        : 0
cpu MHz         : 2000
cache size:     : 2048K
physical id     : 0
siblings        : 2
core id         : 0
cpu cores       : 2
fpu             : yes
fpu exception   : yes
cpuid level     : 1
wp              : yes
flags           : fpu
cache alignment : 64

processor       : 1
vendor_id       : Generic
cpu family      : 0
model           : 0
model name      : Generic
stepping        : 0
cpu MHz         : 2000
cache size:     : 2048K
physical id     : 0
siblings        : 2
core id         : 1
cpu cores       : 2
fpu             : yes
fpu exception   : yes
cpuid level     : 1
wp              : yes
flags           : fpu
cache alignment : 64

SUCCESS
Exiting @ tick 2694923000 because exiting with last active thread context
