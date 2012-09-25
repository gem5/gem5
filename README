This is the gem5 simulator.

For detailed information about building the simulator and getting
started please refer to:
* The main website:     http://www.gem5.org
* Documentation wiki:   http://www.gem5.org/Documentation 
* Doxygen generated:    http://www.gem5.org/docs
* Tutorials:            http://www.gem5.org/Tutorials


Specific pages of interest are:
http://www.gem5.org/Introduction
http://www.gem5.org/Build_System
http://www.gem5.org/Dependencies
http://www.gem5.org/Running_gem5

Short version:
External tools and required versions

To build gem5, you will need the following software:
g++ version 4.3 or newer.
Python, version 2.4 - 2.7 (we don't support Python 3.X). gem5 links in the 
    Python interpreter, so you need the Python header files and shared 
    library (e.g., /usr/lib/libpython2.4.so) in addition to the interpreter
    executable. These may or may not be installed by default. For example,
    on Debian/Ubuntu, you need the "python-dev" package in addition to the
    "python" package. If you need a newer or different Python installation
     but can't or don't want to upgrade the default Python on your system,
     see http://www.gem5.org/Using_a_non-default_Python_installation
SCons, version 0.98.1 or newer. SCons is a powerful replacement for make. 
    If you don't have administrator privileges on your machine, you can use the
    "scons-local" package to install scons in your m5 directory, or install SCons
    in your home directory using the '--prefix=' option.  
SWIG, version 1.3.34 or newer
zlib, any recent version. For Debian/Ubuntu, you will need the "zlib-dev" or
    "zlib1g-dev" package to get the zlib.h header file as well as the library
    itself.
m4, the macro processor.


4. In this directory, type 'scons build/<ARCH>/gem5.opt' where ARCH is one
of ALPHA, ARM, MIPS, POWER, SPARC, or X86. This will build an optimized version
of the gem5 binary (gem5.opt) for the the specified architecture.

If you have questions, please send mail to gem5-users@gem5.org

WHAT'S INCLUDED (AND NOT)
-------------------------

The basic source release includes these subdirectories:
 - gem5:
   - configs: example simulation configuration scripts
   - ext: less-common external packages needed to build gem5
   - src: source code of the gem5 simulator
   - system: source for some optional system software for simulated systems
   - tests: regression tests
   - util: useful utility programs and files

To run full-system simulations, you will need compiled system firmware
(console and PALcode for Alpha), kernel binaries and one or more disk images. 
Please see the gem5 download page for these items at http://www.gem5.org/Download 
