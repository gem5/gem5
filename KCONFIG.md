This file explains how to work with gem5's implementation of the kconfig
configuration system, very similar to what's used by the linux kernel. It talks
about how to work with the Kconfig files themselves which define what user
adjustable configuration parameters there are, and how they work and
interoperate.

This file does *not*:

 * Describe how kconfig works generally. This is well documented elsewhere, for
    instance [here](
    https://www.kernel.org/doc/html/latest/kbuild/kconfig-language.html):
 * The settings in gem5's kconfig files. These should be documented with help
   text within the kconfig files, and in the code where these options are
   consumed.
 * The various tools which can manipulate a given configuration. These are
   documented in gem5's SCons help text.

# kconfiglib

gem5 uses the kconfiglib python library for consuming and manipulating kconfig
configurations. It is very similar to the kconfig implementation used by the
linux kernel, but is slightly different and has some small extensions added to
it. Almost all kconfig documentation written for the kernel's implementation
should apply here as well, but it may occasionally be necessary to refer to the
kconfiglib documentation.

Also, because gem5's build system is more modular than kconfig's design
supports out of the box, particularly for "choice" blocks, we have extended
kconfiglib and added a "cont_choice" keyword. This keyword is very similar to
"choice", except "cont_choice" blocks can be re-opened and extended with more
options further into the config.

This can be used to set up a central point where the user can choose between
mutually exclusive options, and still allow new Kconfig files to add new
options without modifying the original source.

Having to choose between mutually exclusive options should be avoided in
general, but is unavoidable in a few key places in gem5 at the moment. Once
those areas have been addressed, this keyword may be removed in the future.

# The 'CONF' dict in the SCons environment

In "env" SCons environment in SConscript files, or the "main" environment in
SConsopts files, can hold many variables which help SCons operate generally,
like setting what include paths to use, what the compiler command line is, etc.
These environments each have a 'CONF' sub-dict which holds all the variables
which are actually used to configure gem5, and not to configure SCons and the
build process itself.

All variables in this dict are automatically available to include in c++. To
access the value of env['CONF']['FOO'], you would #include "config/foo.hh".
Because these variables are in a shared namespace, their names should be unique
and distinctive.

These values are available in config scripts through the m5.defines.buildEnv
dict.

# Automatic/measured configuration values.

Some configuration values are not set by the user, and are measured from the
host environment. These could reflect the availability of a header file,
library or tool, whether the compiler supports a particular option or language
feature, etc.

These values should be measured in SConsopts files, and stored in the 'CONF'
dict described above. Like any other variable in 'CONF', they are then
available to C++ through generated header files, to config scripts through
buildEnv, etc. They are also available in the kconfig files themselves through
a mechanism discussed below.

# Accessing 'CONF' values in Kconfig files.

When the gem5 Kconfig files are processed to either manipulate a configuration
through a tool, or to apply a configuration to the gem5 build, all the values
in 'CONF' are temporarily put into environment variables. In the Kconfig files
themselves, these environment variables can be accessed using $(FOO) syntax,
which is described in kconfiglib's documentation.

Note that this is slightly different from the kernel's Kconfig syntax, where
the environment variables would have to be imported in using other keywords
first.

This is generally used to make automatic/measured settings which were
determined in SConsopts files available in Kconfig files. They can then be used
to compute dependencies, or to set default values, etc.

# Structure of the Kconfig hierarchy

Unlike SConscript files, gem5 does not find Kconfig files automatically, and
they are only used if they are included explicitly in other Kconfig files.

Kconfig options should be defined as close as possible to where they are used.
This makes them easier to find, keeps related functionality grouped
together in the source tree, and minimizes conflicts from modifying the same
few, central files when changing unrelated parts of gem5.

When including a Kconfig file in another, you should use the "rsource" keyword
which is a kconfiglib extension. This lets you include the other file using a
path which is relative to the current file, and also helps make the kconfig
files more modular and self contained.

# EXTRAS directories.

The EXTRAS variable can be set to a list of directories which hold additional
source that should be built into gem5. Because there's no way to know what (if
any) paths will be in EXTRAS ahead of time, it is not possible to explicitly
include kconfig files in those directories from a static file.

Instead, gem5's real root Kconfig file, which includes the one in src, is
automatically generated as part of the build. It uses the kconfiglib extension
"osource" to optionally source a file called Kconfig in the base of each EXTRAS
directory after it has sourced gem5's main Kconfig. If you want to add Kconfig
options to your EXTRAS directory, you can create that file, and then rsource
any additional internal Kconfig files as needed.
