"""Wrapper rule for the gem5 executable.

Provides gem5_binary() macro that creates a cc_binary with proper
whole-archive linking for SimObject registration and EmbeddedPython
global constructors.
"""

load("@rules_cc//cc:defs.bzl", "cc_binary")

def gem5_binary(name, srcs = [], deps = [], linkopts = [],
                visibility = None, **kwargs):
    """Create a gem5 executable with proper constructor retention.

    All dependency libraries should use alwayslink = True to ensure
    SimObject factories and EmbeddedPython global constructors are
    retained in the final binary.

    Args:
        name: Target name.
        srcs: Direct source files (typically main.cc, date.cc).
        deps: Library dependencies (gem5_all and others).
        linkopts: Additional linker flags.
        visibility: Visibility.
    """
    cc_binary(
        name = name,
        srcs = srcs,
        deps = deps,
        linkopts = linkopts + [
            "-lpthread",
            "-lz",
            "-lrt",
            "-ldl",
        ],
        visibility = visibility,
        **kwargs
    )
