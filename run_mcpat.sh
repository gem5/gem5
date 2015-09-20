#!/usr/bin/env bash
ext/mcpat/GEM5ToMcPAT.py m5out/stats.txt m5out/config.json ext/mcpat/regression/test-0/power_region0.xml -o m5out/mcpat-out.xml
build/mcpat/mcpat -infile m5out/mcpat-out.xml -print_level 0
