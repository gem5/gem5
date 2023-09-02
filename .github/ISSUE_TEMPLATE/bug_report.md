---
name: Bug report
about: Create a report to help us find and fix the bug
title: ''
labels: bug
assignees: ''

---

**Describe the bug**
A clear and concise description of what the bug is.

**Affects version**
State which version of gem5 this bug was found in. If on the develop branch state the Commit revision ID you are working.

**gem5 Modifications**
If you have modified gem5 in some way please state, to the best of your ability, how it has been modified.

**To Reproduce**
Steps to reproduce the behavior. Please assume starting from a clean repository:
1. Compile gem5 with command ...
2. Execute the simulation with...

If writing code, or a terminal command, use code blocks. Either an inline code block, `scons build/ALL/gem5.opt` (enclosed in two '`') or a multi-line codeblock:


```

int x=2;

int y=3'

print(x+y);

```

If possible, please include the Python configuration script used and state clearly any parameters passed.

**Terminal Output**
If applicable, add the terminal output here. If long, only include the relevant lines.
Please put the terminal output in code blocks. I.e.:

```

#Terminal output here#

```

**Expected behavior**
A clear and concise description of what you expected to happen.

**Host Operating System**
Ubuntu 22.04, Mac OS X, etc.

**Host ISA**
ARM, X86, RISC-V, etc.

**Compiler used**
State which compiler was used to compile gem5. Please include the compiler version.

**Additional information**
Add any other information which does not fit in the previous sections but may be of use in fixing this bug.
