
# A wrapper around configs/learning_gem5/part1/two_level.py

# For some reason, this is implicitly needed by run.py
root = None

import m5

def run_test(root):
        # Called from tests/run.py

        # Add paths that we need
        m5.util.addToPath('../configs/learning_gem5/part1')
        m5.util.addToPath('../configs/common')

        # The path to this script is the only parameter. Delete it so we can
        # execute the script that we want to execute.
        import sys
        del sys.argv[1:]
        # Note: at this point, we could add options we want to test.
        # For instance, sys.argv.append('--l2_size=512kB')

        # Execute the script we are wrapping
        execfile('configs/learning_gem5/part1/two_level.py')
