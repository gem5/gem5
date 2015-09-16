
# A wrapper around configs/learning_gem5/part1/simple.py

# For some reason, this is implicitly needed by run.py
root = None

def run_test(root):
        # Called from tests/run.py

        # Execute the script we are wrapping
        execfile('configs/learning_gem5/part1/simple.py')
