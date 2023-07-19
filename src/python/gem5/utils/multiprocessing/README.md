# gem5's wrapper around python multiprocessing

This module wraps python's multiprocessing module so that it works with gem5.
The multiprocessing module creates new python processes, but there is no way to customize the way these processes are created.
This wrapper extends the python multiprocessing to support passing new arguments to the python (or gem5 in this case) executable when a new process is created.

This code replicates some of the multiprocessing module implementation from the python standard library in gem5.
The goal of this code is to enable users to use a *single* set of python scripts to run and analyze a suite of gem5 simulations.

We must reimplement some of the multiprocessing module because it is not flexible enough to allow for customized command line parameter to the "python" executable (gem5 in our case).
To get around this, I extended the Process and context objects to be gem5 specific.

The next steps is to wrap the Process and Pool types with gem5-specific versions that will improve their usability for our needs.
With this changeset, these objects are usable, but it will require significant user effort to reach the goal of running/analyzing many different gem5 simulations.

## Example use

test.py:

```python
from gem5.utils.multiprocessing import Process, Pool
from sim import info, run_sim
if __name__ == '__m5_main__' or __name__ == '__main__':
    info('main line')
    p1 = Process(target=run_sim, args=('bob',))
    p2 = Process(target=run_sim, args=('jane',))
    p1.start()
    p2.start()
    p2.join()
    p1.join()
    with Pool(processes=4, maxtasksperchild=1) as pool:
        pool.map(run_sim, range(10))
```

sim.py:

```python
import os
def info(title):
    print(title)
    print('module name:', __name__)
    print('parent process:', os.getppid())
    print('process id:', os.getpid())
def run_sim(name):
    info('function g')
    from gem5.prebuilt.demo.x86_demo_board import X86DemoBoard
    from gem5.resources.resource import Resource
    from gem5.simulate.simulator import Simulator
    board = X86DemoBoard()
    board.set_kernel_disk_workload(
        kernel=obtain_resource("x86-linux-kernel-5.4.49"),
        disk_image=obtain_resource("x86-ubuntu-18.04-img"),
    )
    simulator = Simulator(board=board)
    simulator.run(max_ticks=10000000)
```

Then, you can run `gem5 test.py`.
This will execute `run_sim` 12 times.
The first two will run in parallel, then the last 10 will run in parallel with up to 4 running at once.

## Limitations

- This only supports the spawn context. This is important because we need a fresh gem5 process for every subprocess.
- When using `Pool`, the `maxtasksperchild` must be 1.
- Process synchronization (queues, pipes, etc.) hasn't been tested
- Functions that are used to execute in the subprocess must be imported from another module. In other words, we cannot pickle functions in the main/runner module.

## Implementation notes

- The `_start_method` must be `None` for the `Spawn_gem5Process` class. Otherwise, in `_bootstrap` in the `BaseProcess` it will try to force the `_start_method` to be gem5-specific, which the `multiprocessing` module doesn't understand.
