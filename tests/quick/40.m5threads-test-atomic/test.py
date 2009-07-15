process = LiveProcess(executable = binpath('m5threads', 'test_atomic'),
                      cmd = ['test_atomic', str(nb_cores)])

for i in range(nb_cores):
    root.system.cpu[i].workload = process
