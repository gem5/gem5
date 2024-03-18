from gem5.isas import ISA
from gem5.simulate.multi_sim import MultiSim

config = [
    {
        "file": "configs/example/gem5_library/riscvmatched-hello.py",
        "function": "run_hello",
        "config_args": ["riscv-hello"],
    },
    {
        "file": "configs/example/gem5_library/riscvmatched-hello.py",
        "function": "run_hello",
        "config_args": ["riscv-hello"],
    },
]
multi_sim = MultiSim(config)
multi_sim.run_all(2)
