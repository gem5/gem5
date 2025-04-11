# gem5 Terminal Client

This directory contains two implementations of a terminal client that connects to gem5's terminal server:

- `term.c`: Original C implementation
- `gem5term`: Python implementation with identical functionality

## Purpose

These clients allow you to interact with a simulated system's serial port in gem5. This is particularly useful for:

- Debugging operating systems
- Interacting with console applications
- Monitoring boot messages
- Sending commands to the simulated system

### Caveats

Using the terminal client to run experiments will not lead to reproducible results.
This is because the terminal client is intended for interactive use and does not provide a controlled environment for running experiments.
To ensure reproducibility in your experiments, you should use a `workload` object that includes a runscript.
This runscript should be copied into gem5 and executed as part of the simulation. By doing so, you can automate the execution of your experiments and ensure that they can be consistently reproduced.

## Connection Methods

### TCP/IP Connection (Default)

After running gem5 with a `Terminal` SimObject you can connect using either client:

```bash
# Using C client
./term localhost 3456

# Using Python client
./gem5term.py localhost 3456
# or simply (localhost is default)
./gem5term.py 3456
```

Note: This is the default for all gem5 full system simulations.

### Unix Domain Socket Connection

1. In your gem5 configuration script set the port of the terminal object. For example on the `X86Board` you can add the line

```python
board.pc.com_1.device.port = "/tmp/gem5.term0"
```

after setting the board and workload:

```python
board = X86Board(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)
workload = obtain_resource("x86-ubuntu-24.04-boot-no-systemd")
board.set_workload(workload)

board.pc.com_1.device.port = "/tmp/gem5.term0"
```

2. Connect using either client:

```bash
# Using C client
./term --unix /tmp/gem5.term0

# Using Python client
./gem5term.py --unix /tmp/gem5.term0
```

### Why use unix sockets?

- **Security**: Unix domain sockets are more secure than TCP/IP sockets as they are only accessible locally
- **Performance**: Unix domain sockets are faster than TCP/IP sockets as they don't require network stack processing
- **Container Usage**: Unix domain sockets are useful when running gem5 in containers as they don't require exposing network ports

## Usage Tips

1. **Escape Sequence**: Use `~.` to exit the terminal client (both implementations)
2. **Container Usage**: Unix domain sockets are useful when running gem5 in containers as they don't require exposing network ports
