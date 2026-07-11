# ISA Agent Notes

The architecture directories define ISA-specific state, decoding, execution
semantics, disassembly, faults, MMUs, TLBs, interrupts, and workload support.
Most ISAs combine hand-written C++/Python with generated C++ from `.isa` files.

## Generated ISA Code

The ISA parser in `src/arch/isa_parser` reads each ISA's `isa/main.isa` and the
files it includes, such as `decoder.isa`, `operands.isa`, `bitfields.isa`, and
`includes.isa`. The generated code creates decoder tables and `StaticInst`
classes used by CPU models. Do not edit generated files under `build/`; change
the `.isa` input, parser support code, or hand-written ISA files instead.

`decoder.isa` describes instruction decode patterns and semantic code.
`operands.isa` maps ISA operands onto gem5 register classes and memory
operands. `bitfields.isa` names encoding fields. `includes.isa` injects C++
used by generated instruction classes. Keep decode predicates, operand
definitions, and disassembly in sync when adding an instruction.

## ISA Contracts

Generated instructions execute through the ISA-neutral `StaticInst` interface
in `src/cpu/static_inst.hh`. Instruction semantic code normally accesses CPU
state through `ExecContext`, advances an ISA-specific `PCState`, and returns a
`Fault`. The same decoded instruction may be consumed by AtomicSimpleCPU,
TimingSimpleCPU, MinorCPU, O3CPU, or checker/trace code, so avoid assumptions
about one CPU pipeline unless the ISA code explicitly requires it.

When adding or changing instructions, check decode, execution, disassembly,
register operands, memory flags, faults, and PC advancement. For macroop or
microop ISAs such as x86, preserve the macro/micro instruction contract and
micro-PC behavior.
