# ISA Agent Notes

The architecture directories define ISA-specific state, decoding, execution
semantics, disassembly, faults, MMUs, TLBs, interrupts, and workload support.
Most ISAs combine hand-written C++/Python with generated C++ from `.isa` files.

## Generated ISA Code

For ISAs registered with `ISADesc`, the parser in `src/arch/isa_parser` reads
the architecture's `isa/main.isa` and its includes. Common inputs include
decoder fragments, `operands.isa`, `bitfields.isa`, and `includes.isa`; exact
file organization differs by ISA. Generated decoder and instruction classes
are used through `StaticInst`. Do not edit generated files under `build/`;
change the `.isa` input, parser support code, or hand-written ISA files
instead.

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

## Validation

Build a target containing the affected ISA so the parser executes and the
generated C++ compiles. Add or run focused execution and disassembly coverage
for the changed encoding; a successful parser run alone does not validate
instruction semantics across CPU models.
