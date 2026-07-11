# CPU Model Agent Notes

CPU models in this directory are mostly ISA-agnostic. They fetch bytes through
ISA-specific decoders, receive `StaticInst` objects, and execute those
instructions through model-specific pipeline, dependency, and memory-access
machinery.

## Instruction Interface

ISA code defines decoding and instruction semantics; CPU models define timing,
ordering, speculation, commit, and memory interaction. The shared contract is
centered on `StaticInst`, `ExecContext`, `ThreadContext`, `PCStateBase`, and
the architectural register abstractions. Be careful when changing these
interfaces: updates may affect every ISA and multiple CPU models.

`StaticInst` records instruction properties, operands, op class, branch
targets, microop state, and execution hooks. CPU models should use these
properties rather than open-coding ISA-specific behavior. Instruction execution
uses an `ExecContext` implementation supplied by the CPU model, so operand
indexing and misc-register access must follow the instruction's operand tables.

## Model Differences

AtomicSimpleCPU and TimingSimpleCPU execute instructions in simple in-order
loops with different memory timing behavior. MinorCPU models an in-order
pipeline with explicit stage timing. O3CPU wraps decoded `StaticInst`s in
dynamic instructions, renames registers, schedules dependencies, speculates,
and commits in order. A fix that is correct for one model may need separate
checks for the others.

When changing instruction flow, memory requests, faults, branches, or
squashing, trace the path through fetch/decode/execute/commit for the relevant
CPU model. For O3, distinguish `StaticInst` architectural semantics from
`DynInst` lifetime, request ownership, and squash/commit behavior.
