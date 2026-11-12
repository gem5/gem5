# Performance Model Changes & Simulation Review Checklist

This checklist provides domain-specific verification items for reviewing gem5
Pull Requests (PRs) that modify simulator timing models, microarchitectural
structures, coherence protocols, ISA semantics, or Python `SimObject` and
standard library configurations.

Aligned with the core philosophy **"Always Measure One Level Deeper"**, every
model change impacting simulated performance or timing predictions should be
justified with rigorous simulation evidence.

-------------------------------------------------------------------------------

## 1. Simulator Performance & Timing Changes (Flag for Human Review)

- [ ] **Call Out Simulator Performance Changes Explicitly**: Might this PR
  change simulator performance or timing model behavior? If so, call this out
  explicitly and prominently in the review.
- [ ] **Emphasize Human Maintainer Review**: If the PR could change simulator
  performance, explicitly emphasize in the review that **it is really important
  for a human maintainer to review**.
- [ ] **Suggest Presenting Concrete Simulation Data**: Suggest to the submitter
  that they present concrete data (e.g., `stats.txt`, IPC, execution cycles, or
  relevant subsystem metrics) showing either:
  - **No Unintended Change**: Simulator results do not change (for pure
    refactors, code cleanups, or non-timing bug fixes), or
  - **Expected, Justified Change**: Simulator results change in an expected,
    justified way (for timing model fixes or performance improvements).
- [ ] **Hypothesis Stated**: Does the PR description clearly state what
  architectural or timing behavior is being changed and which specific
  simulation metrics (`stats.txt`) are expected to change?
- [ ] **Actual Data Contextualized**: Are observed `stats.txt` metrics compared
  against a known baseline (`develop`) or theoretical hardware bound (such as
  wire latency, pipeline width, or memory bandwidth limits)?
- [ ] **Measure One Level Deeper**: Are high-level metric shifts (`simTicks`,
  simulated runtime, overall IPC, or average request latency) explained through
  lower-level causal statistics?
  - *CPU / Core Changes (`src/cpu/`)*: IPC/CPI, branch predictor accuracy
    (MPKI, squashed instructions), pipeline stall cycles (IQ full, LSQ full,
    ROB full, register rename stalls), and functional unit busyness.
  - *Classic Cache Changes (`src/mem/cache/`)*: Demand hit/miss rates, MSHR
    occupancy and misses, writeback counts, cache line replacements, and
    prefetch accuracy/coverage.
  - *Ruby & Network Changes (`src/mem/ruby/`)*: SLICC controller state
    transition counts, TBE occupancy, Garnet/SimpleNetwork router contention
    cycles, link utilization, hop counts, and queueing latency.
  - *Memory Controller Changes (`src/mem/`)*: Read/write queue occupancy, DRAM
    row-buffer hit rate, bus turnaround penalties, and QoS scheduling delays.
- [ ] **Reproducibility**: Are the exact gem5 build target, configuration
  script (`src/python/gem5` or `configs/`), workload/resource, and command-line
  arguments documented so reviewers can reproduce the numbers?

-------------------------------------------------------------------------------

## 2. Verifying PR Correctness & Reproducing Bugs (30-Minute Timeout)

- [ ] **System Capability & Companion Skill Prerequisite**: Only attempt live
  bug reproduction or simulation execution if running on a system capable of
  running gem5 (noting that this must be accompanied by a skill to build and
  run gem5 in this context).
- [ ] **Reproduce the Bug First, Then Verify the Fix**: If there is an
  existing, reproducible way to see the bug (e.g., from the issue description,
  reproduction script, or test case), has the reviewing agent:
  1. Reproduced the bug first (before the PR change),
  2. Applied the PR change, and
  3. Confirmed that the bug goes away?
- [ ] **Attempt a Custom Reproduction When Feasible**: If no reproduction
  recipe is provided, has the reviewing agent considered whether it is feasible
  to construct its own minimal reproduction test or script?
- [ ] **Acknowledge When Reproduction Is Infeasible**: Recognize that live
  reproduction is often not possible for many reasons (e.g., it may not be
  feasible to provide a standalone way to reproduce, or reproducing takes too
  long).
- [ ] **Strict 30-Minute Simulation Timeout**: If running a simulation to
  reproduce a bug or verify the PR, has a **strict timeout of 30 minutes** been
  enforced?
- [ ] **Cross-Model Statistics Comparison**: When a fix should make CPU
  models or memory modes agree, was the same workload run on each relevant
  model and `stats.txt` compared (`simInsts`, `simOps`, subsystem stats), not
  only the program output?
- [ ] **Suggested Changes Verified**: Was every `suggestion`/`diff` in the
  review built and exercised before posting (then reverted *and rebuilt*), or
  explicitly labelled as not compiled/tested?
- [ ] **Review-Only Mode / Long Simulation Audit**: When live simulation is
  unavailable or exceeds 30 minutes, has the reviewer audited the contributor's
  provided `stats.txt` summary, baseline comparisons, and lower-level causal
  statistics for internal consistency?

-------------------------------------------------------------------------------

## 3. gem5 Subsystem Invariants & Determinism

- [ ] **No Generated File Edits**: Does the diff avoid modifying any generated
  files under `build/`? All changes to generated code must happen in `.isa`
  files (`src/arch/`), SLICC `.sm` files (`src/mem/ruby/protocol/`), Python
  `SimObject` declarations, or `build_tools/` scripts.
- [ ] **CPU Model Coverage (`src/cpu/AGENTS.md`)**:
  - Do changes to `StaticInst`, `ExecContext`, `ThreadContext`, or `PCStateBase`
    preserve compatibility across `AtomicSimpleCPU`, `TimingSimpleCPU`,
    `MinorCPU`, and `O3CPU`?
  - For `O3CPU` changes, are `DynInst` lifetime, memory request ownership,
    in-flight squashing, and drain/resume invariants preserved?
- [ ] **Memory Modes & Coherence (`src/mem/AGENTS.md`,
  `src/mem/ruby/AGENTS.md`)**:
  - Are `timing`, `atomic`, `atomic_noncaching`, and zero-time functional debug
    accesses handled properly?
  - Do `Request` or `Packet` changes work across both Classic caches and Ruby
    protocols?
- [ ] **Deterministic Randomness**: Do all random number generators and PRNG
  seeds use gem5's deterministic randomness API (`gem5::Random` / `random_mt`
  in `src/base/random.hh`) rather than `std::rand()`, unseeded static
  generators, wall-clock time, or `/dev/urandom`?
- [ ] **Logging & Checkpointing**:
  - Are `DPRINTF`, `warn`, `inform`, `fatal`, `panic_if`, and `gem5_assert`
    used instead of raw `printf`/`std::cout` or bare `assert()`?
  - Is new *architectural* state handled in `serialize()` and
    `unserialize()` for checkpoint compatibility? (Microarchitectural or
    derived state, such as predictor tables or prefetcher history, may restore
    cold; don't require it to be serialized unless correctness depends on it
    or the PR claims otherwise.)

-------------------------------------------------------------------------------

## 4. Standard Library, SimObjects & Modularity

- [ ] **Strongly Prefer the gem5 Standard Library over `configs/common/`
  (`src/python/AGENTS.md`, `configs/AGENTS.md`)**:
  - Does the PR strongly prefer the gem5 standard library (`src/python/gem5`,
    e.g., `gem5.components`, `gem5.simulate`, `gem5.resources`) over legacy
    `configs/common/` and `configs/deprecated/`?
  - Are any new configurations, examples, or scripts relying on legacy
    `configs/common/` flagged and guided toward modern gem5 standard library
    components and boards?
- [ ] **Standard Library Compatibility**:
  - Does the change preserve public interfaces in `src/python/gem5` (boards,
    processors, cache hierarchies, memory systems, resources, simulator)?
  - Are renamed or deprecated `SimObject` parameters wrapped with
    `DeprecatedParam` (`m5.params.deprecated_params`) and `m5.util.warn` with
    clear migration guidance?
- [ ] **SimObject & Pybind11 Synchronization**: Are Python `SimObject`
  parameters, enums, ports, `cxx_exports`, and pybind11 bindings synchronized
  with C++ `Params` usage and `SConscript` registration?
