# NITC Project Status: Speculative Data Defense (LVP / FLOP)

---

## [Phase 0] Tooling & Setup ✅ COMPLETE
- [x] ARM Binary Compiled (Dummy Program)
- [x] simple-arm.py Verified
- [x] libcapstone-dev installed (capstone header still missing — cosmetic only)
- [x] qemu-user-static installed
- [x] Konata Downloaded (User manually tracking)
- [x] Branch: `research/p1-o3-setup`

---

## [Phase 1] O3 Infrastructure ✅ COMPLETE
- [x] O3 Configuration Script (`flop_research/phase1/o3_arm_test.py`)
  - ArmO3CPU, ROB=256, IQ=120, LSQ=128
- [x] Pipeline Trace Captured (`trace.out` viewed in Konata)
- [x] Hello World exits at tick `72052000`
- [x] Branch: `research/p1-o3-setup`

---

## [Phase 2] LVP Module C++ ✅ COMPLETE
- [x] LVP PC-Tagged Table implementation (`src/cpu/o3/lvp.hh` + `lvp.cc`)
- [x] 8-bit saturating confidence counter (threshold = 250)
- [x] Strong-bias reset on mismatch (counter → 0 immediately)
- [x] 8-byte pointer load heuristic filter (skips 64-bit loads)
- [x] Public API: `lookup()`, `update()`, `clear()`
- [x] gem5 Stats group: `totalLookups`, `totalPredictions`, `predictionHits`, `predictionMisses`
- [x] `LVP` DebugFlag registered in SConscript
- [x] Binary verified: `--debug-help | grep LVP` returns `LVP: Writeback, LVP`
- [x] Phase 2 trace: `0 m5out/lvp_trace.out` (expected — LVP not wired in yet)
- [x] Branch: `research/p2-lvp-logic`

---

## [Phase 3] Pipeline Hook ✅ COMPLETE (May 1 2026)
**Branch:** `research/p3-pipeline-hook`

### What was implemented
- [x] `LoadValuePredictor* lvp` pointer added to `CPU` class (`cpu.hh`)
- [x] LVP instantiated in `CPU::CPU()` constructor (`cpu.cc`) using plain C++ constructor
- [x] `rename.hh` / `rename.cc`: `setLVP()` + `lookup()` called per load at Rename stage
  - On confident hit: injects predicted value into physical dest register, marks scoreboard ready
- [x] `iew.hh` / `iew.cc`: `setLVP()` + `update()` called after `ldstQueue.executeLoad()`
  - Reads `inst->memData` + `inst->effSize` for actual value
  - On misprediction: calls `squashDueToMemOrder()` to flush pipeline
- [x] SConscript: removed `SimObject('LoadValuePredictor.py', ...)` — LVP is now plain C++

### Verification Results
| Check | Result |
|---|---|
| `Hello world!` exits cleanly | ✅ tick 72052000 (same as baseline) |
| No infinite squash loop | ✅ No extra cycles |
| `wc -l m5out/lvp_trace.out` | ✅ **2130 lines** |
| Trace line 0 | ✅ `LoadValuePredictor 'system.cpu.lvp' created: numEntries=1024, confidenceThreshold=250` |
## Phase 3: Pipeline Surgery (✅ Completed)
### Goal:
Integrate the LVP into the gem5 O3 pipeline (`ArmO3CPU`), hooking into the Rename and IEW stages to enable speculative forwarding and squashing.

### Implementation:
- **LVP as Plain C++ Class**: Refactored the LVP to drop `SimObject` inheritance, sidestepping `Params` initialization issues at the CPU constructor level.
- **Rename Hook (`src/cpu/o3/rename.cc`)**: Added `lvp->lookup()`. On confident predictions, the value is written directly to the architectural destination register (`cpu->setReg`) and marked ready on the scoreboard, allowing dependent instructions to execute out-of-order immediately.
- **IEW Hook (`src/cpu/o3/iew.cc`)**: Moved `lvp->update()` into `writebackInsts()`, where the actual memory data is returned from the cache. On a misprediction, it triggers `squashDueToMemOrder(inst, tid)`, flushing the pipeline and redirecting fetch.

### Verification:
- Built `lvp_probe.c` and a corresponding gem5 test script (`run_lvp_probe.py`).
- **Proof of Forwarding**: Analyzed the native `Exec` trace and proved that the ALU executes dependent `add` instructions using the injected `0x2a` (42) value over 60,000 ticks *before* the misprediction squash occurs.
- **Proof of Squashing**: Verified 900+ squashes triggered by the intentional tamper load due to our IEW hook.
- **Status**: 100% Verified. The speculative window is successfully open.

## Phase 4: The Attack Gadget (⏳ Pending)

### Known Issues / Design Decisions
| Item | Decision | Reason |
|---|---|---|
| `LoadValuePredictor` removed from SimObject | Intentional | `SimObject(const Params&)` is the only constructor — no string shortcut exists. Plain C++ is sufficient for research. |
| LVP stats not in `stats.txt` | Known limitation | `nullptr` parent = legacy stats (exist internally but not in named hierarchy). Will fix in Phase 4 if stats are needed for evaluation. |
| `inst->memData` / `inst->effSize` | Correct field names | `getMemData()` / `getMemSize()` do not exist on `DynInst` — confirmed from `dyn_inst.hh` |
| capstone warning | Cosmetic | Library not installed; does not affect LVP or simulation |

### Build Errors Encountered & Fixes
| Error | Root Cause | Fix |
|---|---|---|
| `getMemData()` not found | Used non-existent getter | Changed to `inst->memData` (uint8_t*) + `memcpy` |
| `getMemSize()` not found | Used non-existent getter | Changed to `inst->effSize` |
| `SimObject(const string&)` not found | Only `SimObject(const Params&)` exists | Removed SimObject inheritance entirely |
| pybind11 static assertion failed | LVP in SConscript as SimObject but C++ class no longer inherits | Removed `SimObject('LoadValuePredictor.py', ...)` from SConscript |
| Binary missing execute bit | Ctrl+C after `[LINK]` — linker didn't finish chmod | `chmod +x build/ARM/gem5.opt` |

---

## [Phase 4] Attack Gadget — NEXT
- [ ] Write `flop_research/phase4/flop_attack.cpp`
  - Struct with `type` field (4-byte int)
  - Training loop: 250 iterations with `type=SAFE`
  - Trigger: switch to `type=MALICIOUS` + `dc ivac` cache flush
- [ ] Cross-compile for AArch64
- [ ] Run in gem5 with `--debug-flags=LVP,O3PipeView`
- [ ] Observe in Konata: speculative window before IEW squash

## [Phase 5] Managed Defense
- [ ] Sensitive-context tagging OR Speculation Depth Throttling
- [ ] Bypass LVP when tag active

## [Phase 6] Evaluation
- [ ] Baseline cycles (no LVP)
- [ ] Vulnerable cycles (LVP enabled, this branch)
- [ ] Secure cycles (defense active)
