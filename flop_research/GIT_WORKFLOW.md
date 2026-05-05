# FLOP Research: Git Checkpoint Strategy

Because we are doing "surgery" on the core gem5 C++ pipeline (`rename.cc`, `iew.cc`), we must have a way to undo our changes instantly if the C++ compiler starts throwing errors or the simulator crashes.

We use a **Hierarchical Prefix system** to keep research grouped together and strictly separated by phase logic.

## 1. Branching Strategy
We leave the `stable` branch completely alone. That is our pristine, original gem5 code.
We create a new branch for each phase, naming it `research/p[phase#]-[task-name]`.

**Examples**:
- `research/p1-o3-setup` (Current)
- `research/p2-lvp-logic`
- `research/p3-pipeline-hook`
- `research/p4-attack-gadget`
- `research/integration` (The unified working branch where completed phases merge)

**The Workflow**:
1. Start on the clean `stable` branch.
2. Create a phase-specific branch (e.g., `git checkout -b research/p1-o3-setup`).
3. Once the task works perfectly, merge it into `research/integration` and start the next branch for Phase 2.

## 2. GitHub Connectivity
GitHub serves as a critical off-site backup for WSL environments and provides a clean commit history for NITC evaluation.

**Setup Command**:
```bash
git remote add origin https://github.com/yourname/gem5-flop-research.git
git push -u origin research/p1-o3-setup
```

## 3. How to Undo Mistakes ("Going Back")

### Scenario A: "I haven't committed yet, but I broke my code"
```bash
git restore src/cpu/o3/rename.cc
```
*This permanently wipes your current unsaved changes and reverts to the last checkpoint.*

### Scenario B: "I committed something broken and want to go back in time"
To view your timeline of save states:
```bash
git log --oneline
```
To hard reset your entire pipeline back to the last good commit (Warning: deletes newer commits):
```bash
git reset --hard HEAD~1
```

---
*Note: Your `STATUS.md` tracks our roadmap position in the root folder!*
