"""Run after TPC_TESTS=1 JOBS=2 ./build-gem5-fast.sh. No guest execution."""
import json
from pathlib import Path
import sys
import m5
from m5.objects import Root, TPCStateFixture, TPCFixtureComponent, FrameworkQueue, DivisionOfLaborController
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'configs'))
import prefetchers
from system import make_system
policy = '--policy' in sys.argv
if policy:
    def factory():
        routes = [FrameworkQueue(accounting_target=k) for k in (1,2,3)]
        # Deliberately reverse order; T recognizes but generates no candidates.
        components = [TPCFixtureComponent(accounting_source=k) for k in (3,2,1)]
        layers = prefetchers.no_prefetch()
        layers.update(l1d=DivisionOfLaborController(mode='TPC',routes=routes,components=components),
                      l2=routes[1],l3=routes[2])
        return layers
    prefetchers.PROFILES['fixture'] = factory
folder=Path(m5.options.outdir).resolve()
spec=dict(prefetcher='fixture' if policy else 'tpc-c', binary=str(ROOT/'results/tpc-test-program'),
          args=[],cwd=str(folder),outdir=str(folder))
system=make_system(spec)
controller=system.cpu.dcache.prefetcher
component=next(c for c in controller.components if int(c.accounting_source)==(1 if policy else 3))
system.fixture=TPCStateFixture(cache=system.cpu.icache,controller=controller,component=component)
root=Root(full_system=False,system=system)
m5.instantiate()
# LRU uses tick 0 for invalid entries. Avoid an artificial all-zero tie.
m5.simulate(1)
system.fixture.run()
print('TPC_LIFECYCLE_PASS',flush=True)
