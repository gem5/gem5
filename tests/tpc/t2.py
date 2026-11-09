"""Focused T2 functional run using the frozen team topology (no checkpoint)."""
import argparse
from pathlib import Path
import sys
import m5
from m5.objects import Root
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'configs'))
from system import make_system
parser=argparse.ArgumentParser()
parser.add_argument('--binary',required=True)
parser.add_argument('--profile',choices=('tpc-t','tpc-c','no-prefetch'),default='tpc-t')
parser.add_argument('--trace',action='store_true')
parser.add_argument('--args',nargs='*',default=[])
a=parser.parse_args()
folder=Path(m5.options.outdir).resolve()
spec=dict(prefetcher=a.profile,binary=str(Path(a.binary).resolve()),args=a.args,
          cwd=str(folder),outdir=str(folder))
system=make_system(spec)
controller=system.cpu.dcache.prefetcher
if a.profile=='tpc-t':
    for component in controller.components: component.trace=a.trace
root=Root(full_system=False,system=system)
m5.instantiate()
event=m5.simulate(2_000_000_000)
if event.getCause()!='exiting with last active thread context' or event.getCode()!=0:
    raise RuntimeError((event.getCause(),event.getCode()))
print('TPC_FUNCTIONAL_PASS',a.profile,'tick',m5.curTick(),flush=True)
