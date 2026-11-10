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
parser.add_argument('--latency-mode',choices=('service','demand'),default='service')
parser.add_argument('--margin-cycles',type=int,default=None,
                    help='Override the team profile margin; omitted preserves its tuned value')
parser.add_argument('--dram-timing-scale',type=float,default=1,
                    help='Synthetic DRAM timing sensitivity; 1 preserves the frozen model')
parser.add_argument('--args',nargs='*',default=[])
a=parser.parse_args()
if (a.margin_cycles is not None and a.margin_cycles < 0) or not 0 < a.dram_timing_scale <= 4:
    parser.error('margin must be nonnegative; DRAM timing scale must be in (0,4]')
folder=Path(m5.options.outdir).resolve()
spec=dict(prefetcher=a.profile,binary=str(Path(a.binary).resolve()),args=a.args,
          cwd=str(folder),outdir=str(folder))
system=make_system(spec)
controller=system.cpu.dcache.prefetcher
if a.profile=='tpc-t':
    for component in controller.components:
        component.trace=a.trace
        if a.margin_cycles is not None:
            component.margin_cycles=a.margin_cycles
        component.latency_mode=a.latency_mode
if a.dram_timing_scale != 1:
    from m5.params import Latency
    # Synthetic sensitivity only: scale concrete latency values, preserving proxies.
    # This also scales DRAM clock/burst/refresh timing, not the caches or buses.
    for mem in system.mem_ctrls:
        for name in mem.dram._params.keys():
            value=getattr(mem.dram,name)
            if isinstance(value,Latency):
                setattr(mem.dram,name,str(float(value)*a.dram_timing_scale)+'s')
root=Root(full_system=False,system=system)
m5.instantiate()
event=m5.simulate(2_000_000_000)
if event.getCause()!='exiting with last active thread context' or event.getCode()!=0:
    raise RuntimeError((event.getCause(),event.getCode()))
print('TPC_FUNCTIONAL_PASS',a.profile,'tick',m5.curTick(),flush=True)
