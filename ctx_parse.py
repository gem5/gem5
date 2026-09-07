#!/usr/bin/env python3
import re, sys, os

def first_block(path):
    cur, indump = [], False
    with open(path) as f:
        for line in f:
            if 'Begin Simulation Statistics' in line: indump, cur = True, []; continue
            if 'End Simulation Statistics' in line and indump: return cur
            if indump: cur.append(line)
    return cur

def last_block(path):
    blocks = []
    cur, indump = [], False
    with open(path) as f:
        for line in f:
            if 'Begin Simulation Statistics' in line:
                indump, cur = True, []
                continue
            if 'End Simulation Statistics' in line and indump:
                blocks.append(cur)
                indump = False
            if indump:
                cur.append(line)
    return blocks[-1] if blocks else []

def parse_stats(lines):
    s = {}
    for line in lines:
        line = line.split('#')[0].strip()
        if not line: continue
        p = line.split()
        if len(p) >= 2:
            try: s[p[0]] = float(p[1])
            except ValueError: pass
    return s

def find(s, *pats):
    for pat in pats:
        rx = re.compile(pat)
        for k, v in s.items():
            if rx.search(k): return k, v
    return None, None

def parse_ctx(path):
    d = {'total': None, 'stages': {}, 'access': {}, 'miss': {}, 'insts': 0}
    if not os.path.exists(path): return d
    for line in open(path):
        m = re.search(r'TOTAL_CONTEXT_SWITCHES:\s*(\d+)', line)
        if m: d['total'] = int(m.group(1)); continue
        if line.lstrip().startswith('misses'):
            d['miss'] = {k: int(v) for k, v in re.findall(r'(\w+)=(\d+)', line)}; continue
        if line.lstrip().startswith('access'):
            d['access'] = {k: int(v) for k, v in re.findall(r'(\w+)=(\d+)', line)}; continue
        m2 = re.match(r'\s*insts\s+(\d+)', line)
        if m2: d['insts'] = int(m2.group(1)); continue
        m3 = re.match(r'\s*(\w+)\s+cnt=(\d+)', line)
        if m3:
            name, cnt = m3.group(1), int(m3.group(2))
            a = re.search(r'avg_ns=(\d+)', line); t = re.search(r'total_ns=(\d+)', line)
            a = int(a.group(1)) if a else 0
            d['stages'][name] = {'cnt': cnt, 'total_ns': int(t.group(1)) if t else a * cnt}
    return d

path = sys.argv[1] if len(sys.argv) > 1 else 'm5out_memproc_atomic/stats.txt'
outdir = os.path.dirname(path) or '.'

# [اصلاح اصلی]: خواندن آخرین بلاک جم۵ که بعد از m5 resetstats ثبت شده است
s = parse_stats(last_block(path))
if not s: print("no stats block in", path); sys.exit()

ik, insts = find(s, r'^simInsts$', r'committedInsts::total$', r'committedInsts$')
if insts is None: insts = 1.0 # جلوگیری از ZeroDivisionError در صورت نبود فیلد
print(f"instructions ({ik}) = {insts:,.0f}")

cb = parse_ctx(os.path.join(outdir, 'ctx_before.txt'))
ca = parse_ctx(os.path.join(outdir, 'ctx_after.txt'))

if cb['total'] is not None and ca['total'] is not None:
    print(f"context switches (window) = {ca['total']-cb['total']:,}   [{cb['total']:,} -> {ca['total']:,}]")
    parts = []
    for st in ['switch_mm', 'switch_to', 'finish', 'prepare']:
        if st in ca['stages'] and st in cb['stages']:
            dc = ca['stages'][st]['cnt'] - cb['stages'][st]['cnt']
            dt = ca['stages'][st]['total_ns'] - cb['stages'][st]['total_ns']
            if dc > 0: parts.append(f"{st}={dt/dc:.0f}")
    if parts: print("per-switch avg_ns: " + "  ".join(parts))

if ca.get('access') and cb.get('access'):
    di = ca['insts'] - cb['insts']
    print(f"\n==== context-switch window ====")
    print(f"{'level':6}{'access':>16}{'miss':>14}{'miss_rate%':>13}{'MPKI':>10}")
    for k, label in {'l1d': 'L1-D', 'l1i': 'L1-I', 'l2': 'L2', 'dtlb': 'dTLB', 'itlb': 'iTLB'}.items():
        a = ca['access'].get(k, 0) - cb['access'].get(k, 0)
        mi = ca['miss'].get(k, 0) - cb['miss'].get(k, 0)
        rate = 100 * mi / a if a else 0
        mpki = 1000 * mi / di if di else 0
        print(f"{label:6}{a:16,.0f}{mi:14,.0f}{rate:13.3f}{mpki:10.3f}")

l1_acc = l1_mis = 0
print(f"\n==== Total workload (gem5 stats) ====")
print(f"{'level':6}{'access':>14}{'miss':>12}{'miss_rate%':>13}{'MPKI':>10}")
for name, (ap, mp) in {
    'L1-D': (r'dcache\.overallAccesses::total$', r'dcache\.overallMisses::total$'),
    'L1-I': (r'icache\.overallAccesses::total$', r'icache\.overallMisses::total$'),
    'L2':   (r'l2.*\.overallAccesses::total$',   r'l2.*\.overallMisses::total$'),
}.items():
    _, acc = find(s, ap); _, mis = find(s, mp)
    if acc is None or mis is None: print(f"{name:6}{'keys not found':>26}"); continue
    print(f"{name:6}{acc:14,.0f}{mis:12,.0f}{(100*mis/acc if acc else 0):13.3f}{(1000*mis/insts if insts else 0):10.3f}")
    if name in ('L1-D', 'L1-I'): l1_acc += acc; l1_mis += mis

# [اصلاح پترن‌ها]: سازگار با ساختارهای استاندارد mmu و dtb/itb در معماری‌های آرم
_, dra = find(s, r'mmu\..*tb\..*Accesses', r'dtb\.rdAccesses', r'dtb\.readAccesses')
_, drm = find(s, r'mmu\..*tb\..*Misses', r'dtb\.rdMisses', r'dtb\.readMisses')
if dra is not None and drm is not None:
    print(f"{'dTLB':6}{dra:14,.0f}{drm:12,.0f}{(100*drm/dra if dra else 0):13.3f}{(1000*drm/insts if insts else 0):10.3f}")

_, ia = find(s, r'mmu\..*tb\..*Accesses', r'itb\.instAccesses', r'itb\.accesses')
_, im = find(s, r'mmu\..*tb\..*Misses', r'itb\.instMisses', r'itb\.misses')
if ia is not None and im is not None:
    print(f"{'iTLB':6}{ia:14,.0f}{im:12,.0f}{(100*im/ia if ia else 0):13.3f}{(1000*im/insts if insts else 0):10.3f}")

if l1_acc:
    print(f"\n==== Total workload (L1-D + L1-I) ====")
    print(f"total Access = {l1_acc:,.0f}")
    print(f"total miss   = {l1_mis:,.0f}")
    print(f"miss rate    = {100*l1_mis/l1_acc:.3f} %")
    print(f"MPKI         = {1000*l1_mis/insts:.3f}")
