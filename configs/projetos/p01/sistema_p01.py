"""
Sistema do Projeto 1: x86 em modo SE com N CPUs, L1 privadas, L2 compartilhada
e o TaskMonitor.

Uma execução do gem5 = um comando:

    gem5.opt -d saida sistema_p01.py --cores 4 --policy ws --seed 1 -- ./uts 1000

O que vem depois do "--" é o programa e seus parâmetros de tamanho. O script
monta a linha de comando COMPLETA do programa simulado:

    <programa> <cores> <estatica|roubo> <seed> <tamanho...>

Assim, o número de CPUs criadas e o número de trabalhadores pedido ao programa
vêm da mesma opção (--cores): se fossem diferentes, a criação de threads
falharia no simulador.
"""
import argparse
import sys

import m5
from m5.objects import *

ap = argparse.ArgumentParser()
ap.add_argument("--cores", type=int, default=4)
ap.add_argument("--policy", choices=["static", "ws"], default="ws")
ap.add_argument("--cpu", choices=["timing", "o3"], default="timing")
ap.add_argument("--seed", type=int, default=1)
ap.add_argument("--no-monitor", action="store_true",
                help="Executa sem TaskMonitor (linha de base)")
ap.add_argument("--max-ticks", type=int, default=0,
                help="Aborta a simulação neste tick (0 = sem limite); serve "
                     "para diagnosticar execuções que não terminam")
ap.add_argument("cmd", nargs="+",
                help="programa seguido dos seus parâmetros de tamanho")
args = ap.parse_args()

# Dependendo da versão do Python, o "--" pode ou não ser mantido em `cmd`.
cmd = [a for a in args.cmd if a != "--"]
programa, extras = cmd[0], cmd[1:]

# Nomes das políticas como o PROGRAMA as espera.
POLITICA_DO_PROGRAMA = {"static": "estatica", "ws": "roubo"}

CPU = X86TimingSimpleCPU if args.cpu == "timing" else X86O3CPU


# Caches
class L1(Cache):
    tag_latency = 2
    data_latency = 2
    response_latency = 2
    mshrs = 8
    tgts_per_mshr = 16


class L2(Cache):
    size = "256KiB"
    assoc = 8
    tag_latency = 10
    data_latency = 10
    response_latency = 10
    mshrs = 16
    tgts_per_mshr = 16


# Sistema base e relógio
system = System()
system.clk_domain = SrcClockDomain(clock="2GHz",
                                   voltage_domain=VoltageDomain())
system.mem_mode = "timing"
system.mem_ranges = [AddrRange("2GiB")]
system.cache_line_size = 64

# Barramentos (criados antes das CPUs, que se ligam a eles)
system.l2bus = L2XBar()
system.membus = SystemXBar()
system.system_port = system.membus.cpu_side_ports

# L2 compartilhada entre todas as CPUs
system.l2 = L2()
system.l2.cpu_side = system.l2bus.mem_side_ports
system.l2.mem_side = system.membus.cpu_side_ports

# `args.cores` CPUs, cada uma com L1I e L1D privadas. (O rascunho criava UMA
# CPU só e ignorava --cores e --cpu.)
system.cpu = [CPU(cpu_id=i) for i in range(args.cores)]
for cpu in system.cpu:
    cpu.icache = L1(size="32KiB", assoc=8)
    cpu.dcache = L1(size="32KiB", assoc=8)
    cpu.icache.cpu_side = cpu.icache_port
    cpu.dcache.cpu_side = cpu.dcache_port
    cpu.icache.mem_side = system.l2bus.cpu_side_ports
    cpu.dcache.mem_side = system.l2bus.cpu_side_ports

    cpu.createInterruptController()          # exigido pelo x86
    cpu.interrupts[0].pio = system.membus.mem_side_ports
    cpu.interrupts[0].int_requestor = system.membus.cpu_side_ports
    cpu.interrupts[0].int_responder = system.membus.mem_side_ports

system.mem_ctrl = MemCtrl(dram=DDR4_2400_8x8(range=system.mem_ranges[0]))
system.mem_ctrl.port = system.membus.mem_side_ports

# Processo: o mesmo para todas as CPUs, para que os contextos extras fiquem
# disponíveis para o clone das threads do programa. (O rascunho usava `cmd`,
# que não existia, e passava a lista inteira como `executable`.)
argv = [programa, str(args.cores), POLITICA_DO_PROGRAMA[args.policy],
        str(args.seed)] + extras
system.workload = SEWorkload.init_compatible(programa)
process = Process(cmd=argv, executable=programa)
for cpu in system.cpu:
    cpu.workload = process
    cpu.createThreads()

if not args.no_monitor:
    system.monitor = TaskMonitor(cpus=system.cpu)

root = Root(full_system=False, system=system)

m5.instantiate()
if args.max_ticks:
    evento = m5.simulate(args.max_ticks)
else:
    evento = m5.simulate()
causa = evento.getCause()
print(f"Fim @ tick {m5.curTick()}: {causa}")

# Se parou por causa do limite de ticks, o programa NÃO terminou: sinaliza ao
# script de campanha com um código de saída diferente de zero.
if "limit reached" in causa:
    sys.exit(2)