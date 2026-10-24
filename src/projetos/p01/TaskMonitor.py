from m5.params import *
from m5.SimObject import SimObject


class TaskMonitor(SimObject):
    type = "TaskMonitor"
    cxx_header = "projetos/p01/task_monitor.hh"
    cxx_class = "gem5::TaskMonitor"

    cpus = VectorParam.BaseCPU("CPUs observadas, na ordem dos contextos")

    # Parametros do Histograma
    max_ticks = Param.Latency("10000", "Valor máximo de ticks do histograma")
    intervalo_faixas = Param.Unsigned(
        "1000", "Número do intervalo entre as faixas do histograma"
    )
