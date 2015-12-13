import m5
from m5.objects import *

m5.util.addToPath('../common')

binary_dir = '/home/itecgo/Tools/GEM5/cpu2006/'
data_dir = binary_dir

# 437.leslie3d
leslie3d = LiveProcess()
leslie3d.executable = binary_dir + '437.leslie3d/run/run_base_ref_amd64-m64-gcc42-nn.0000/leslie3d_base.amd64-m64-gcc42-nn'
stdin = data_dir + '437.leslie3d/run/run_base_ref_amd64-m64-gcc42-nn.0000/leslie3d.in'
leslie3d.cmd = [leslie3d.executable]
leslie3d.input = stdin
leslie3d.output = 'leslie3d.stdout'
