import m5
from m5.objects import *

root = Root(full_system=False)
root.hello = MyHelloObject(time_to_wait='2us', number_of_fires=10)

m5.instantiate()

print("begin")
exit_event = m5.simulate()
print('exit @ tick {} beacuase {}'
        .format(m5.curTick(), exit_event.getCause()))