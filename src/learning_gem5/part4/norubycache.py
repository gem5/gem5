import m5
from m5.objects import Root, System, GarnetNetwork, TrafficGen, ClockDomain, SrcClockDomain
from m5.objects import RubyNetwork


# 创建系统顶层
system = System()

# 设置时钟和电源域
system.clk_domain = SrcClockDomain(clock="1GHz", voltage_domain=m5.objects.VoltageDomain())

# 创建 Garnet 网络
system.network = RubyNetwork(
    topology="Mesh_XY",   # 经典mesh拓扑
)
system.network.data_msg_size = 10

# 创建 TrafficGen 端点
system.trafficgens = [TrafficGen(), TrafficGen()]


# 将端点连接到网络
# 通常情况下你需要手动连接端口，可以根据实际网络端口进行调整:
# system.network.connectPorts(system.trafficgens[0].port, 0)  # 连接到0号router
# system.network.connectPorts(system.trafficgens[1].port, 3)  # 连接到3号router

# 设置仿真根
root = Root(full_system=False, system=system)
root.system.mem_mode = "timing"

# 启动仿真
m5.instantiate()
print("Beginning simulation!")
exit_event = m5.simulate()
print('Exiting @ tick {} because {}'.format(m5.curTick(), exit_event.getCause()))