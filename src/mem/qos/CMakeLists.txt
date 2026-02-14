# src/mem/qos/CMakeLists.txt
# Converted from src/mem/qos/SConscript

gem5_add_source(mem_ctrl.cc)
gem5_add_source(mem_sink.cc)
gem5_add_source(policy.cc)
gem5_add_source(policy_fixed_prio.cc)
gem5_add_source(policy_pf.cc)
gem5_add_source(q_policy.cc)
gem5_add_source(turnaround_policy_ideal.cc)

# SimObjects
gem5_add_simobject(QoSMemCtrl.py SIM_OBJECTS QoSMemCtrl ENUMS QoSQPolicy)
gem5_add_simobject(QoSMemSinkCtrl.py SIM_OBJECTS QoSMemSinkCtrl)
gem5_add_simobject(QoSMemSinkInterface.py SIM_OBJECTS QoSMemSinkInterface)
gem5_add_simobject(QoSPolicy.py
    SIM_OBJECTS QoSPolicy QoSFixedPriorityPolicy QoSPropFairPolicy)
gem5_add_simobject(QoSTurnaround.py
    SIM_OBJECTS QoSTurnaroundPolicy QoSTurnaroundPolicyIdeal)
