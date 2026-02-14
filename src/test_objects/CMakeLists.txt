# src/test_objects/CMakeLists.txt
# Converted from src/test_objects/SConscript
# Conditional on CONF_USE_TEST_OBJECTS (guarded by parent)

gem5_add_source(stat_tester.cc)

gem5_add_simobject(StatTester.py
    SIM_OBJECTS
        StatTester ScalarStatTester VectorStatTester
        Vector2dStatTester SparseHistStatTester)
