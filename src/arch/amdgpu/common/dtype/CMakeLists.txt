# src/arch/amdgpu/common/dtype/CMakeLists.txt
# AMD GPU MXFP data types (header-only; test only)

# --- Unit tests ---
gem5_add_test_source(mxfp.test.cc CONDITION CONF_BUILD_GPU)
