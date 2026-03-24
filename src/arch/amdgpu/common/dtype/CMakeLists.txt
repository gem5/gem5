# src/arch/amdgpu/common/dtype/CMakeLists.txt
# AMD GPU MXFP data types (header-only; test only)
#
# Test registration is in the parent src/arch/amdgpu/CMakeLists.txt
# (before the CONF_BUILD_GPU early return) so the safety check in
# Gem5Targets.cmake works correctly for non-GPU builds.
