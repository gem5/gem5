#!/bin/bash
# SCons-to-CMake migration audit script
# Checks for residual SCons-era references in active files.
#
# Usage: bash util/audit/scons_to_cmake_audit.sh [output-dir]
# Default output: temp/audit-scons-to-cmake/<date>/

set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

DATE=$(date -u '+%Y-%m-%d')
OUTDIR="${1:-temp/audit-scons-to-cmake/$DATE}"
mkdir -p "$OUTDIR"
FAIL=0

# Common exclusion globs for deprecated/generated/temp/vendored files
EXCLUDE=(
    --hidden
    --glob '!RELEASE-NOTES.md'
    --glob '!configs/deprecated/**'
    --glob '!tests/deprecated/**'
    --glob '!.humanize/**'
    --glob '!.git/**'
    --glob '!build*/**'
    --glob '!temp/**'
    --glob '!cmake/**'
    --glob '!*.pyc'
    --glob '!util/audit/**'
    --glob '!ext/pybind11/**'
    --glob '!ext/googletest/**'
)

echo "=== SCons-to-CMake Migration Audit ==="
echo "Date: $(date -u '+%Y-%m-%dT%H:%M:%SZ')"
echo "Output directory: $OUTDIR"
echo ""

# L1: gem5.opt/debug/fast in active files (including .github/workflows)
echo "--- L1: gem5.opt/debug/fast references ---"
if rg -n 'gem5\.(opt|debug|fast)\b' \
    "${EXCLUDE[@]}" \
    --glob '!src/mem/ruby/protocol/**' \
    --glob '!ext/systemc/**/SConscript' \
    --glob '!util/m5/**/SConscript*' \
    -- . 2>/dev/null | tee "$OUTDIR/l1-gem5-opt.txt" | grep -q .; then
    echo "WARNING: Active gem5.opt/debug/fast references found (see $OUTDIR/l1-gem5-opt.txt)"
    FAIL=1
else
    echo "PASS: No active gem5.opt/debug/fast references"
fi
echo ""

# L2: libgem5_opt/debug/fast references
echo "--- L2: libgem5_opt/debug/fast references ---"
if rg -n 'libgem5_(opt|debug|fast)' \
    "${EXCLUDE[@]}" \
    -- . 2>/dev/null | tee "$OUTDIR/l2-libgem5.txt" | grep -q .; then
    echo "WARNING: Active libgem5_opt/debug/fast references found"
    FAIL=1
else
    echo "PASS: No active libgem5_opt references"
fi
echo ""

# L3: scons build commands in active docs (including hidden paths)
echo "--- L3: SCons build commands in active docs ---"
if rg -n '\bscons\s+build/' \
    "${EXCLUDE[@]}" \
    --glob '!SConscript*' \
    --glob '!SConstruct*' \
    -- . 2>/dev/null | tee "$OUTDIR/l3-scons-build.txt" | grep -q .; then
    echo "WARNING: Active scons build commands found"
    FAIL=1
else
    echo "PASS: No active scons build commands"
fi
echo ""

# L4: .opt/.fast in CI matrix (compiler-tests)
echo "--- L4: .opt/.fast in compiler-tests.yaml matrix ---"
if rg -n '\.opt|\.fast' .github/workflows/compiler-tests.yaml 2>/dev/null | tee "$OUTDIR/l4-compiler-matrix.txt" | grep -q .; then
    echo "WARNING: .opt/.fast found in compiler-tests.yaml"
    FAIL=1
else
    echo "PASS: No .opt/.fast in compiler-tests.yaml"
fi
echo ""

# L5: build/gem5 without variant directory in active docs (including yaml/workflow files)
echo "--- L5: build/gem5 without variant (should be build/<VARIANT>/gem5) ---"
if rg -n '(^|[^/])build/gem5\b' \
    "${EXCLUDE[@]}" \
    --glob '!CMakeLists.txt' \
    --glob '!**/CMakeLists.txt' \
    --glob '!*.cmake' \
    -- . 2>/dev/null | tee "$OUTDIR/l5-build-gem5-no-variant.txt" | grep -q .; then
    echo "WARNING: build/gem5 without variant found (should be build/<VARIANT>/gem5)"
    FAIL=1
else
    echo "PASS: No build/gem5 without variant"
fi
echo ""

# L6: SConscript references in active documentation (not in actual SConscript files)
# Note: References that document SConscript files as "historical artifacts" are expected
echo "--- L6: SConscript references in documentation ---"
if rg -n 'SConscript' \
    "${EXCLUDE[@]}" \
    --glob '!SConscript*' \
    --glob '!**/SConscript*' \
    --glob '!*.py' \
    --type md \
    -- . 2>/dev/null | grep -v 'historical' | tee "$OUTDIR/l6-sconscript-doc-refs.txt" | grep -q .; then
    echo "WARNING: SConscript references in documentation"
    FAIL=1
else
    echo "PASS: No SConscript references in documentation (or only historical artifact notices)"
fi
echo ""

# L7: -B build without variant (catches all cmake ... -B build forms regardless of flag order)
echo "--- L7: -B build without variant ---"
if rg -n '\-B\s+build\b' \
    "${EXCLUDE[@]}" \
    --glob '!CMakeLists.txt' \
    --glob '!**/CMakeLists.txt' \
    --glob '!*.cmake' \
    -- . 2>/dev/null | grep -v 'build/' | grep -v 'build_' | tee "$OUTDIR/l7-cmake-no-variant.txt" | grep -q .; then
    echo "WARNING: -B build without variant found (should be -B build/<VARIANT> or -B build_cxx)"
    FAIL=1
else
    echo "PASS: No -B build without variant"
fi
echo ""

# L8: ninja -C build without variant
echo "--- L8: ninja -C build without variant ---"
if rg -n 'ninja\s+-C\s+build\b' \
    "${EXCLUDE[@]}" \
    -- . 2>/dev/null | grep -v 'build/' | grep -v 'build_' | tee "$OUTDIR/l8-ninja-no-variant.txt" | grep -q .; then
    echo "WARNING: ninja -C build without variant found"
    FAIL=1
else
    echo "PASS: No ninja -C build without variant"
fi
echo ""

echo "=== Audit Complete ==="
if [ $FAIL -eq 0 ]; then
    echo "RESULT: ALL CHECKS PASSED"
else
    echo "RESULT: SOME CHECKS HAVE WARNINGS (review output in $OUTDIR)"
fi
exit $FAIL
