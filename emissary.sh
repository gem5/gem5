/home/gem5/build/X86/gem5.opt \
--outdir=/home/gem5/Trace/emissary \
/home/gem5/configs/deprecated/example/se.py \
--cmd=/home/gem5/benchmarks/emissary_effect_test \
--cpu-type=DerivO3CPU \
--caches --l2cache \
--l2_rp=LRUEmissary \
--lru_ways=2 --preserve_ways=6 \
--emissary-enable \
--starveAtleast=2 --starveRandomness=100 \