#!/bin/bash
set -e
TYPE=$1; N=$2; NCPUS=${3:-1}
case "$TYPE" in
  atomic) CPU=AtomicSimpleCPU; TARGET=100000 ;;
  timing) CPU=TimingSimpleCPU; TARGET=20000 ;;
  o3)     CPU=O3CPU;           TARGET=10000 ;;
  *) echo "type = atomic|timing|o3"; exit 1 ;;
esac
if [ "$N" -le 1 ]; then LAPS=$TARGET; else LAPS=$((TARGET / N)); fi
OUT=m5out_${TYPE}_c${NCPUS}_t${N}
cat > sweep.rcS <<RCS
#!/bin/sh
cat /proc/ctx_switch_stats > /tmp/cb.txt
m5 resetstats
/root/ctxbench $N $LAPS
echo "BENCH_EXIT=\$?"
m5 dumpstats
cat /proc/ctx_switch_stats > /tmp/ca.txt
m5 writefile /tmp/cb.txt ctx_before.txt
m5 writefile /tmp/ca.txt ctx_after.txt
sync
m5 exit
RCS
echo ">>> $TYPE  ncpus=$NCPUS  threads=$N  laps=$LAPS  -> $OUT"
./build/ARM/gem5.opt --outdir=$OUT configs/example/fs.py \
  --kernel=/home/zag-20/context_switch/linux-5.4.49-instrumented/vmlinux \
  --disk-image=/home/zag-20/back/arm64-dev.img \
  --machine-type=VExpress_GEM5_V2 --cpu-type=$CPU \
  --caches --l2cache --num-cpus=$NCPUS --mem-size=8GB \
  --script=sweep.rcS \
  --command-line="earlyprintk=pl011,0x1c090000 console=ttyAMA0 lpj=19988480 norandmaps rw loglevel=8 mem=8GB root=/dev/sda1"
echo ">>> parse:"
python3 ctx_parse.py $OUT/stats.txt
