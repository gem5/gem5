#! /bin/sh
build/ALPHA_MESI_Two_Level/gem5.opt configs/numa/ccnuma_se.py --l1i_size='32kB' --l1d_size='64kB' --l2_size='1MB' --num_cpus_per_domain=2 --num_domains=2 --mem_size_per_domain='512MB'