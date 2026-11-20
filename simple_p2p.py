#!/usr/bin/env python3

import torch


def run_p2p_test(N):
    print("Initializing PyTorch...")
    if not torch.cuda.is_available():
        print("Error: CUDA/ROCm not available.")
        return

    gpu_count = torch.cuda.device_count()
    print(f"Detected {gpu_count} GPUs.")

    if gpu_count < 2:
        print("Error: Need at least 2 GPUs for Peer-to-Peer test.")
        return

    dev0 = torch.device("cuda:0")
    dev1 = torch.device("cuda:1")
    cpu = torch.device("cpu")

    # 1. CPU -> GPU0
    print(
        f"1. CPU -> GPU0: Allocating {N} floats on CPU and copying to {dev0}..."
    )
    t_cpu = torch.full((N,), 1.0, device=cpu)
    t0 = t_cpu.to(dev0)  # Triggers Host -> Device copy
    torch.cuda.synchronize(dev0)

    # 2. GPU0 -> GPU1 (P2P)
    print(f"2. GPU0 -> GPU1: Copying data from {dev0} to {dev1}...")
    t1 = t0.to(dev1)  # Triggers Device -> Device (XGMI) copy
    torch.cuda.synchronize(dev1)

    # 3. Compute on GPU1
    print(f"3. GPU1: Performing calculation (adding 1.0 to each element)...")
    # This forces the CU to read the data that was just written by SDMA
    t_result = t1 + 1.0
    torch.cuda.synchronize(dev1)

    # 4. GPU1 -> CPU
    print(f"4. GPU1 -> CPU: Copying result from {dev1} back to CPU...")
    t_final_cpu = t_result.to(cpu)  # Triggers Device -> Host copy

    print("Verifying data...")
    result = t_final_cpu.sum().item()
    return result


test_num = 50
run_testing = run_p2p_test(test_num)
expected_val = float(test_num) * 2.0
if abs(run_testing - expected_val) < 1e-5:
    print("SUCCESS: Data transferred and calculated correctly!")
else:
    print(
        f"FAILURE: Data mismatch. Expected {expected_val}, got {run_testing}"
    )
