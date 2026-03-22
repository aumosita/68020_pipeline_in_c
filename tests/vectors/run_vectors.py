#!/usr/bin/env python3
"""
run_vectors.py — Run SingleStepTests 680x0 test vectors against our emulator.

Usage:
    python3 run_vectors.py [test_file.json.gz ...]

If no files specified, runs all .json.gz files in the same directory.

Each test vector contains:
  - initial: register state + RAM contents before instruction
  - final: expected register state + RAM contents after instruction

The script spawns ./test_vector_runner for each batch of tests.
"""

import json
import gzip
import subprocess
import sys
import os
import struct

RUNNER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "test_vector_runner")

def run_test_batch(tests, filename):
    """Run a batch of tests and return (pass_count, fail_count, errors)."""
    passed = 0
    failed = 0
    errors = []

    for i, test in enumerate(tests):
        ini = test["initial"]
        fin = test["final"]

        # Build input line: hex-encoded initial state
        # Format: pc sr d0..d7 a0..a6 usp ssp ram_pairs... | expected_final
        # RAM: addr:val addr:val ...

        # Build RAM: include prefetch words at PC address as instruction bytes
        extra_ram = []
        prefetch = ini.get("prefetch", [])
        pc = ini["pc"]
        for j, word in enumerate(prefetch):
            addr = pc + j * 2
            extra_ram.append((addr, (word >> 8) & 0xFF))
            extra_ram.append((addr + 1, word & 0xFF))

        all_ram_init = extra_ram + ini.get("ram", [])
        ram_init = " ".join(f"{a}:{v}" for a, v in all_ram_init)
        ram_final = " ".join(f"{a}:{v}" for a, v in fin.get("ram", []))

        # Build stdin line for the C runner
        line = (
            f"{ini['pc']} {ini['sr']} "
            f"{ini['d0']} {ini['d1']} {ini['d2']} {ini['d3']} "
            f"{ini['d4']} {ini['d5']} {ini['d6']} {ini['d7']} "
            f"{ini['a0']} {ini['a1']} {ini['a2']} {ini['a3']} "
            f"{ini['a4']} {ini['a5']} {ini['a6']} "
            f"{ini['usp']} {ini['ssp']} "
            f"{len(all_ram_init)} {ram_init} "
            f"| "
            f"{fin['pc']} {fin['sr']} "
            f"{fin['d0']} {fin['d1']} {fin['d2']} {fin['d3']} "
            f"{fin['d4']} {fin['d5']} {fin['d6']} {fin['d7']} "
            f"{fin['a0']} {fin['a1']} {fin['a2']} {fin['a3']} "
            f"{fin['a4']} {fin['a5']} {fin['a6']} "
            f"{fin['usp']} {fin['ssp']} "
            f"{len(fin.get('ram', []))} {ram_final}"
        )

        try:
            result = subprocess.run(
                [RUNNER],
                input=line,
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                passed += 1
            else:
                failed += 1
                if failed <= 10:  # only show first 10 failures
                    errors.append(f"  [{i}] {test['name']}: {result.stdout.strip()}")
        except subprocess.TimeoutExpired:
            failed += 1
            if failed <= 10:
                errors.append(f"  [{i}] {test['name']}: TIMEOUT")

    return passed, failed, errors


def main():
    vec_dir = os.path.dirname(os.path.abspath(__file__))

    if len(sys.argv) > 1:
        files = sys.argv[1:]
    else:
        files = sorted([
            os.path.join(vec_dir, f)
            for f in os.listdir(vec_dir)
            if f.endswith(".json.gz") and os.path.getsize(os.path.join(vec_dir, f)) > 100
        ])

    if not os.path.exists(RUNNER):
        print(f"ERROR: Runner not found at {RUNNER}")
        print("Build it with: make test_vector_runner")
        sys.exit(1)

    total_pass = 0
    total_fail = 0

    for filepath in files:
        name = os.path.basename(filepath)
        print(f"\n{'='*60}")
        print(f"Testing: {name}")

        with gzip.open(filepath, 'rt') as f:
            tests = json.load(f)

        # Run first 100 tests per file for quick validation
        batch = tests[:100]
        p, f_count, errs = run_test_batch(batch, name)
        total_pass += p
        total_fail += f_count

        print(f"  {p}/{len(batch)} passed", end="")
        if f_count:
            print(f" ({f_count} FAILED)")
            for e in errs:
                print(e)
        else:
            print()

    print(f"\n{'='*60}")
    print(f"TOTAL: {total_pass}/{total_pass+total_fail} passed", end="")
    if total_fail:
        print(f" ({total_fail} FAILED)")
    else:
        print()
    sys.exit(1 if total_fail else 0)


if __name__ == "__main__":
    main()
