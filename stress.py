#!/usr/bin/env python3
import multiprocessing as mp
import os
import time

DURATION_SECONDS = 30
RAM_FRACTION = 0.65
CHUNK_SIZE = 64 * 1024 * 1024
PAGE_SIZE = 4096


def available_memory() -> int:
    try:
        import psutil
        return psutil.virtual_memory().available
    except ImportError:
        if os.uname().sysname == "Darwin":
            import subprocess

            total = int(
                subprocess.check_output(["sysctl", "-n", "hw.memsize"]).strip()
            )
            return total // 2

        pages = os.sysconf("SC_AVPHYS_PAGES")
        page_size = os.sysconf("SC_PAGE_SIZE")
        return pages * page_size


def burn_cpu(deadline: float) -> None:
    value = 1
    while time.monotonic() < deadline:
        for i in range(1, 200_000):
            value = (value * 1664525 + i + 1013904223) & 0xFFFFFFFF


def pressure_memory(deadline: float, target_bytes: int) -> None:
    chunks = []
    allocated = 0

    try:
        while allocated < target_bytes and time.monotonic() < deadline:
            size = min(CHUNK_SIZE, target_bytes - allocated)
            block = bytearray(size)

            for offset in range(0, size, PAGE_SIZE):
                block[offset] = 1

            chunks.append(block)
            allocated += size

        while time.monotonic() < deadline:
            for block in chunks:
                for offset in range(0, len(block), PAGE_SIZE):
                    block[offset] = (block[offset] + 1) & 0xFF
    except MemoryError:
        while time.monotonic() < deadline:
            time.sleep(0.05)


def main() -> None:
    logical_cores = os.cpu_count() or 2
    cpu_workers = max(1, logical_cores // 2)
    memory_target = int(available_memory() * RAM_FRACTION)
    deadline = time.monotonic() + DURATION_SECONDS

    print(
        f"Running for {DURATION_SECONDS}s: "
        f"{cpu_workers}/{logical_cores} CPU threads, "
        f"{memory_target / 1024**3:.1f} GiB RAM target"
    )

    processes = [
        mp.Process(target=burn_cpu, args=(deadline,))
        for _ in range(cpu_workers)
    ]
    processes.append(
        mp.Process(target=pressure_memory, args=(deadline, memory_target))
    )

    for process in processes:
        process.start()

    try:
        for process in processes:
            process.join(DURATION_SECONDS + 5)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        for process in processes:
            if process.is_alive():
                process.terminate()
        for process in processes:
            process.join()

    print("Finished.")


if __name__ == "__main__":
    mp.set_start_method("spawn")
    main()
