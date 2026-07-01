#!/usr/bin/env bash
set -u

status=0
ran=()
skipped=()
failed=()

run_required() {
    local name="$1"
    shift
    echo "==> $name"
    if "$@"; then
        ran+=("$name: PASS")
    else
        ran+=("$name: FAIL")
        failed+=("$name")
        status=1
    fi
}

run_optional() {
    local name="$1"
    local probe="$2"
    shift 2
    if eval "$probe" >/dev/null 2>&1; then
        run_required "$name" "$@"
    else
        echo "==> $name: SKIP (missing tool)"
        skipped+=("$name")
    fi
}

run_required "release build" cargo build --release
run_required "format check" cargo fmt --check
run_required "clippy" cargo clippy --all-targets -- -D warnings
run_required "tests" cargo test -p libairspyhf

run_optional \
    "miri" \
    "cargo +nightly miri --version" \
    cargo +nightly miri test -p libairspyhf --lib

run_optional \
    "loom" \
    "cargo metadata --no-deps" \
    env RUSTFLAGS="--cfg loom" cargo test -p libairspyhf stop_protocol_retires_workers

run_optional \
    "kani" \
    "cargo kani --version" \
    cargo kani -p libairspyhf

run_optional \
    "careful" \
    "cargo +nightly careful --version" \
    cargo +nightly careful test -p libairspyhf

echo
echo "Summary"
for item in "${ran[@]}"; do
    echo "  $item"
done
for item in "${skipped[@]}"; do
    echo "  $item: SKIP"
done

if ((${#failed[@]} > 0)); then
    echo
    echo "Failed tiers: ${failed[*]}"
fi

exit "$status"
