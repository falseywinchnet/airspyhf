#!/bin/sh
set -eu

if [ "$#" -ne 2 ]; then
    echo "usage: $0 BASELINE_LIBRARY READABLE_LIBRARY" >&2
    exit 2
fi

baseline_library=$1
readable_library=$2
baseline_symbols=$(mktemp)
readable_symbols=$(mktemp)
cleanup()
{
    rm -f "$baseline_symbols" "$readable_symbols"
}
trap cleanup EXIT HUP INT TERM

extract_symbols()
{
    library=$1
    output=$2
    case "$(uname -s)" in
        Darwin)
            nm -gU "$library" |
                awk '$2 ~ /^[TDBS]$/ {print $3}' |
                sed 's/^_//' |
                awk '/^airspy_/ {print}' |
                sort > "$output"
            ;;
        *)
            nm -D --defined-only "$library" |
                awk '{print $3}' |
                awk '/^airspy_/ {print}' |
                sort > "$output"
            ;;
    esac
}

extract_symbols "$baseline_library" "$baseline_symbols"
extract_symbols "$readable_library" "$readable_symbols"

if ! diff -u "$baseline_symbols" "$readable_symbols"; then
    echo "readable driver export set differs from the C baseline" >&2
    exit 1
fi
