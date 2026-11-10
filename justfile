fmt_paths := "firmware/lib firmware/src/sim firmware/test"
fmt_files := "\\( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \\)"
suite := "-e test-robot -e test-route -e test-trial -e test-metrics"
runtime := "-e test-runtime -e test-physics"

default:
    @just --list

build-native:
    pio run -d firmware -e sim {{suite}} {{runtime}}

# These contracts are not evidence of obstacle bypass or track completion.
test:
    pio run -d firmware -e test-robot -e test-route -e test-trial -t exec
    pio run -d firmware -e test-metrics -t exec --program-arg=--self-test

test-sim-runtime:
    pio run -d firmware {{runtime}} -t exec

sim:
    pio run -d firmware -e sim -t exec

check: format-check lint test test-sim-runtime build-native

format:
    find {{fmt_paths}} -type f {{fmt_files}} -print0 | xargs -0 -r clang-format -i

format-check:
    find {{fmt_paths}} -type f {{fmt_files}} -print0 | xargs -0 -r clang-format --dry-run --Werror

# Keep each compilation database separate: engine tests must not gain controller sources.
lint:
    #!/usr/bin/env bash
    set -euo pipefail
    for target in sim test-robot test-route test-trial test-metrics test-runtime test-physics; do
      rm -rf "firmware/.pio/build/$target/compile_commands.json"
      pio run -d firmware -e "$target" -t compiledb
      cp firmware/compile_commands.json "firmware/.pio/build/$target/compile_commands.json"
      (cd firmware && python3 -c 'import json, subprocess, sys; entries=json.load(open(sys.argv[1]+"/compile_commands.json")); subprocess.run(["clang-tidy", "--warnings-as-errors=*", "-p", sys.argv[1]] + [entry["file"] for entry in entries], check=True)' ".pio/build/$target")
    done
    cp firmware/.pio/build/sim/compile_commands.json firmware/compile_commands.json

lint-esp32:
    pio check -d firmware -e esp32s3

compiledb:
    pio run -d firmware -e sim -t compiledb

sanitize:
    #!/usr/bin/env bash
    set -euo pipefail
    export PLATFORMIO_BUILD_DIR=.pio/sanitize
    export PLATFORMIO_BUILD_FLAGS='-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all'
    just test test-sim-runtime

clean:
    pio run -d firmware -e sim -e evaluate {{suite}} {{runtime}} -t clean
