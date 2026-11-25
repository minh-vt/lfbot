fmt_paths := "firmware/core firmware/esp32 firmware/test"
fmt_files := "\\( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \\)"

default:
    @just --list

build:
    pio run -d firmware -e esp32s3

upload:
    pio run -d firmware -e esp32s3 -t upload

test:
    pio run -d firmware -e test -t exec

format:
    find {{fmt_paths}} -type f {{fmt_files}} -print0 | xargs -0 -r clang-format -i

format-check:
    find {{fmt_paths}} -type f {{fmt_files}} -print0 | xargs -0 -r clang-format --dry-run --Werror

compiledb:
    pio run -d firmware -e esp32s3 -t compiledb

lint:
    pio run -d firmware -e test -t compiledb
    clang-tidy --warnings-as-errors=* -p firmware $(find firmware/core firmware/test -name '*.cpp')

check: format-check lint test build

sanitize:
    PLATFORMIO_BUILD_DIR=.pio/sanitize PLATFORMIO_BUILD_FLAGS="-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all" just test

clean:
    pio run -d firmware -e esp32s3 -e test -t clean
