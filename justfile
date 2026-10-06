# LF-BOT Command Runner (tuong tu npm scripts)

default:
    @just --list

# Chay mo phong Desktop Raylib
sim:
    cd firmware && pio run -e sim -t exec

# Chay toan bo unit tests headless
test:
    cd firmware && pio run -e test_core -t exec

# Chay kiem thu mo phong headless 2 luot (Map & Sprint)
sim-test:
    cd firmware && pio run -e sim_test -t exec

# Kiem tra toan dien: format + lint + test + sim-test
check: format-check test sim-test

# Tu dong format toan bo ma nguon bang clang-format
format:
    clang-format -i firmware/lib/*/*.hpp firmware/lib/*/*.cpp firmware/src/*/*.hpp firmware/src/*/*.cpp firmware/test/*.cpp

# Kiem tra format ma nguon (tra ve loi neu chua format)
format-check:
    clang-format --dry-run --Werror firmware/lib/*/*.hpp firmware/lib/*/*.cpp firmware/src/*/*.hpp firmware/src/*/*.cpp firmware/test/*.cpp

# Kiem tra tinh ma nguon bang clang-tidy
lint:
    clang-tidy -p firmware/compile_commands.json firmware/lib/*/*.cpp firmware/src/sim/*.cpp firmware/test/*.cpp

# Tu dong ap dung cac fix cua clang-tidy va format lai code
lint-fix:
    clang-tidy -p firmware/compile_commands.json --fix --fix-errors firmware/lib/*/*.cpp firmware/src/sim/*.cpp firmware/test/*.cpp
    just format

# Tao lai database compile_commands.json cho linter va clangd
compiledb:
    cd firmware && pio run -e sim -t compiledb

# Sinh sa ban procedural generation (vd: just track --complexity 5 --seed 42)
track *args:
    python3 firmware/assets/generate_track.py {{ args }}

# Nap chuong trinh xuong chip ESP32-S3
upload:
    cd firmware && pio run -e esp32s3 -t upload

# Mo serial monitor ESP32-S3 (115200 baud)
monitor:
    cd firmware && pio device monitor -b 115200

# Don dep cache PlatformIO
clean:
    cd firmware && pio run -t clean
