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

# Kiem tra toan dien: format + lint (native + esp32) + test
check: format-check lint test

# Tu dong format toan bo ma nguon bang clang-format
format:
    clang-format -i firmware/lib/*/*.hpp firmware/lib/*/*.cpp firmware/src/*/*.hpp firmware/src/*/*.cpp firmware/test/*.cpp

# Kiem tra format ma nguon (tra ve loi neu chua format)
format-check:
    clang-format --dry-run --Werror firmware/lib/*/*.hpp firmware/lib/*/*.cpp firmware/src/*/*.hpp firmware/src/*/*.cpp firmware/test/*.cpp

# Kiem tra tinh ma nguon: clang-tidy cho native + cppcheck cho esp32
lint: lint-native lint-esp32

# clang-tidy cho code build duoc tren host (kernel, sim, test, header thuần esp32)
lint-native:
    clang-tidy -p firmware/compile_commands.json firmware/lib/*/*.cpp firmware/src/sim/*.cpp firmware/test/*.cpp

# cppcheck cho firmware ESP32-S3 (IDF không parse được bằng clang-tidy host)
lint-esp32:
    cd firmware && pio check -e esp32s3

# Tu dong ap dung cac fix cua clang-tidy va format lai code
lint-fix:
    clang-tidy -p firmware/compile_commands.json --fix --fix-errors firmware/lib/*/*.cpp firmware/src/sim/*.cpp firmware/test/*.cpp
    just format

# Tao lai database compile_commands.json cho linter va clangd (native + esp32)
compiledb:
    cd firmware && pio run -e lint -t compiledb
    python3 tools/gen_compiledb.py

# Chuan hoa sa ban tu anh thuc te AI (Vong loai 6000x1500, Chung ket 10000x1500)
track:
    python3 tools/normalize_ai_tracks.py

# Nap chuong trinh xuong chip ESP32-S3
upload:
    cd firmware && pio run -e esp32s3 -t upload

# Mo serial monitor ESP32-S3 (115200 baud)
monitor:
    cd firmware && pio device monitor -b 115200

# Don dep cache PlatformIO
clean:
    cd firmware && pio run -t clean
