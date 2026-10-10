# Custom espidf component cho app ESP32 — chỉ biên dịch esp32/* (KHÔNG src/sim).
# lib/kernel được PlatformIO xử lý như component riêng (build_src_filter).
idf_component_register(
    SRCS
        "esp32/main.cpp"
        "esp32/sensors.cpp"
        "esp32/actuators.cpp"
    INCLUDE_DIRS
        "esp32"
        "../lib")
