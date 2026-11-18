idf_component_register(
    SRCS
      "esp32/main.cpp"
      "esp32/adc_inputs.cpp"
      "esp32/blackbox.cpp"
      "esp32/encoders.cpp"
      "esp32/imu.cpp"
      "esp32/motors.cpp"
      "esp32/status_led.cpp"
      "esp32/console.cpp"
      "esp32/tof.cpp"
      "esp32/tof_platform.cpp"
      "../lib/robot/motion.cpp"
      "../lib/robot/drive.cpp"
      "../lib/robot/maneuver.cpp"
      "../lib/robot/line.cpp"
      "../lib/robot/race.cpp"
      "../lib/robot/robot.cpp"
      "../third_party/vl53l1x/vl53l1x_api.c"
    INCLUDE_DIRS "../lib/robot" "esp32" "../third_party/vl53l1x"
    REQUIRES esp_adc esp_partition esp_driver_gpio esp_driver_i2c esp_driver_pcnt esp_driver_mcpwm
             esp_driver_rmt esp_timer)
# Vendor headers use GNU/C++20 constructs (designated initializers, anonymous
# structs), so -Wpedantic cannot be applied to this target. Host tests keep the
# strict flags; this target keeps everything else, including -Werror.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
