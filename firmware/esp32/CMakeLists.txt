idf_component_register(
    SRCS
      "main.cpp"
      "console.cpp"
      "drivers/adc_inputs.cpp"
      "drivers/encoders.cpp"
      "drivers/imu.cpp"
      "drivers/motors.cpp"
      "drivers/status_led.cpp"
      "drivers/tof.cpp"
      "drivers/tof_platform.cpp"
      "storage/blackbox.cpp"
      "storage/course_store.cpp"
      "../core/motion.cpp"
      "../core/drive.cpp"
      "../core/line.cpp"
      "../core/pilot.cpp"
      "../core/course.cpp"
      "../core/robot.cpp"
      "../third_party/vl53l1x/vl53l1x_api.c"
    INCLUDE_DIRS "." "drivers" "storage" "../core" "../third_party/vl53l1x"
    REQUIRES esp_adc esp_partition esp_driver_gpio esp_driver_i2c esp_driver_pcnt esp_driver_mcpwm
             esp_driver_rmt esp_timer)
# Vendor headers use GNU/C++20 constructs (designated initializers, anonymous
# structs), so -Wpedantic cannot be applied to this target. Host tests keep the
# strict flags; this target keeps everything else, including -Werror.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
