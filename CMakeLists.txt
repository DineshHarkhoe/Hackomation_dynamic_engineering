# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)
set(EXTRA_COMPONENT_DIRS "components/ahrs" "components/mpu9250")
set(COMPONENTS bootloader esptool_py main ahrs mpu9250)  # This reduces the number of redundant components included
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Hackomation_v1)
